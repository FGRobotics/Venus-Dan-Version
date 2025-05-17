/**
 * ServoUtilities.java
 * .
 * Overview:
 * A utility class for managing and manipulating ServoImplEx objects with smooth incremental movements,
 * analog position verification, servo enable/disable controls, and range scaling. Designed for modular
 * use in TeleOp and Autonomous modes, it ensures precision and safety when handling sensitive or
 * high-load mechanisms like arms, joints, and wrists. This class provides a reusable and readable way
 * to control servo hardware in FTC robotics applications.
 * .
 * Features:
 * - Smooth position transitions with damping
 * - Analog feedback verification with voltage normalization
 * - Range scaling and PWM enable/disable controls
 * - Input debouncing for manual servo tuning
 * - Timeout-based safety mechanisms
 * .
 * Use Case:
 * Built for FTC robotics applications using ServoImplEx and servos with analog sensors like the
 * Axon Series Servos to fine-tune and automate actuator positioning with accuracy and safety.
 * .
 * --------------------------------------------------------------------------------
 * Created On:          May 5, 2025
 * Last Updated:        May 15, 2025
 * Original Author:     Daniel Carrillo
 * Contributors:        [Add others if applicable]
 * Documentation:       Generated with assistance from OpenAI's ChatGPT
 * Organization:        Venus Robot - 10265 Force Green 2024-2025
 * --------------------------------------------------------------------------------
 */

package Venus.boilerplate.helpers;

import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.AnalogInput;

import java.util.HashMap;
import java.util.Map;

public class ServoUtilities {

    private static final double DEBOUNCE_SECONDS = 0.10;
    private static final Map<ServoImplEx, ElapsedTime> servoTimers = new HashMap<>();

    private static final long   SERVO_MOVEMENT_DELAY_MS = 1;
    private static final double ANALOG_POSITION_TOLERANCE = 0.10;
    private boolean isRunning = false;
    private double targetPosition;

    /**
     * Disables the PWM signal for the given servos to reduce power consumption.
     * @param servos One or more ServoImplEx objects to disable.
     */
    public static void disableServos(ServoImplEx... servos) {
        for (ServoImplEx servo : servos) {
            servo.setPwmDisable();
        }
    }

    /**
     * Enables the PWM signal for the given servos, allowing them to move again.
     * @param servos One or more ServoImplEx objects to enable.
     */
    public static void enableServos(ServoImplEx... servos) {
        for (ServoImplEx servo : servos) {
            servo.setPwmEnable();
        }
    }

    /**
     * Adjusts a servo's position based on button input with debounce protection.
     * @param increaseButton True if the increment button is pressed.
     * @param decreaseButton True if the decrement button is pressed.
     * @return The new servo position after applying input.
     */
    public static void manualPositionIncrement(ServoImplEx servo, boolean increaseButton, boolean decreaseButton, double increment) {
        servoTimers.putIfAbsent(servo, new ElapsedTime());
        ElapsedTime timer = servoTimers.get(servo);

        if (timer.seconds() > DEBOUNCE_SECONDS) {
            double currentPos = servo.getPosition();
            double newPos = currentPos;

            if (increaseButton) {
                newPos += increment;
                timer.reset();
            } else if (decreaseButton) {
                newPos -= increment;
                timer.reset();
            }

            // Clamp between 0.0 and 1.0
            newPos = Math.max(0.0, Math.min(1.0, newPos));
            servo.setPosition(newPos);
        }
    }

    /**
     * Smoothly moves the servo to the target position in small increments,
     * optionally verifying analog feedback, with a timeout.
     * @param servo The servo to move.
     * @param analogSensor The analog input for position verification.
     * @param targetPosition The desired position to reach.
     * @param verifyAnalog If true, verifies movement with analog sensor.
     * @param timeoutSeconds Maximum time allowed to complete the movement.
     */
    public void smoothlyMoveServoToTarget(ServoImplEx servo, AnalogInput analogSensor, double targetPosition, double servoIncrement, double servoDampener, boolean verifyAnalog, double timeoutSeconds) {
            if (isRunning) return; // Prevent multiple motions at once
            this.targetPosition = Range.clip(targetPosition, 0.00, 1.00);
            Thread motionThread = new Thread(() -> {
                isRunning = true;
                double currentPosition = servo.getPosition();

                while (Math.abs(currentPosition - targetPosition) > 0.01) {
                    double step = (targetPosition > currentPosition) ? servoIncrement : -servoIncrement;
                    if (Math.abs(targetPosition - currentPosition) < 0.2) {
                        step /= servoDampener;
                    }

                    currentPosition += step;
                    currentPosition = Range.clip(currentPosition, 0.0, 1.0);
                    servo.setPosition(currentPosition);

                    try {
                        Thread.sleep(1);
                    } catch (InterruptedException e) {
                        Thread.currentThread().interrupt();
                        break;
                    }
                }
                servo.setPosition(targetPosition); // snap to final target
                isRunning = false;

                if (verifyAnalog) {
                    waitForAnalogToReachTarget(analogSensor, targetPosition, timeoutSeconds);
                }
            });
            motionThread.start();
        }

    /**
     * Waits until the analog sensor indicates that the servo has reached the target position, or until the timeout expires.
     * @param analogSensor The analog input connected to the servo.
     * @param targetPosition The target normalized position (0.0 to 1.0).
     * @param timeoutSeconds The maximum wait time before aborting.
     */
    private static void waitForAnalogToReachTarget(AnalogInput analogSensor, double targetPosition, double timeoutSeconds) {
        ElapsedTime timeoutTimer = new ElapsedTime();
        while (!isAnalogSensorAtTargetPosition(analogSensor, targetPosition) && timeoutTimer.seconds() < timeoutSeconds) {
            Thread.yield(); // Avoids tight CPU loop
            // Optional: Add telemetry or method of visual verification here if needed
        }
    }

    /**
     * Checks if the analog sensor's current voltage corresponds to the target position, within an acceptable tolerance.
     * @param analogSensor The analog input to check.
     * @param targetPosition The normalized target position (0.0 to 1.0).
     * @return True if the sensor is within tolerance of the target.
     */
    private static boolean isAnalogSensorAtTargetPosition(AnalogInput analogSensor, double targetPosition) {
        double voltage = analogSensor.getVoltage();
        double currentPosition = normalizeAnalogVoltagePosition(analogSensor, voltage);
        return Math.abs(currentPosition - targetPosition) <= ANALOG_POSITION_TOLERANCE;
    }

    /**
     * Converts an analog sensor voltage reading to a normalized position value (0.0 to 1.0), based on calibrated voltage ranges for known sensor types.
     * Make sure the voltage ranges are accurate for your specific servos, otherwise the analog feedback may not be accurate.
     * @param analogSensor The analog input device.
     * @param voltage The voltage reading from the sensor.
     * @return The normalized position from 0.0 to 1.0.
     */
    private static double normalizeAnalogVoltagePosition(AnalogInput analogSensor, double voltage) {
        double minVoltage = 0.0, maxVoltage = 3.3;
        switch (analogSensor.getDeviceName()) {
            case "basePos":  minVoltage = 1.38; maxVoltage = 1.94; break; // Make sure to use a testing program to check the voltage ranges for your specific servo. Example: 1.38 when servo at 0; 1.94 when servo at 1.
            case "jointPos": minVoltage = 0.53; maxVoltage = 2.54; break;
            case "wristPos": minVoltage = 0.31; maxVoltage = 2.98; break;
        }
        return (voltage - minVoltage) / (maxVoltage - minVoltage);
    }
}
