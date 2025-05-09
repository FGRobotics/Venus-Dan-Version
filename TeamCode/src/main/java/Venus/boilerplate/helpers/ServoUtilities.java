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
 * Last Updated:        May 9, 2025
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

/**
 * Utility class for advanced servo control operations including smooth movement,
 * analog verification, range scaling, and servo state toggling.
 */
public class ServoUtilities {

    private static final double DEBOUNCE_SECONDS = 0.15;
    private static final long   SERVO_MOVEMENT_DELAY_MS = 1;
    private static final double ANALOG_POSITION_TOLERANCE = 0.10;
    private static final double SMOOTH_POSITION_INCREMENT = 0.02;
    private static final double END_RANGE_SLOWDOWN_DIVISOR = 5.0;
    private static final double END_RANGE_SLOWDOWN_THRESHOLD = 0.2;
    private static final double SERVO_RANGE_MIN = 0.0, SERVO_RANGE_MAX = 1.0;

    /**
     * Scales the servo's range of motion to standard min and max (0.0 to 1.0).
     * @param servos One or more ServoImplEx objects to scale.
     */
    public static void scaleServoRange(ServoImplEx... servos) {
        for (ServoImplEx servo : servos) {
            servo.scaleRange(SERVO_RANGE_MIN, SERVO_RANGE_MAX);
        }
    }

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
     * @param currentPosition The current position of the servo.
     * @param increase True if the increment button is pressed.
     * @param decrease True if the decrement button is pressed.
     * @param debounceTimer Timer to prevent rapid toggling.
     * @return The new servo position after applying input.
     */
    public static double manualPositionIncrement(double currentPosition, boolean increase, boolean decrease, ElapsedTime debounceTimer) {
        if (increase && debounceTimer.seconds() > DEBOUNCE_SECONDS) {
            currentPosition = Range.clip(currentPosition + SMOOTH_POSITION_INCREMENT, SERVO_RANGE_MIN, SERVO_RANGE_MAX);
            debounceTimer.reset();
        } else if (decrease && debounceTimer.seconds() > DEBOUNCE_SECONDS) {
            currentPosition = Range.clip(currentPosition - SMOOTH_POSITION_INCREMENT, SERVO_RANGE_MIN, SERVO_RANGE_MAX);
            debounceTimer.reset();
        }
        return currentPosition;
    }

    /**
     * Sets the servo's position only if the target differs significantly from the current.
     * @param servo The servo to update.
     * @param targetPosition The desired target position.
     */
    public static void updateServoPosition(ServoImplEx servo, double targetPosition) {
        if (Math.abs(servo.getPosition() - targetPosition) > 0.005) {
            servo.setPosition(targetPosition);
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
    public static void smoothlyMoveServoToTarget(ServoImplEx servo, AnalogInput analogSensor, double targetPosition, boolean verifyAnalog, double timeoutSeconds) {
        double currentPosition = servo.getPosition();
        ElapsedTime moveTimer = new ElapsedTime();

        while (Math.abs(currentPosition - targetPosition) > 0.01) {
            double step = (targetPosition > currentPosition) ? SMOOTH_POSITION_INCREMENT : -SMOOTH_POSITION_INCREMENT;
            if (Math.abs(targetPosition - currentPosition) <= END_RANGE_SLOWDOWN_THRESHOLD) {
                step /= END_RANGE_SLOWDOWN_DIVISOR;
            }
            currentPosition += step;
            currentPosition = Range.clip(currentPosition, SERVO_RANGE_MIN, SERVO_RANGE_MAX);
            servo.setPosition(currentPosition);
            delay();

            if (moveTimer.seconds() > timeoutSeconds) {
                break;
            }
        }
        if (verifyAnalog) {
            waitForAnalogToReachTarget(analogSensor, targetPosition, timeoutSeconds);
        }
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

    /**
     * Returns an array of formatted strings showing each servo's name and current position.
     * @param servos One or more ServoImplEx objects to read.
     * @return A String array of device names and positions.
     */
    public static String[] getAllServoPosition(ServoImplEx... servos) {
        String[] positionStrings = new String[servos.length];
        for (int i = 0; i < servos.length; i++) {
            positionStrings[i] = servos[i].getDeviceName() + " Pos: " + servos[i].getPosition();
        }
        return positionStrings;
    }

    /** Helper function to safely pause execution. */
    private static void delay() {
        try {
            Thread.sleep(ServoUtilities.SERVO_MOVEMENT_DELAY_MS);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}
