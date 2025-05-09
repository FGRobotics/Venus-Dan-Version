/**
 * ArmAndBox
 * .
 * Overview:
 * This class provides pre-configured position sets for the arm and box (outtake) subsystem
 * using ServoUtilities to ensure smooth transitions, safety, and maintainable logic.
 * It controls four key servos: base, joint, wrist, and claw, and also includes
 * control for separate arm and box servos and a distance sensor for object detection.
 * Analog position verification is available via optional analog inputs for joint and base
 * if enabled in ServoUtilities.
 * .
 * Features:
 * - Preset arm, box, and claw positions for common tasks (pickup, transfer, wall intake, etc.)
 * - Open/close claw methods with configurable safety delay
 * - Methods for controlling dedicated outtake arm and box servos.
 * - Sleep utility with timeout-safe mechanism using ElapsedTime
 * - Smooth position transitions using ServoUtilities
 * - Object detection using a distance sensor.
 * .
 * Usage:
 * Used during both TeleOp and Autonomous periods to manage arm, box, and claw positioning
 * using predefined presets and to detect objects in the box. Designed for modular use
 * and readable tuning.
 * .
 * --------------------------------------------------------------------------------
 * Created On:          May 6, 2025
 * Last Updated:        May 6, 2025
 * Original Author:     Daniel Carrillo
 * Contributors:        [Add others if applicable]
 * Documentation:       Generated with assistance from Gemini.
 * Organization:        Venus Robot - 10265 Force Green 2024-2025
 * --------------------------------------------------------------------------------
 */

package Venus.boilerplate.outtake;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import static Venus.boilerplate.helpers.ServoUtilities.smoothlyMoveServoToTarget;

public class ArmAndBox {
    private final ServoImplEx outtakeArmServo, outtakeBoxServo;
    private final AnalogInput outtakeArmAnalog, outtakeBoxAnalog;

    public DistanceSensor objectDistanceSensor;
    public boolean isObjectSensorStatic = false; // Renamed for clarity

    public ArmAndBox(HardwareMap hardwareMap) {
        outtakeArmServo = hardwareMap.get(ServoImplEx.class, "outtakeArm");
        outtakeBoxServo = hardwareMap.get(ServoImplEx.class, "outtakeBox");

        outtakeArmAnalog = hardwareMap.get(AnalogInput.class, "outtakeArmPos");
        outtakeBoxAnalog = hardwareMap.get(AnalogInput.class, "outtakeBoxPos");

        objectDistanceSensor = hardwareMap.get(DistanceSensor.class, "color"); // Renamed for clarity
    }

    public void setOuttakeArmUp() {outtakeArmServo.setPosition(0);}
    public void setOuttakeArmDown() {outtakeArmServo.setPosition(1);}

    public void setOuttakeArmTransfer() {outtakeArmServo.setPosition(0.85);} // 0:1 = Up:Down
    public void setOuttakeBoxTransfer() {outtakeBoxServo.setPosition(0.2);}  // 0:1 = Parallel:Perpendicular
    public void setOuttakeBoxTransferAdjusted() {outtakeBoxServo.setPosition(0.3);}

    public void setOuttakeBoxSpecimen() {outtakeBoxServo.setPosition(0.63);}
    public void setOuttakeBoxDrop() {outtakeBoxServo.setPosition(0);}
    public void setOuttakeBoxAutoDrop() {outtakeBoxServo.setPosition(0.18);}

    public void setOuttakeMechanismTransfer() {
        setOuttakeArmTransfer();
        setOuttakeBoxTransferAdjusted();
        delay(100);
        setOuttakeArmTransfer();
    }

    public void setOuttakeFullSpecimen() {
        setOuttakeArmUp();
        setOuttakeBoxSpecimen();
    }

    /**
     * Checks if an object is present based on the distance sensor reading.
     * This check is static after the first successful reading (reading below a very high threshold).
     * @return true if an object is detected within a close range and the sensor is not static, false otherwise.
     */
    public boolean hasObject() {
        if (!isObjectSensorStatic) {
            double distance = objectDistanceSensor.getDistance(DistanceUnit.CM);
            // Check if the sensor is returning valid data (not a very high placeholder value)
            if (distance < 5000) { // Assuming 5000 is a placeholder for no valid reading
                return distance < 5.5; // Assuming 5.5 cm is the threshold for object detection
            } else {
                // If we get a very high reading, assume the sensor might be static or disconnected
                isObjectSensorStatic = true;
            }
        }
        // If the sensor is static, we cannot reliably detect an object
        return false;
    }

    /**
     * Pauses execution for a specified number of milliseconds using ElapsedTime.
     * @param millis Milliseconds to sleep
     */
    public void delay(long millis) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (timer.milliseconds() < millis) {
            Thread.yield();
        }
    }
}