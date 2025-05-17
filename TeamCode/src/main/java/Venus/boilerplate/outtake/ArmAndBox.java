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
 * Last Updated:        May 15, 2025
 * Original Author:     Daniel Carrillo
 * Contributors:        [Add others if applicable]
 * Documentation:       Generated with assistance from Gemini.
 * Organization:        Venus Robot - 10265 Force Green 2024-2025
 * --------------------------------------------------------------------------------
 */

package Venus.boilerplate.outtake;

import Venus.boilerplate.helpers.ServoUtilities;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ArmAndBox {
    public final ServoImplEx outtakeArmServo, outtakeBoxServo;
    private final AnalogInput outtakeArmAnalog, outtakeBoxAnalog;

    public DistanceSensor objectDistanceSensor;
    public boolean isObjectSensorStatic = false;

    private final ServoUtilities servoUtils = new ServoUtilities();

    public ArmAndBox(HardwareMap hardwareMap) {
        outtakeArmServo = hardwareMap.get(ServoImplEx.class, "outtakeArm");
        outtakeBoxServo = hardwareMap.get(ServoImplEx.class, "outtakeBox");

        outtakeArmAnalog = hardwareMap.get(AnalogInput.class, "outtakeArmPos");
        outtakeBoxAnalog = hardwareMap.get(AnalogInput.class, "outtakeBoxPos");

        objectDistanceSensor = hardwareMap.get(DistanceSensor.class, "color");
    }

    public void setOuttakeSampleDeliver() {
        setOuttakeBoxSample();
        setOuttakeArmSample();
    }
    public void setOuttakeSpecimenDeliver() {
        setOuttakeBoxSpecimen();
        setOuttakeArmSpecimen();
    }
    public void setOuttakeMechanismTransfer() {
        setOuttakeBoxTransfer();
        setOuttakeArmTransfer();
    }

    // Arm Servo: 0:1 = Up:Down
    // Box Servo: 0:1 = Parallel:Perpendicular

    public void setOuttakeArmUp() {servoUtils.smoothlyMoveServoToTarget(outtakeArmServo, outtakeArmAnalog,0.01, 0.02, 2, false, -1);}
    public void setOuttakeArmDown() {outtakeArmServo.setPosition(1);}

    public void setOuttakeArmTransfer() {servoUtils.smoothlyMoveServoToTarget(outtakeArmServo, outtakeArmAnalog,0.90, 0.05, 5, false, -1);}
    public void setOuttakeBoxTransfer() {servoUtils.smoothlyMoveServoToTarget(outtakeBoxServo, outtakeBoxAnalog,0.40, 0.05, 10, false, -1);}
    //public void setOuttakeBoxTransferAdjusted() {outtakeBoxServo.setPosition(0.00);}

    public void setOuttakeArmSpecimen() {servoUtils.smoothlyMoveServoToTarget(outtakeArmServo, outtakeArmAnalog,0.16, 0.05, 10, false, -1);}
    public void setOuttakeBoxSpecimen() {servoUtils.smoothlyMoveServoToTarget(outtakeBoxServo, outtakeBoxAnalog,0.12, 0.05, 10, false, -1);}

    public void setOuttakeArmSample() {servoUtils.smoothlyMoveServoToTarget(outtakeArmServo, outtakeArmAnalog,0.01, 0.05, 10, false, -1);}
    public void setOuttakeBoxSample() {servoUtils.smoothlyMoveServoToTarget(outtakeBoxServo, outtakeBoxAnalog,0.01, 0.05, 10, false, -1);}
    public void setOuttakeBoxSampleDrop() {outtakeBoxServo.setPosition(1.00);}


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
}