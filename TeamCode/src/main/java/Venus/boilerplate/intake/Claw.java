/**
 * Claw.java
 * .
 * Overview:
 * This class provides pre-configured position sets for the claw subsystem using ServoUtilities
 * to ensure smooth transitions, safety, and maintainable logic. It controls four key servos:
 * base, joint, wrist, and claw. Analog position verification is available via optional
 * analog inputs for joint and base if enabled in ServoUtilities.
 * .
 * Features:
 * - Preset claw positions for common tasks (pickup, transfer, wall intake, etc.)
 * - Open/close claw methods with configurable safety delay
 * - Sleep utility with timeout-safe mechanism using ElapsedTime
 * - Smooth position transitions using ServoUtilities
 * .
 * Usage:
 * Used during both TeleOp and Autonomous periods to manage arm and claw positioning
 * using predefined presets. Designed for modular use and readable tuning.
 * .
 * --------------------------------------------------------------------------------
 * Created On:          May 6, 2025
 * Last Updated:        May 15, 2025
 * Original Author:     Daniel Carrillo
 * Contributors:        [Add others if applicable]
 * Documentation:       Generated with assistance from OpenAI's ChatGPT
 * Organization:        Venus Robot - 10265 Force Green 2024-2025
 * --------------------------------------------------------------------------------
 */

package Venus.boilerplate.intake;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.AnalogInput;
import Venus.boilerplate.helpers.ServoUtilities;

public class Claw {
    private final ServoUtilities servoUtils = new ServoUtilities();
    public final  ServoImplEx baseServo, jointServo, wristServo, clawServo;
    private final AnalogInput baseAnalog;
    private final AnalogInput jointAnalog;
    private final AnalogInput wristAnalog;

    public Claw(HardwareMap hardwareMap) {
        baseServo   = hardwareMap.get(ServoImplEx.class, "base");
        jointServo  = hardwareMap.get(ServoImplEx.class, "joint");
        wristServo  = hardwareMap.get(ServoImplEx.class, "wrist");
        clawServo   = hardwareMap.get(ServoImplEx.class, "claw");

        baseAnalog  = hardwareMap.get(AnalogInput.class, "basePos");
        jointAnalog = hardwareMap.get(AnalogInput.class, "jointPos");
        wristAnalog = hardwareMap.get(AnalogInput.class, "wristPos");
    }

/*
Claw: 0:1 = Open:Close
Wrist: 0:1 = PowerSide:VoltageSide (Purple)
Joint: 0:1 =        (Yellow)
Base: 0:1 = Down:Up (Red)
 */
    public void setWristNeutral() {wristServo.setPosition(0.66);}
    public void closeClaw()       {clawServo.setPosition(1.00);}
    public void openClaw()        {clawServo.setPosition(0.00);}

    public void pickup() {
        setIntakePickup();
        delay(150);
        closeClaw();
    }

    public void specimenPickupSequence() {
        setSpecimenFloorPickUp();
        closeClaw();
    }

    public void setIntakeScan() {
        setWristNeutral();
        servoUtils.smoothlyMoveServoToTarget(jointServo, jointAnalog, 0.91, 0.05, 20, false, -1);
        servoUtils.smoothlyMoveServoToTarget(baseServo, baseAnalog, 0.37, 0.1, 10,true, -1);
        openClaw();
    }

    public void setNoWristScan() {
        servoUtils.smoothlyMoveServoToTarget(jointServo, jointAnalog, 0.91, 0.05, 20, false, -1);
        servoUtils.smoothlyMoveServoToTarget(baseServo, baseAnalog, 0.37, 0.1, 10, true, -1);
        openClaw();
    }

    public void setIntakePickup() {
        openClaw();
        servoUtils.smoothlyMoveServoToTarget(jointServo, jointAnalog, 0.83, 0.1, 5, false, -1);
        servoUtils.smoothlyMoveServoToTarget(baseServo, baseAnalog, 0.18, 0.1, 5,true, -1);
    }

    public void setIntakeRetract() {
        setWristNeutral();
        servoUtils.smoothlyMoveServoToTarget(jointServo, jointAnalog, 0.65, 0.1, 5, false, -1);
        servoUtils.smoothlyMoveServoToTarget(baseServo, baseAnalog, 1.00, 0.1, 5, true, -1);
    }

    public void setIntakeTransfer() {
        setWristNeutral();
        servoUtils.smoothlyMoveServoToTarget(jointServo, jointAnalog, 0.01, 0.05, 10, false, -1);
        servoUtils.smoothlyMoveServoToTarget(baseServo, baseAnalog, 0.80, 0.1, 10, true, -1);
    }

    public void setSpecimenFloorScan() {
        openClaw();
        setWristNeutral();
        servoUtils.smoothlyMoveServoToTarget(jointServo, jointAnalog, 0.67, 0.05, 10, false, -1);
        servoUtils.smoothlyMoveServoToTarget(baseServo, baseAnalog, 0.15, 0.1, 10, true, -1);
    }

    public void setSpecimenFloorPickUp() {
        openClaw();
        setWristNeutral();
        servoUtils.smoothlyMoveServoToTarget(jointServo, jointAnalog, 0.67, 0.05, 10, false, -1);
        servoUtils.smoothlyMoveServoToTarget(baseServo, baseAnalog, 0.15, 0.1, 10, true, -1);
    }

    public void delay(long millis) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (timer.milliseconds() < millis) {
            Thread.yield();
        }
    }
}
