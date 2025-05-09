/**
 * TelopTele
 * .
 * Overview:
 * This class provides modular control logic for single or dual-slide mechanisms in FTC robots using DcMotorEx motors.
 * It includes smooth motion control with acceleration and deceleration ramping, docking/reset mechanisms using limit switches,
 * and utility functions for homing, range travel, and safety monitoring.
 * .
 * Features:
 * - Dynamic velocity scaling for acceleration and deceleration
 * - Docking detection with encoder reset using digital limit switches
 * - Configurable motor directions, tolerances, and safety thresholds
 * - Dual motor support with synchronized movement
 * - Homing with configurable backoff for accurate reset
 * - Timeout-aware movement with position checking
 * .
 * Usage:
 * Used during TeleOp and Autonomous to control vertical or horizontal slide mechanisms that require accurate extension,
 * reset, and limit switch docking. Designed to be modular and safe for high-load actuator control in arm or lift subsystems.
 * .
 * --------------------------------------------------------------------------------
 *  * Created On:          May 6, 2025
 *  * Last Updated:        May 9, 2025
 *  * Original Author:     Daniel Carrillo
 *  * Contributors:        [List of other contributors]
 *  * Documentation:       Generated with assistance from OpenAI's ChatGPT.
 *  * Organization:        Venus Robot - 10265 Force Green 2024-2025
 * --------------------------------------------------------------------------------
 */

package Venus.boilerplate.tele;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import Venus.boilerplate.intake.Claw;
import Venus.boilerplate.outtake.ArmAndBox;
import Venus.boilerplate.helpers.SlideUtilities;
import Venus.boilerplate.helpers.LimitSwitchUtilities;
import Venus.boilerplate.Chassis;

@TeleOp(name = "DanMain")
public class DanMain extends LinearOpMode {

    private boolean chassisReversed = true;
    private boolean transferReady = false;
    private boolean wallPickupOpen = false;

    private enum DrivingState { FULL_SPEED, SLOW_SPEED, PRECISION_SPEED}
    private enum SubAssemblyState { INTAKE, TRANSFER, OUTTAKE }
    private enum ClawState { RETRACT, SCAN, PICKUP }

    private DrivingState driveState = DrivingState.FULL_SPEED;
    private SubAssemblyState subState = SubAssemblyState.INTAKE;
    private ClawState clawState = ClawState.SCAN;

    private Claw intakeLogic;
    private ArmAndBox outtakeLogic;
    private SlideUtilities intakeSlide, outtakeSlide;
    private Chassis chassis;

    @Override
    public void runOpMode() throws InterruptedException {
        chassis = new Chassis(hardwareMap);

        intakeLogic = new Claw(hardwareMap);
        LimitSwitchUtilities intakeLimitSwitch = new LimitSwitchUtilities(hardwareMap, "leftIntakeSlide", "rightIntakeSlide");
        intakeSlide = new SlideUtilities(hardwareMap, "HorizontalSlideMotor", DcMotorEx.Direction.REVERSE, 3000, 1500, 15, 0, 1220, intakeLimitSwitch);

        outtakeLogic = new ArmAndBox(hardwareMap);
        LimitSwitchUtilities outtakeLimitSwitch = new LimitSwitchUtilities(hardwareMap, "leftOuttakeSlide", "rightOuttakeSlide");
        outtakeSlide = new SlideUtilities(hardwareMap, "leftVerticalSlideMotor", "rightVerticalSlideMotor", DcMotorEx.Direction.FORWARD, DcMotorEx.Direction.FORWARD, 3000, 1500, 15, 0, 1220, outtakeLimitSwitch);

        intakeSlide.homeSlide(opModeInInit());
        outtakeSlide.homeSlide(opModeInInit());

        waitForStart();

        intakeLogic.setIntakeScan();

        while (opModeIsActive()) {
            driveControl();
            handleGamePad1Inputs();
            handleSubAssembly();
            displayTelemetry();
        }
    }

    private void driveControl() {
        // Handle driving states
        switch (driveState) {
            case FULL_SPEED:
                chassis.teleDrive(-gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.right_trigger, 0.1, 1.0, chassisReversed, gamepad1.dpad_left, gamepad1.dpad_right);
                break;
            case SLOW_SPEED:
                chassis.teleDrive(-gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.right_trigger, 0.2, 0.2, chassisReversed, gamepad1.dpad_left, gamepad1.dpad_right);
                break;
            case PRECISION_SPEED:
                chassis.teleDrive(-gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.right_trigger, 0.5, 0.5, chassisReversed, gamepad1.dpad_left, gamepad1.dpad_right);
                break;
        }

    }

    private void handleSubAssembly() {
        switch (subState) {
            case INTAKE:
                handleIntake();
                break;
            case TRANSFER:
                handleTransfer();
                break;
            case OUTTAKE:
                handleOuttake();
                break;
        }
    }

    private void handleGamePad1Inputs() {
        if (gamepad1.dpad_up) {
            driveState = DrivingState.FULL_SPEED;
        } else if (gamepad1.options) {
            driveState = DrivingState.SLOW_SPEED;
        } else if (gamepad1.dpad_down) {
            driveState = DrivingState.PRECISION_SPEED;
        } else {
            driveState = DrivingState.FULL_SPEED;
        }
        if (gamepad1.circle && getRuntime() > 0.2) {
            chassisReversed = !chassisReversed;
            resetRuntime();
        }
        if (gamepad1.right_bumper && gamepad1.left_bumper) {
            Chassis.imu.resetIMU();
        }
    }

    private void handleIntake() {
        if (!outtakeSlide.checkAndResetIfDocked()) {
            outtakeSlide.goToMinExtension(opModeIsActive());
        }
        if (!intakeSlide.checkAndResetIfDocked()) {
            intakeSlide.goToMinExtension(opModeIsActive());
        }
        if (gamepad2.right_trigger > 0.2) {
            intakeSlide.getMotor1().setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            intakeSlide.getMotor1().setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }
        if (gamepad2.dpad_down) {
            intakeSlide.moveToPosition(600, opModeIsActive());
        } else if (gamepad2.dpad_up) {
            intakeSlide.goToMaxExtension(opModeIsActive());
        }
        outtakeLogic.setOuttakeMechanismTransfer();
        updateClawControls();
    }

    private void updateClawControls() {
        if (gamepad2.cross && getRuntime() > 0.23) {
            resetRuntime();
            switch (clawState) {
                case PICKUP:
                    intakeLogic.setIntakeScan();
                    clawState = ClawState.RETRACT;
                    break;
                case SCAN:
                    intakeLogic.pickup();
                    clawState = ClawState.PICKUP;
                    break;
                default:
                    transferReady = false;
                    subState = SubAssemblyState.TRANSFER;
                    break;
            }
        }

        if (gamepad2.triangle && getRuntime() > 0.25) {
            resetRuntime();
            if (wallPickupOpen) {
                intakeLogic.setSpecimenFloorScan();
                clawState = ClawState.SCAN;
            } else {
                intakeLogic.setSpecimenFloorPickUp();
                intakeLogic.setIntakeRetract();
                clawState = ClawState.RETRACT;
            }
            wallPickupOpen = !wallPickupOpen;
        }

        if (gamepad2.circle) {
            intakeLogic.setIntakeScan();
            clawState = ClawState.SCAN;
        }

        if (gamepad2.right_bumper && getRuntime() > 0.13) {
            resetRuntime();
            intakeLogic.wristServo.setPosition(intakeLogic.wristServo.getPosition() + 0.1);
        }
        if (gamepad2.left_bumper && getRuntime() > 0.13) {
            resetRuntime();
            intakeLogic.wristServo.setPosition(intakeLogic.wristServo.getPosition() - 0.1);
        }

        if (gamepad2.square && getRuntime() > 0.3) {
            resetRuntime();
            transitionToOuttake();
        }
    }

    private void handleOuttake() {
        if (!intakeSlide.checkAndResetIfDocked()) {
            intakeSlide.goToMinExtension(opModeIsActive());
        }
        if (!outtakeSlide.checkAndResetIfDocked()) {
            outtakeSlide.goToMinExtension(opModeIsActive());
        }
        if (gamepad2.right_trigger > 0.2) {
            outtakeSlide.getMotor1().setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            outtakeSlide.getMotor1().setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            outtakeSlide.getMotor2().setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            outtakeSlide.getMotor2().setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }
        if (gamepad2.dpad_down) {
            outtakeSlide.moveToPosition(800, opModeIsActive());
        } else if (gamepad2.dpad_up) {
            outtakeSlide.goToMaxExtension(opModeIsActive());
        }
        intakeLogic.setIntakeScan();
        outtakeLogic.setOuttakeArmUp();

        if (gamepad2.cross) {
            outtakeLogic.setOuttakeBoxDrop();
        } else {
            outtakeLogic.setOuttakeBoxSpecimen();
        }

        if (gamepad2.square && getRuntime() > 0.3) {
            resetRuntime();
            transitionToIntake();
        }
    }

    private void handleTransfer() {
        if (!transferReady) {
            outtakeLogic.setOuttakeMechanismTransfer();
            intakeLogic.setIntakeTransfer();
            transferReady = true;
        }
        if (outtakeLogic.hasObject() || gamepad2.touchpad) {
            telemetry.addLine("Object detected or transfer override → OUTTAKE");
            transitionToOuttake();
        } else if (gamepad2.triangle) {
            telemetry.addLine("Returning to INTAKE (manual override)");
            transitionToIntake();
        } else {
            telemetry.addLine("Waiting for object detection or override...");
        }
    }

    private void transitionToIntake() {
        subState = SubAssemblyState.INTAKE;
        intakeLogic.setIntakeScan();
        clawState = ClawState.SCAN;
    }

    private void transitionToOuttake() {
        subState = SubAssemblyState.OUTTAKE;
    }

    private void displayTelemetry() {
        telemetry.addData("Chassis Reversed", chassisReversed);
        telemetry.addData("Drive State", driveState);
        telemetry.addData("Subsystem State", subState);
        telemetry.addData("Claw State", clawState);
        telemetry.addData("Transfer Ready", transferReady);
        telemetry.update();
    }
}
