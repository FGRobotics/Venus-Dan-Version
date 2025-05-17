package Venus.boilerplate.tele;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import Venus.boilerplate.intake.Claw;
import Venus.boilerplate.outtake.ArmAndBox;
import Venus.boilerplate.helpers.SlideUtilities;
import Venus.boilerplate.helpers.LimitSwitchUtilities;
import Venus.boilerplate.Chassis;
import Venus.boilerplate.helpers.ServoUtilities;

@TeleOp(name = "DanMain")
public class DanMain extends LinearOpMode {

    private boolean chassisReversed = true;
    private boolean transferReady = false;

    private enum ClawState {SCAN, PICKUP, RETRACT, TRANSFER}
    private enum SubAssemblyState { INTAKE, TRANSFER, OUTTAKE }
    private enum DrivingState { FULL_SPEED, SLOW_SPEED, PRECISION_SPEED}
    public enum OuttakeState {SPECIMEN_DELIVER, DELIVER, BOX_SAMPLE_DROP}

    private ClawState clawState = ClawState.SCAN;
    OuttakeState outtakeState = OuttakeState.DELIVER;
    private DrivingState driveState = DrivingState.FULL_SPEED;
    private SubAssemblyState subState = SubAssemblyState.INTAKE;

    private Chassis chassis;
    private Claw intakeLogic;
    private ServoImplEx wristServo;
    private ArmAndBox outtakeLogic;
    private SlideUtilities intakeSlide, outtakeSlide;

    @Override
    public void runOpMode() throws InterruptedException {
        chassis = new Chassis(hardwareMap);

        wristServo  = hardwareMap.get(ServoImplEx.class, "wrist");

        intakeLogic = new Claw(hardwareMap);
        LimitSwitchUtilities intakeLimitSwitch = new LimitSwitchUtilities(hardwareMap, "leftIntakeLimitSwitch", "rightIntakeLimitSwitch");
        intakeSlide = new SlideUtilities(hardwareMap, "intakeSlideMotor", DcMotorEx.Direction.REVERSE,10,0,1100, 1000, 1.5, 3000, 0.05, intakeLimitSwitch);

        outtakeLogic = new ArmAndBox(hardwareMap);
        LimitSwitchUtilities outtakeLimitSwitch = new LimitSwitchUtilities(hardwareMap, "leftOuttakeLimitSwitch", "rightOuttakeLimitSwitch");
        outtakeSlide = new SlideUtilities(hardwareMap, "leftOuttakeSlideMotor", "rightOuttakeSlideMotor", DcMotorEx.Direction.FORWARD,10,0,1500, 1500, 0.5, 3000, 0.5, outtakeLimitSwitch);

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
                chassis.driveTeleOp(-gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, 1.0, 1.0, 1.0, chassisReversed, gamepad1.dpad_left, gamepad1.dpad_right, gamepad1.options);
                //chassis.teleDrive(-gamepad1.left_stick_x, gamepad1.right_stick_x, -gamepad1.left_stick_y, gamepad1.right_trigger, 1.0, 1.0, chassisReversed, gamepad1.dpad_left, gamepad1.dpad_right);
                break;
            case SLOW_SPEED:
                chassis.driveTeleOp(-gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, 0.2, 0.2, 0.2, chassisReversed, gamepad1.dpad_left, gamepad1.dpad_right, gamepad1.options);
                break;
            case PRECISION_SPEED:
                chassis.driveTeleOp(-gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, 0.5, 0.5, 0.5, chassisReversed, gamepad1.dpad_left, gamepad1.dpad_right, gamepad1.options);
                break;
        }
    }

    private void handleSubAssembly() {
        switch (subState) {
            case INTAKE:
                handleIntake();
                break;
            case TRANSFER:
                intakeSlide.homeSlide(opModeIsActive());
                outtakeSlide.homeSlide(opModeIsActive());
                handleTransfer();
                break;
            case OUTTAKE:
                handleOuttake();
                break;
        }
    }

    private void handleGamePad1Inputs() {
        if (gamepad1.ps) {
            // Toggle between SLOW_SPEED and FULL_SPEED when PS button is pressed
            if (driveState == DrivingState.FULL_SPEED) {
                driveState = DrivingState.SLOW_SPEED; // Return to normal speed
            }
        } else if (gamepad1.right_trigger > 0.2) {
            // If right trigger is held, drive in precision mode
            driveState = DrivingState.PRECISION_SPEED;
        } else {
            // Default to FULL_SPEED when none of the above conditions are met
            driveState = DrivingState.FULL_SPEED;
        }

        if (gamepad1.touchpad) chassis.rotate180Degrees();
        if (gamepad1.right_bumper && gamepad1.left_bumper) Chassis.imu.resetIMU();
        if (gamepad1.circle && getRuntime() > 0.2) {chassisReversed = !chassisReversed; resetRuntime();}
    }

    private void handleIntake() {
        outtakeSlide.setMotorVelocity(0);
        outtakeLogic.setOuttakeMechanismTransfer();
        intakeSlide.slideControl(-gamepad2.left_stick_y, gamepad2.right_trigger > 0.2);

        if (gamepad2.ps && getRuntime() >  0.25) {intakeSlide.goToMinExtension(opModeIsActive()); resetRuntime();}
        if (gamepad2.dpad_up && getRuntime() >  0.25) {intakeSlide.goToMaxExtension(opModeIsActive()); resetRuntime();}
        if (gamepad2.dpad_down && getRuntime() >  0.25) {intakeSlide.moveToPosition((600-50), opModeIsActive()); resetRuntime();} //tends to overshoot by 50 ticks

        updateClawControls();
    }

    private void updateClawControls() {
        boolean xPressed = gamepad2.cross && getRuntime() > 0.2;
        boolean trianglePressed = gamepad2.triangle && getRuntime() > 0.2;

        if (xPressed || trianglePressed) {
            resetRuntime();
            switch (clawState) {
                case SCAN:
                    if (trianglePressed) intakeLogic.specimenPickupSequence();  // Specimen-specific pickup
                    else intakeLogic.pickup();  // Normal pickup
                    clawState = ClawState.PICKUP;
                    break;

                case PICKUP:
                    intakeLogic.closeClaw();
                    intakeLogic.setIntakeRetract();
                    intakeLogic.delay(1);
                    clawState = ClawState.RETRACT;
                    break;

                case RETRACT:
                    transferReady = false;
                    subState = SubAssemblyState.TRANSFER;
                    clawState = ClawState.TRANSFER;
                    break;
            }
        }

        if (gamepad2.circle) {
            intakeLogic.setNoWristScan();
            clawState = ClawState.SCAN;
        }

        if (gamepad2.right_stick_button) {
            intakeLogic.setSpecimenFloorScan();
            clawState = ClawState.SCAN;
        }

       ServoUtilities.manualPositionIncrement(wristServo, gamepad2.right_bumper, gamepad2.left_bumper, 0.15 );

        if (gamepad2.square && getRuntime() > 0.3) {
            resetRuntime();
            transitionToOuttake();
        }
    }

    private void handleOuttake() {
        intakeSlide.setMotorVelocity(0);
        intakeLogic.setIntakeRetract();
        outtakeSlide.slideControl(-gamepad2.left_stick_y, gamepad2.right_trigger > 0.2);

        if (gamepad2.dpad_down && getRuntime() >  0.25) {
            outtakeLogic.setOuttakeSpecimenDeliver();
            outtakeSlide.moveToPosition(800, opModeIsActive());
            resetRuntime();
        }

        if (gamepad2.ps && getRuntime() >  0.25) {outtakeSlide.goToMinExtension(opModeIsActive());  resetRuntime();}
        if (gamepad2.dpad_up && getRuntime() >  0.25) {outtakeSlide.goToMaxExtension(opModeIsActive());  resetRuntime();}

        boolean xPressed = gamepad2.cross && getRuntime() > 0.2;
        boolean circlePressed = gamepad2.circle && getRuntime() > 0.2;

        if (xPressed || circlePressed) {
            resetRuntime();
            switch (outtakeState) {
                case DELIVER:
                    if (circlePressed) {
                        outtakeLogic.setOuttakeSpecimenDeliver();
                        outtakeState = OuttakeState.SPECIMEN_DELIVER;
                    } else {
                        outtakeLogic.setOuttakeBoxSampleDrop();
                        outtakeState = OuttakeState.BOX_SAMPLE_DROP;
                    }
                    break;

                case BOX_SAMPLE_DROP:
                    if (circlePressed) {
                        outtakeLogic.setOuttakeSpecimenDeliver();
                        outtakeState = OuttakeState.SPECIMEN_DELIVER;
                    } else {
                        outtakeLogic.setOuttakeSampleDeliver();
                        outtakeState = OuttakeState.DELIVER;
                    }
                    break;

                case SPECIMEN_DELIVER:
                    outtakeLogic.setOuttakeSampleDeliver();
                    outtakeState = OuttakeState.DELIVER;
                    break;
            }
        }

        if (gamepad2.square && getRuntime() > 0.3) {
            resetRuntime();
            transitionToIntake();
        }
    }

    private void handleTransfer() {
        if (gamepad2.circle) intakeLogic.openClaw();

        if (!transferReady) {
            outtakeLogic.setOuttakeMechanismTransfer();
            intakeLogic.setIntakeTransfer();
            transferReady = true;
        }
        if (outtakeLogic.hasObject() || gamepad2.touchpad) {
            telemetry.addLine("Object detected or transfer override → OUTTAKE");
            transitionToOuttake();
        } else if (gamepad2.options) {
            telemetry.addLine("Returning to INTAKE (manual override)");
            transitionToIntake();
        } else {
            telemetry.addLine("Waiting for object detection or override...");
        }
    }

    private void transitionToIntake() {
        intakeLogic.setIntakeScan();

        intakeSlide.homeSlide(opModeIsActive());
        outtakeSlide.homeSlide(opModeIsActive());

        clawState = ClawState.SCAN;
        subState = SubAssemblyState.INTAKE;
    }

    private void transitionToOuttake() {
        outtakeLogic.setOuttakeArmUp();
        outtakeLogic.setOuttakeBoxSample();
        outtakeLogic.setOuttakeSampleDeliver();

        intakeSlide.homeSlide(opModeIsActive());
        //outtakeSlide.homeSlide(opModeIsActive());

        outtakeState = OuttakeState.DELIVER;
        subState = SubAssemblyState.OUTTAKE;
    }

    private void displayTelemetry() {
        telemetry.addData("Chassis Reversed:", chassisReversed);
        telemetry.addData("Drive State:", driveState);
        telemetry.addData("Outtake State:", outtakeState);
        telemetry.addLine();
        telemetry.addData("Subsystem State:", subState);
        telemetry.addData("Transfer Ready:", transferReady);
        telemetry.addLine();
        telemetry.addData("Claw State:", clawState);
        telemetry.addLine();
        telemetry.addData("Intake Slide Position:", intakeSlide.getMotor1().getCurrentPosition());
        telemetry.addData("Intake Motor Velocity:", intakeSlide.getMotor1().getVelocity());
        telemetry.addData("Left Limit:", intakeSlide.limitSwitchUtils.isLeftSwitchPressed());
        telemetry.addData("Right Limit:", intakeSlide.limitSwitchUtils.isRightSwitchPressed());
        telemetry.addData("Intake Docked:", intakeSlide.limitSwitchUtils.areBothSwitchesPressed());
        telemetry.addLine();
        telemetry.addData("Outtake Motor Velocity:", outtakeSlide.getMotor1().getVelocity());
        telemetry.addData("Left Limit:", outtakeSlide.limitSwitchUtils.isLeftSwitchPressed());
        telemetry.addData("Right Limit:", outtakeSlide.limitSwitchUtils.isRightSwitchPressed());
        telemetry.addData("Outtake Docked:", outtakeSlide.limitSwitchUtils.areBothSwitchesPressed());
        telemetry.addLine();
        telemetry.addData("Left Outtake Slide Position:", outtakeSlide.getMotor1().getCurrentPosition());
        telemetry.addData("Right Outtake Slide Position:", outtakeSlide.getMotor2().getCurrentPosition());
        telemetry.addData("Outtake Slide Position:", (outtakeSlide.getMotor1().getCurrentPosition() + outtakeSlide.getMotor2().getCurrentPosition()) / 2);
        telemetry.addLine();

        telemetry.update();
    }
}
