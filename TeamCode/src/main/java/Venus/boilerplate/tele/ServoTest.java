package Venus.boilerplate.tele;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * TeleOp Mode: ServoTest
 * Controls and monitors multiple servos for testing and tuning.
 * Includes Free-Move Mode and automated sequences.
 */
@TeleOp(name = "ServoTest", group = "TeleOp")
public class ServoTest extends LinearOpMode {

    // Constants for servo movement
    private static final double SERVO_INCREMENT = 0.01;
    private static final double SMOOTH_INCREMENT = 0.02;
    private static final double EPSILON = 0.001;
    private static final double MIN_POSITION = 0.0, MAX_POSITION = 1.0;
    private static final long SERVO_STEP_DELAY = 1;
    private static final double MIN_WAIT_TIME = 0.15;
    private static final double POSITION_TOLERANCE = 0.10; // Error range for position checking
    // Analog voltage ranges for each servo (observed values)
    private static final double BASE_MIN_VOLTAGE = 1.38;
    private static final double BASE_MAX_VOLTAGE = 1.94;
    private static final double JOINT_MIN_VOLTAGE = 0.53;
    private static final double JOINT_MAX_VOLTAGE = 2.54;
    private static final double WRIST_MIN_VOLTAGE = 0.31;
    private static final double WRIST_MAX_VOLTAGE = 2.98;
    // State tracking
    private final ElapsedTime timer = new ElapsedTime();
    // Servo and analog sensor declarations
    private ServoImplEx base, joint, wrist, claw, outtakeArm, outtakeBox;
    private AnalogInput baseAnalog, jointAnalog, wristAnalog;
    // Servo position tracking
    private double basePosition = 0.5, jointPosition = 0.5, wristPosition = 0.5, clawPosition = 0.5;
    private double outtakeArmPosition = 0.5, outtakeBoxPosition = 0.5;
    private boolean freeMoveMode = false;
    private boolean runningAutoAction = false;
    private boolean manualOverride = true;

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        telemetry.addData("Status", "Initialized. Press Start to run.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            toggleFreeMoveMode();
            if (!freeMoveMode) {
                if (!runningAutoAction) handleButtonPresses();
                if (manualOverride) controlServosManually();
                updateAllServos();
            }
            updateTelemetry();
            telemetry.update();
        }
    }

    // Initializes all servos and analog sensors
    private void initHardware() {
        base = hardwareMap.get(ServoImplEx.class, "base");
        joint = hardwareMap.get(ServoImplEx.class, "joint");
        wrist = hardwareMap.get(ServoImplEx.class, "wrist");
        claw = hardwareMap.get(ServoImplEx.class, "claw");
        outtakeArm = hardwareMap.get(ServoImplEx.class, "outtakeArm");
        outtakeBox = hardwareMap.get(ServoImplEx.class, "outtakeBox");

        baseAnalog = hardwareMap.get(AnalogInput.class, "basePos");
        jointAnalog = hardwareMap.get(AnalogInput.class, "jointPos");
        wristAnalog = hardwareMap.get(AnalogInput.class, "wristPos");

        scaleServos();
    }

    // Applies full movement range to all servos
    private void scaleServos() {
        ServoImplEx[] servos = {base, joint, wrist, claw, outtakeArm, outtakeBox};
        for (ServoImplEx servo : servos) {
            servo.scaleRange(MIN_POSITION, MAX_POSITION);
        }
    }

    // Toggle free-move mode using game-pad touchpad
    private void toggleFreeMoveMode() {
        if (gamepad1.touchpad) {
            freeMoveMode = !freeMoveMode;
            if (freeMoveMode) {
                disableServos();
                telemetry.addData("Free-Move Mode", "Enabled");
            } else {
                enableServos();
                telemetry.addData("Free-Move Mode", "Disabled");
            }
            waitForTouchpadRelease();
        }
    }

    // Waits until the touchpad is released to prevent bouncing
    private void waitForTouchpadRelease() {
        while (gamepad1.touchpad && opModeIsActive()) {
            telemetry.update();
        }
    }

    private void disableServos() {
        base.setPwmDisable();
        joint.setPwmDisable();
        wrist.setPwmDisable();
        claw.setPwmDisable();
    }

    private void enableServos() {
        base.setPwmEnable();
        joint.setPwmEnable();
        wrist.setPwmEnable();
        claw.setPwmEnable();
    }

    // Handles button inputs to trigger automated sequences
    private void handleButtonPresses() {
        // if (gamepad1.ps) runPickupSequence();
        // if (gamepad1.start) runDeliverSequence();
        // if (gamepad1.square) runScanSequence();
        // if (gamepad1.circle) runMiddleSequence();
    }

    // Allows user to control each servo manually using game-pad
    private void controlServosManually() {
        basePosition = controlSingleServo(basePosition, gamepad1.dpad_up, gamepad1.dpad_down);
        jointPosition = controlSingleServo(jointPosition, gamepad1.left_bumper, gamepad1.left_trigger > 0.2);
        outtakeArmPosition = controlSingleServo(outtakeArmPosition, gamepad1.triangle, gamepad1.cross);
        outtakeBoxPosition = controlSingleServo(outtakeBoxPosition, gamepad1.right_bumper, gamepad1.right_trigger > 0.2);
        wristPosition = controlSingleServo(wristPosition, gamepad1.left_stick_x > 0.2, gamepad1.left_stick_x < -0.2);
        clawPosition = controlSingleServo(clawPosition, gamepad1.right_stick_y > 0.2, gamepad1.right_stick_y < -0.2);
    }

    // Moves a single servo in small increments based on input
    private double controlSingleServo(double position, boolean increase, boolean decrease) {
        if (increase) {
            if (timer.seconds() > MIN_WAIT_TIME) {
                position = Math.min(position + SERVO_INCREMENT, MAX_POSITION);
                timer.reset();
            }
        } else if (decrease) {
            if (timer.seconds() > MIN_WAIT_TIME) {
                position = Math.max(position - SERVO_INCREMENT, MIN_POSITION);
                timer.reset();
            }
        }
        return position;
    }

    // Updates all servo positions with target values
    private void updateAllServos() {
        updateServoPosition(base, basePosition);
        updateServoPosition(joint, jointPosition);
        updateServoPosition(wrist, wristPosition);
        updateServoPosition(claw, clawPosition);
        updateServoPosition(outtakeArm, outtakeArmPosition);
        updateServoPosition(outtakeBox, outtakeBoxPosition);
    }

    // Moves a servo to a target position if it's not already close
    private void updateServoPosition(ServoImplEx servo, double targetPosition) {
        if (Math.abs(servo.getPosition() - targetPosition) > 0.005) {
            servo.setPosition(targetPosition);
        }
    }

    // Checks if the analog feedback position is within tolerance of the target position
    private boolean isServoAtTargetPosition(AnalogInput analogInput, double targetPosition) {
        double voltage = analogInput.getVoltage();

        double MIN_VOLTAGE = 0;
        double MAX_VOLTAGE = 0;

        if (analogInput == baseAnalog) {
            MIN_VOLTAGE = BASE_MIN_VOLTAGE;
            MAX_VOLTAGE = BASE_MAX_VOLTAGE;
        } else if (analogInput == jointAnalog) {
            MIN_VOLTAGE = JOINT_MIN_VOLTAGE;
            MAX_VOLTAGE = JOINT_MAX_VOLTAGE;
        } else if (analogInput == wristAnalog) {
            MIN_VOLTAGE = WRIST_MIN_VOLTAGE;
            MAX_VOLTAGE = WRIST_MAX_VOLTAGE;
        }

        double actualPosition = mapVoltageToPosition(voltage, MIN_VOLTAGE, MAX_VOLTAGE);
        return Math.abs(actualPosition - targetPosition) <= POSITION_TOLERANCE;
    }

    // Waits until the servo is at the target position based on the analog feedback
    private void waitForServoToReachPosition(AnalogInput analogInput, double targetPosition) {
        while (!isServoAtTargetPosition(analogInput, targetPosition) && opModeIsActive()) {
            telemetry.addData("Waiting for Position", targetPosition);
            telemetry.update();
        }
    }

    // Executes automated pickup motion sequence
    private void runPickupSequence() {
        runningAutoAction = true;
        manualOverride = false;
        moveServoSmoothlyAndSet(wrist, 0.68);
        waitForServoToReachPosition(wristAnalog, 0.68);
        moveServoSmoothlyAndSet(joint, 0.25);
        waitForServoToReachPosition(jointAnalog, 0.25);
        moveServoSmoothlyAndSet(base, 0.18);
        waitForServoToReachPosition(baseAnalog, 0.18);
        runningAutoAction = false;
        manualOverride = true;
    }

    private void runScanSequence() {
        runningAutoAction = true;
        manualOverride = false;
        moveServoSmoothlyAndSet(wrist, 0.68);
        waitForServoToReachPosition(wristAnalog, 0.68);
        moveServoSmoothlyAndSet(joint, 0.20);
        waitForServoToReachPosition(jointAnalog, 0.20);
        moveServoSmoothlyAndSet(base, 0.30);
        waitForServoToReachPosition(baseAnalog, 0.30);
        runningAutoAction = false;
        manualOverride = true;
    }

    private void runMiddleSequence() {
        runningAutoAction = true;
        manualOverride = false;
        moveServoSmoothlyAndSet(wrist, 0.68);
        waitForServoToReachPosition(wristAnalog, 0.68);
        moveServoSmoothlyAndSet(joint, 0.50);
        waitForServoToReachPosition(jointAnalog, 0.50);
        moveServoSmoothlyAndSet(base, 1.00);
        waitForServoToReachPosition(baseAnalog, 1.00);
        runningAutoAction = false;
        manualOverride = true;
    }

    // Executes automated deliver motion sequence
    private void runDeliverSequence() {
        runningAutoAction = true;
        manualOverride = false;
        moveServoSmoothlyAndSet(base, 0.93);
        waitForServoToReachPosition(baseAnalog, 0.93);
        moveServoSmoothlyAndSet(joint, 1.0);
        waitForServoToReachPosition(jointAnalog, 1.0);
        runningAutoAction = false;
        manualOverride = true;
    }

    // Gradually moves servo to a new position for smooth movement
    private void moveServoSmoothly(ServoImplEx servo, double targetPosition) {
        double currentPosition = servo.getPosition();
        targetPosition = Math.max(MIN_POSITION, Math.min(MAX_POSITION, targetPosition));

        while (opModeIsActive() && Math.abs(currentPosition - targetPosition) > EPSILON) {
            double increment = (Math.abs(targetPosition - currentPosition) > 0.2)
                    ? SMOOTH_INCREMENT : SMOOTH_INCREMENT / 5.0;

            currentPosition = currentPosition < targetPosition
                    ? Math.min(currentPosition + increment, targetPosition)
                    : Math.max(currentPosition - increment, targetPosition);

            servo.setPosition(currentPosition);
            sleep(SERVO_STEP_DELAY);
        }
    }

    // Smoothly moves servo and updates the corresponding position variable
    private void moveServoSmoothlyAndSet(ServoImplEx servo, double targetPosition) {
        moveServoSmoothly(servo, targetPosition);

        if (servo == base) basePosition = targetPosition;
        else if (servo == joint) jointPosition = targetPosition;
        else if (servo == wrist) wristPosition = targetPosition;
        else if (servo == claw) clawPosition = targetPosition;
        else if (servo == outtakeArm) outtakeArmPosition = targetPosition;
        else if (servo == outtakeBox) outtakeBoxPosition = targetPosition;
    }

    // Displays sensor readings and commanded positions on telemetry
    private void updateTelemetry() {
        sendAnalogTelemetry("Base", baseAnalog, base);
        sendAnalogTelemetry("Joint", jointAnalog, joint);
        sendAnalogTelemetry("Wrist", wristAnalog, wrist);

        telemetry.addLine();
        telemetry.addData("Free-Move Mode", freeMoveMode ? "ENABLED" : "DISABLED");
        telemetry.addData("Auto Action Running", runningAutoAction ? "YES" : "NO");
    }

    private void sendAnalogTelemetry(String label, AnalogInput analogInput, ServoImplEx servo) {
        double MIN_VOLTAGE = 0;
        double MAX_VOLTAGE = 0;

        if (analogInput == baseAnalog) {
            MIN_VOLTAGE = BASE_MIN_VOLTAGE;
            MAX_VOLTAGE = BASE_MAX_VOLTAGE;
        } else if (analogInput == jointAnalog) {
            MIN_VOLTAGE = JOINT_MIN_VOLTAGE;
            MAX_VOLTAGE = JOINT_MAX_VOLTAGE;
        } else if (analogInput == wristAnalog) {
            MIN_VOLTAGE = WRIST_MIN_VOLTAGE;
            MAX_VOLTAGE = WRIST_MAX_VOLTAGE;
        }

        telemetry.addData(label + " Voltage", analogInput.getVoltage());
        telemetry.addData(label + " Position", mapVoltageToPosition(analogInput.getVoltage(), MIN_VOLTAGE, MAX_VOLTAGE));
        telemetry.addData(label + " Commanded", servo.getPosition());

    }

    // Maps voltage to a normalized position for each servo
    private double mapVoltageToPosition(double voltage, double minVoltage, double maxVoltage) {
        return Math.max(0.0, Math.min(1.0, 1 - ((voltage - minVoltage) / (maxVoltage - minVoltage))));
    }
}
