/**
 * Chassis.java
 * .
 * Overview:
 * This class manages the drivetrain subsystem for a mecanum-wheeled FTC robot.
 * It supports field-centric TeleOp control, IMU-based heading correction, and manual strafe overrides.
 * Designed for reuse in both TeleOp and Autonomous modes, it cleanly abstracts motor logic, movement computation,
 * and orientation handling using SparkIMU and angle normalization utilities.
 * .
 * Features:
 * - Field-centric and robot-centric drive modes
 * - Toggleable heading lock with rotation control
 * - Smooth strafe override logic for precise alignment
 * - IMU-based angular feedback for orientation tracking
 * - Extendable modular hardware structure with clean motor abstraction
 * .
 * Usage:
 * Used during both TeleOp and Autonomous operation to manage all movement and orientation logic.
 * Integrates seamlessly with external IMU (SparkIMU) and provides utility methods for field-aligned driving,
 * rotation control, and maneuverability during driver control and trajectory execution.
 * .
 * --------------------------------------------------------------------------------------
 * Created On:          May 08, 2025
 * Last Updated:        May 15, 2025
 * Original Author:     Daniel Carrillo
 * Contributors:        [Add others if applicable]
 * Documentation:       Generated with assistance from OpenAI's ChatGPT
 * Organization:        Venus Robot - 10265 Force Green 2024-2025
 * --------------------------------------------------------------------------------
 */

package Venus.boilerplate;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import Venus.boilerplate.sensorclasses.SparkIMU;

public class Chassis {

    public static SparkIMU imu;

    private final DcMotorEx frontLeftMotor;
    private final DcMotorEx frontRightMotor;
    private final DcMotorEx backLeftMotor;
    private final DcMotorEx backRightMotor;

    private boolean headingLockEnabled = false;
    private boolean headingLockButtonPreviouslyPressed = false;

    private double lockedHeading = 0.0;

    /**
     * Constructs a Chassis object and initializes motor directions and IMU.
     * @param hardwareMap the hardware map provided by the OpMode
     */
    public Chassis(HardwareMap hardwareMap) {
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "FL");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "FR");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "BL");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "BR");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = new SparkIMU(hardwareMap);
    }

    /**
     * Performs TeleOp driving with optional field-centric and manual strafe control.
     * @param xInput                   lateral joystick input
     * @param yInput                   forward/backward joystick input
     * @param rotationInput            rotation joystick input
     * @param strafeSpeedScalar        speed for manual strafing
     * @param movementSpeedScalar      scalar for normal movement
     * @param rotationSpeedScalar      scalar for rotation speed
     * @param reverseControlsEnabled   true if driving direction is reversed
     * @param forceStrafeLeft          true to override input and strafe left
     * @param forceStrafeRight         true to override input and strafe right
     * @param toggleHeadingLock        true to toggle heading lock
     */
    public void driveTeleOp(double xInput, double yInput, double rotationInput, double movementSpeedScalar, double rotationSpeedScalar, double strafeSpeedScalar, boolean reverseControlsEnabled, boolean forceStrafeLeft, boolean forceStrafeRight, boolean toggleHeadingLock) {

        double directionFactor = reverseControlsEnabled ? -1.0 : 1.0;
        boolean useFieldCentric = true;
        double rotationalPower;

        if (Math.abs(xInput) < 0.05) xInput = 0;
        if (Math.abs(yInput) < 0.05) yInput = 0;


        if (forceStrafeLeft || forceStrafeRight) {
            xInput = directionFactor * strafeSpeedScalar * (forceStrafeRight ? 1.0 : -1.0);
            yInput = 0.0;
            rotationalPower = 0.0;
            useFieldCentric = false;
        } else {
            xInput *= -directionFactor * movementSpeedScalar;
            yInput *= -directionFactor * movementSpeedScalar;

            double currentHeading = imu.otos.getPosition().h;

            if (toggleHeadingLock && !headingLockButtonPreviouslyPressed) {
                if (headingLockEnabled) {
                    headingLockEnabled = false;
                } else {
                    lockedHeading = normalizeAngle(currentHeading);
                    headingLockEnabled = true;
                }
            }
            headingLockButtonPreviouslyPressed = toggleHeadingLock;

            if (headingLockEnabled) {
                double headingError = normalizeDeltaAngle(lockedHeading - currentHeading);
                double headingCorrectionCoefficient = 2.0; // Tunable gain
                rotationalPower = headingError * headingCorrectionCoefficient;
                rotationalPower = Math.max(-0.5, Math.min(0.5, rotationalPower));
            } else {
                rotationalPower = rotationInput * rotationSpeedScalar * directionFactor;
            }
        }

        double inputMagnitude = Math.hypot(xInput, yInput);
        double inputDirection = normalizeAngle(Math.toRadians(90.0) - Math.atan2(yInput, xInput));

        drivePolar(inputMagnitude, inputDirection, rotationalPower, useFieldCentric);
    }

    /**
     * Drives robot using polar coordinates, optionally field-centric.
     * @param magnitude     drive power magnitude (0.0 to 1.0)
     * @param direction     angle of motion in radians
     * @param rotational    rotational power (-1.0 to 1.0)
     * @param fieldCentric  true for field-aligned movement
     */
    public void drivePolar(double magnitude, double direction, double rotational, boolean fieldCentric) {
        double robotHeading = fieldCentric ? imu.otos.getPosition().h : 0.0;
        double adjustedDirection = fieldCentric ? normalizeAngle(direction - robotHeading) : direction;

        double powerA = Math.sin(adjustedDirection + Math.PI / 4.0) * magnitude;
        double powerB = -Math.sin(adjustedDirection - Math.PI / 4.0) * magnitude;

        setMotorPowers(powerA + rotational, powerB + rotational, powerB - rotational, powerA - rotational);
    }

    /**
     * Sets power to individual drive motors.
     * @param frontRightPower power for front right motor
     * @param backRightPower  power for back right motor
     * @param frontLeftPower  power for front left motor
     * @param backLeftPower   power for back left motor
     */
    private void setMotorPowers(double frontRightPower, double backRightPower, double frontLeftPower, double backLeftPower) {
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
    }

    public void rotate180Degrees() {
        double currentHeading = imu.otos.getPosition().h;
        lockedHeading = normalizeAngle(currentHeading + Math.PI);
        headingLockEnabled = true;
    }

    /**
     * Normalizes angle to the range [0, 2π).
     * @param radians angle in radians
     * @return normalized angle in [0, 2π)
     */
    private double normalizeAngle(double radians) {
        while (radians < 0.0) radians += 2.0 * Math.PI;
        while (radians >= 2.0 * Math.PI) radians -= 2.0 * Math.PI;
        return radians;
    }

    /**
     * Normalizes angle difference to the range [-π, π].
     * @param angle angle difference in radians
     * @return normalized difference in [-π, π]
     */
    private double normalizeDeltaAngle(double angle) {
        while (angle > Math.PI) angle -= 2.0 * Math.PI;
        while (angle < -Math.PI) angle += 2.0 * Math.PI;
        return angle;
    }
}
