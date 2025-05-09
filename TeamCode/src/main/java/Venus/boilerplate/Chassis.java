/**
 * Chassis.java
 * .
 * Overview:
 * This class manages the drivetrain subsystem for a mecanum-wheeled FTC robot.
 * It handles TeleOp field-centric driving, precision mode, and integrated deadwheel
 * odometry support. It also supports Limelight-assisted control and IMU-based orientation tracking.
 * .
 * Features:
 * - Field-centric and robot-centric drive modes
 * - Toggleable precision control for sensitive movement
 * - Integrated odometry via deadwheel encoders
 * - IMU-based heading correction
 * - Limelight-compatible movement interface
 * - Modular setup for expandability and multi-mode reuse
 * .
 * Usage:
 * Used during Both TeleOp and Autonomous modes to manage all drive and movement logic for the robot.
 * The class ensures consistent field-aligned driving and precise control, abstracting hardware setup
 * and movement calculations into a safe and reusable module.
 * .
 * --------------------------------------------------------------------------------------
 * Created On:          May 08, 2025
 * Last Updated:        May 09, 2025
 * Original Author:     YourNameHere
 * Contributors:        -
 * Documentation:       Generated with assistance from OpenAI's ChatGPT
 * Organization:        Venus Robot - 10265 Force Green 2024-2025
 * --------------------------------------------------------------------------------------
 */

package Venus.boilerplate;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.boilerplate.sensorclasses.SparkIMU;

public class Chassis {

    public static SparkIMU imu;
    private final DcMotorEx frontLeft, frontRight, backLeft, backRight;

    /**
     * Constructor initializes all hardware components used for driving and localization.
     * @param hardwareMap HardwareMap from OpMode
     */
    public Chassis(HardwareMap hardwareMap) {
        frontLeft = hardwareMap.get(DcMotorEx.class, "FL");
        backLeft = hardwareMap.get(DcMotorEx.class, "BL");
        backRight = hardwareMap.get(DcMotorEx.class, "BR");
        frontRight = hardwareMap.get(DcMotorEx.class, "FR");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

            imu = new SparkIMU(hardwareMap);
    }


    /**
     * Handles TeleOp drive logic using field-centric inputs and optional strafing overrides.
     * @param x               joystick X input
     * @param y               joystick Y input
     * @param rotation        rotational input
     * @param rightTrigger    right trigger pressure (for speed control)
     * @param rotationalMultiplier  scalar for rotation power; sets speed for rotation (0.00-1.00)
     * @param lateralMultiplier     scalar for strafing power; sets speed for normal movement (0.00-1.00)
     * @param isReversed      true if controls should be reversed
     * @param strafeLeft      true to force left strafe
     * @param strafeRight     true to force right strafe
     */
    public void teleDrive(double x, double y, double rotation, double rightTrigger,
                          double rotationalMultiplier, double lateralMultiplier, boolean isReversed,
                          boolean strafeLeft, boolean strafeRight) {

        double reverse = isReversed ? -1 : 1;
        double triggerSpeed = rightTrigger > 0 ? 0.4 : 2.0;
        double rotPower = rotation * rotationalMultiplier * reverse;
        boolean useFieldCentric = true;

        if (strafeLeft) {
            x = -0.5 * reverse;
            y = 0;
            rotPower = 0;
            useFieldCentric = false;
        } else if (strafeRight) {
            x = 0.5 * reverse;
            y = 0;
            rotPower = 0;
            useFieldCentric = false;
        } else {
            x = -reverse * x * triggerSpeed * lateralMultiplier;
            y = -reverse * y * triggerSpeed;
        }

        double magnitude = Math.hypot(x, y);
        double direction = Math.atan2(y, x);
        direction = normalizeAngle(Math.toRadians(90) - direction);

        drive(magnitude, direction, rotPower, useFieldCentric);
    }

    /**
     * Drives the robot based on a polar vector, optionally using field-centric rotation.
     * @param magnitude     drive power magnitude
     * @param direction     vector angle in radians
     * @param rotational    turn power
     * @param fieldCentric  true if heading should be adjusted to field orientation
     */
    public void drive(double magnitude, double direction, double rotational, boolean fieldCentric) {
        double heading = fieldCentric ? imu.otos.getPosition().h : 0.0;
        double adjustedDirection = fieldCentric ? normalizeAngle(direction - heading) : direction;

        double powerA = Math.sin(adjustedDirection + (Math.PI / 4)) * magnitude;
        double powerB = -Math.sin(adjustedDirection - (Math.PI / 4)) * magnitude;

        frontRight.setPower(powerA + rotational);
        backLeft.setPower(powerA - rotational);
        backRight.setPower(powerB + rotational);
        frontLeft.setPower(powerB - rotational);
    }

    /**
     * Utility to normalize an angle to [0, 2π) radians.
     * @param radians input angle
     * @return normalized angle
     */
    private double normalizeAngle(double radians) {
        while (radians < 0) radians += 2 * Math.PI;
        while (radians >= 2 * Math.PI) radians -= 2 * Math.PI;
        return radians;
    }
}
