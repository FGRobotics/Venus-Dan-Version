/**
 * SlideUtilities
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

package Venus.boilerplate.helpers;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/** Utility class for managing dual or single motor slide systems with safe, optimized motion. */
public class SlideUtilities {

    private final DcMotorEx slideMotor1, slideMotor2;
    private final LimitSwitchUtilities limitSwitchUtils;

    private final double maximumVelocity;
    private final double minimumVelocity;
    private final int minExtension;
    private final int maxExtension;
    private final int positionTolerance;

    private static final int ACCELERATION_DISTANCE_THRESHOLD = 600;
    private static final int DECELERATION_DISTANCE_THRESHOLD = 200;

    private static final int HOME_BACKOFF_TICKS = -50;
    private static final double DOCK_VELOCITY_THRESHOLD = 50;

    /**
     * Constructor for a single motor slide system.
     * @param hardwareMap         FTC hardware map
     * @param motorName           Name of the slide motor
     * @param direction           Desired motor direction
     * @param maxVel              Maximum target velocity
     * @param minVel              Minimum movement velocity
     * @param tolerance           Position tolerance for completion
     * @param limitSwitchUtils    Limit switch utility reference
     */
    public SlideUtilities(HardwareMap hardwareMap,
                          String motorName,
                          DcMotor.Direction direction,
                          double maxVel,
                          double minVel,
                          int tolerance,
                          int minExtension,
                          int maxExtension,
                          LimitSwitchUtilities limitSwitchUtils) {
        this(hardwareMap, motorName, null, direction, null, maxVel, minVel, tolerance, minExtension, maxExtension, limitSwitchUtils);
    }
    /**
     * Constructor for a dual motor slide system.
     * @param hardwareMap         FTC hardware map
     * @param motorName1          Name of the first slide motor
     * @param motorName2          Name of the second slide motor (can be null for single)
     * @param direction1          Motor 1 direction
     * @param direction2          Motor 2 direction (null if unused)
     * @param maxVel              Maximum allowed velocity
     * @param minVel              Minimum velocity threshold
     * @param tolerance           Encoder position tolerance
     * @param limitSwitchUtils    Limit switch utility instance
     */
    public SlideUtilities(HardwareMap hardwareMap,
                          String motorName1,
                          String motorName2,
                          DcMotor.Direction direction1,
                          DcMotor.Direction direction2,
                          double maxVel,
                          double minVel,
                          int tolerance,
                          int minExtension,
                          int maxExtension,
                          LimitSwitchUtilities limitSwitchUtils) {
        this.limitSwitchUtils = limitSwitchUtils;

        slideMotor1 = hardwareMap.get(DcMotorEx.class, motorName1);
        slideMotor1.setDirection(direction1);
        slideMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (motorName2 != null) {
            slideMotor2 = hardwareMap.get(DcMotorEx.class, motorName2);
            slideMotor2.setDirection(direction2);
            slideMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slideMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } else {
            slideMotor2 = null;
        }
        this.maximumVelocity = maxVel;
        this.minimumVelocity = minVel;
        this.positionTolerance = tolerance;
        this.minExtension = minExtension;
        this.maxExtension = maxExtension;
    }
    /**
     * Moves the slide toward a target encoder position using velocity scaling.
     * @param targetPosition  Desired encoder position
     * @param opModeIsActive  Boolean flag from OpMode to maintain control loop
     */
    public void moveToPosition(int targetPosition, boolean opModeIsActive) {
        targetPosition = Math.max(minExtension, Math.min(maxExtension, targetPosition));

        while (opModeIsActive &&
                Math.abs(slideMotor1.getCurrentPosition() - targetPosition) > positionTolerance ||
                (slideMotor2 != null && Math.abs(slideMotor2.getCurrentPosition() - targetPosition) > positionTolerance)) {

            int remaining1 = targetPosition - slideMotor1.getCurrentPosition();
            double velocity1 = computeVelocity(remaining1);

            if (slideMotor2 != null) {
                int remaining2 = targetPosition - slideMotor2.getCurrentPosition();
                double velocity2 = computeVelocity(remaining2);
                slideMotor2.setVelocity(Math.signum(remaining2) * velocity2);
            }

            slideMotor1.setVelocity(Math.signum(remaining1) * velocity1);

            checkAndResetIfDocked();  // Auto reset if docked
        }
        setMotorVelocity(0);
    }
    /**
     * Returns scaled velocity based on distance remaining.
     * @param remainingDistance  Encoder ticks to target
     * @return Velocity value in ticks/sec
     */
    private double computeVelocity(int remainingDistance) {
        int absRemaining = Math.abs(remainingDistance);
        if (absRemaining > ACCELERATION_DISTANCE_THRESHOLD) {
            return calculateAccelerationVelocity(remainingDistance);
        } else if (absRemaining > DECELERATION_DISTANCE_THRESHOLD) {
            return maximumVelocity;
        } else {
            return calculateDecelerationVelocity(remainingDistance);
        }
    }

    /** Calculates smooth ramp-up velocity. */
    private double calculateAccelerationVelocity(int remainingDistance) {
        return Math.min(maximumVelocity,
                Math.abs((double) remainingDistance / ACCELERATION_DISTANCE_THRESHOLD * maximumVelocity));
    }

    /** Calculates smooth slowdown velocity. */
    private double calculateDecelerationVelocity(int remainingDistance) {
        return Math.max(minimumVelocity,
                Math.abs((double) remainingDistance / DECELERATION_DISTANCE_THRESHOLD * maximumVelocity));
    }

    /** Applies velocity to both slide motors. */
    private void setMotorVelocity(double velocity) {
        slideMotor1.setVelocity(velocity);
        if (slideMotor2 != null) {
            slideMotor2.setVelocity(velocity);
        }
    }

    /**
     * Checks limit switch docking state and resets encoders if docked.
     * @return True if docked and encoders reset
     */
    public boolean checkAndResetIfDocked() {
        if (slideMotor2 != null) {
            return limitSwitchUtils.checkAndResetIfDocked(slideMotor1, slideMotor2, DOCK_VELOCITY_THRESHOLD);
        } else {
            return limitSwitchUtils.checkAndResetIfDocked(slideMotor1, DOCK_VELOCITY_THRESHOLD);
        }
    }
    /**
     * Moves the slide to home position using limit switches and backs off slightly.
     * @param opModeIsActive  OpMode status flag
     */
    public void homeSlide(boolean opModeIsActive) {
        setMotorVelocity(-minimumVelocity);

        long startTime = System.currentTimeMillis();
        final long timeoutMillis = 4000;  // 4 seconds timeout

        while (opModeIsActive && !limitSwitchUtils.areBothSwitchesPressed()) {
            if (System.currentTimeMillis() - startTime > timeoutMillis) {
                setMotorVelocity(0);
                throw new RuntimeException("Homing failed: Limit switches not triggered within timeout.");
            }
        }
        setMotorVelocity(0);

        slideMotor1.setTargetPosition(HOME_BACKOFF_TICKS);
        slideMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor1.setPower(0.3);

        if (slideMotor2 != null) {
            slideMotor2.setTargetPosition(HOME_BACKOFF_TICKS);
            slideMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slideMotor2.setPower(0.3);
        }

        while (opModeIsActive &&
                slideMotor1.isBusy() &&
                (slideMotor2 == null || slideMotor2.isBusy())) {
            Thread.yield(); // Avoids tight CPU loop
        }
        slideMotor1.setPower(0);
        if (slideMotor2 != null) slideMotor2.setPower(0);

        slideMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (slideMotor2 != null) {
            slideMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slideMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    /** Convenience method to move slide to 0 ticks. */
    public void goToMinExtension(boolean opModeIsActive) {
        moveToPosition(minExtension, opModeIsActive);
    }
    /** Moves slide to maximum allowed extension. */
    public void goToMaxExtension(boolean opModeIsActive) {
        moveToPosition(maxExtension, opModeIsActive);
    }
    public DcMotorEx getMotor1() {
        return slideMotor1;
    }
    public DcMotorEx getMotor2() {
        return slideMotor2;
    }
}
