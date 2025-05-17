/**
 * SlideUtilities
 * .
 * Overview:
 * This class provides modular control logic for single or dual-slide mechanisms in 
 * FTC robots using DcMotorEx motors. It includes smooth motion control with acceleration
 * and deceleration ramping, docking/reset mechanisms using limit switches, and utility
 * functions for homing, range travel, and safety monitoring.
 * .
 * Features:
 * - Trapezoidal motion profiling for smooth, controlled velocity
 * - Docking detection with encoder reset using digital limit switches and fail-safe timeout
 * - Configurable motor directions, tolerances, and safety thresholds
 * - Dual motor support with synchronized movement
 * .
 * Usage:
 *  Used during both TeleOp and Autonomous modes for controlling slide actuators via joystick
 *  input or precise motion-to-position calls. Integrates with limit switches for calibration
 *  and safety, and uses velocity-based movement instead of raw power for greater precision.
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

package Venus.boilerplate.helpers;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

/** Utility class for managing dual or single motor slide systems with safe, optimized motion. */
public class SlideUtilities {

    private final DcMotorEx slideMotor1, slideMotor2;
    public final LimitSwitchUtilities limitSwitchUtils;

    private final double minimumVelocity, maximumVelocity;
    private final int minExtension, maxExtension;
    private final int positionTolerance;

    private static final double DOCK_VELOCITY_THRESHOLD = 50;
    private final SlideExtensionMotionProfile motionProfile;
    private final double minimumVelocityRate, maximumVelocityRate;

    public DcMotorEx getMotor1() {return slideMotor1;}
    public DcMotorEx getMotor2() {return slideMotor2;}
    public void goToMinExtension(boolean opModeIsActive) {moveToPosition(minExtension, opModeIsActive);}
    public void goToMaxExtension(boolean opModeIsActive) {moveToPosition(maxExtension-positionTolerance, opModeIsActive);}

    /**
     * Constructor for a single motor slide system.
     * @param hardwareMap      FTC hardware map
     * @param motorName        Name of the slide motor
     * @param direction        Desired motor direction
     * @param minVel           Minimum allowed velocity
     * @param maxVel           Maximum allowed velocity
     * @param tolerance        Encoder position tolerance
     * @param limitSwitchUtils Limit switch utility instance
     */
    public SlideUtilities(HardwareMap hardwareMap, String motorName, DcMotorEx.Direction direction, int tolerance, int minExtension, int maxExtension, double minVel, double minVelRate, double maxVel, double maxVelRate,  LimitSwitchUtilities limitSwitchUtils) {
        this(hardwareMap, motorName, null, direction, tolerance,  minExtension,  maxExtension,  minVel,  minVelRate,  maxVel,  maxVelRate,   limitSwitchUtils);
    }

    /**
     * Constructor for a dual motor slide system.
     * @param hardwareMap      FTC hardware map
     * @param motorName1       Name of the first slide motor
     * @param motorName2       Name of the second slide motor
     * @param direction        Desired motor direction
     * @param minVel           Minimum allowed velocity
     * @param maxVel           Maximum allowed velocity
     * @param tolerance        Encoder position tolerance
     * @param limitSwitchUtils Limit switch utility instance
     */
    public SlideUtilities(HardwareMap hardwareMap, String motorName1, String motorName2, DcMotorEx.Direction direction, int tolerance, int minExtension, int maxExtension, double minVel, double minVelRate, double maxVel, double maxVelRate,  LimitSwitchUtilities limitSwitchUtils) {
        this.limitSwitchUtils = limitSwitchUtils;

        slideMotor1 = hardwareMap.get(DcMotorEx.class, motorName1);
        slideMotor1.setDirection(direction);
        slideMotor1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        if (motorName2 != null) {
            slideMotor2 = hardwareMap.get(DcMotorEx.class, motorName2);
            slideMotor2.setDirection(direction == DcMotorEx.Direction.FORWARD // Reverse motor 2's direction to always mirror motor 1
                    ? DcMotorEx.Direction.REVERSE
                    : DcMotorEx.Direction.FORWARD);

            slideMotor2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            slideMotor2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        } else {
            slideMotor2 = null;
        }
        this.maximumVelocity = maxVel;
        this.minimumVelocityRate = minVelRate;
        this.minimumVelocity = minVel;
        this.maximumVelocityRate = maxVelRate;
        this.positionTolerance = tolerance;
        this.minExtension = minExtension;
        this.maxExtension = maxExtension;
        this.motionProfile = new SlideExtensionMotionProfile(maximumVelocity, minimumVelocity, maximumVelocityRate, minimumVelocityRate, positionTolerance);
    }

    /**
     * Controls the slide manually using a joystick input, with optional overrideTrigger for recalibration.
     * This method respects extension limits unless overrideTrigger is true.
     * @param joystickPower   Power from joystick input (-1 to 1)
     * @param overrideTrigger If true, bypasses encoder limits for manual recalibration
     */
    public void slideControl(double joystickPower, boolean overrideTrigger) {
        int currentPos = slideMotor1.getCurrentPosition();
        double motorPower;

        if (Math.abs(joystickPower) < 0.05) joystickPower = 0;

        if (overrideTrigger) {
            motorPower = joystickPower;
        } else {
            boolean overMax = currentPos > maxExtension && joystickPower > 0;
            boolean belowMin = currentPos < minExtension && joystickPower < 0;
            boolean isDocked = limitSwitchUtils.areBothSwitchesPressed();

            if (overMax || belowMin || (isDocked && joystickPower < 0)) {
                motorPower = 0;
            } else {
                motorPower = joystickPower;
            }
        }
        slideMotor1.setPower(motorPower);
        if (slideMotor2 != null) {
            slideMotor2.setPower(motorPower);
        }
        checkAndResetIfDocked();  // Auto reset if docked
    }

    /**
     * Moves the slide toward a target encoder position using velocity scaling, calling on a trapezoidal motion profile.
     * @param targetPosition Desired encoder position
     * @param opModeIsActive Boolean flag from OpMode to maintain control loop
     */
    public void moveToPosition(int targetPosition, boolean opModeIsActive) {
        targetPosition = Math.max(minExtension, Math.min(maxExtension, targetPosition));
        long startTime = System.currentTimeMillis();
        final long timeoutMillis = 10000;  // 10 seconds timeout

        while (opModeIsActive &&
                (Math.abs(slideMotor1.getCurrentPosition() - targetPosition) > positionTolerance || (slideMotor2 != null && Math.abs(slideMotor2.getCurrentPosition() - targetPosition) > positionTolerance))) {

            int currentPosition = (slideMotor2 != null)
                    ? (slideMotor1.getCurrentPosition() + slideMotor2.getCurrentPosition()) / 2
                    : slideMotor1.getCurrentPosition();

            int remaining = targetPosition - currentPosition;

            double velocity = motionProfile.computeVelocity(remaining, targetPosition);
            velocity = Math.max(minimumVelocity, velocity);  // Clamp to min velocity

            if (Math.abs(remaining) <= positionTolerance) velocity = 0;
            if (Math.abs(velocity) < 50) velocity = 0;


            if (System.currentTimeMillis() - startTime <= timeoutMillis) {
                if (slideMotor2 != null) {
                    slideMotor1.setVelocity(Math.signum(remaining) * velocity);
                    slideMotor2.setVelocity(Math.signum(remaining) * velocity);
                    checkAndResetIfDocked();  // Auto reset if docked
                } else {
                    slideMotor1.setVelocity(Math.signum(remaining) * velocity);
                    checkAndResetIfDocked();  // Auto reset if docked
                }
            } else {
                setMotorVelocity(0);
                break;
                //throw new RuntimeException("Timeout reached while moving slide.");
            }
        }
        setMotorVelocity(0);
    }

    /**
     * Moves the slide to home position using limit switches.
     * @param opModeIsActive OpMode status flag
     */
    public void homeSlide(boolean opModeIsActive) {

        long startTime = System.currentTimeMillis();
        final long timeoutMillis = 10000;  // 10 seconds timeout

        while (opModeIsActive && !limitSwitchUtils.areBothSwitchesPressed()) {
            setMotorVelocity(-minimumVelocity);
            if (System.currentTimeMillis() - startTime > timeoutMillis) {
                setMotorVelocity(0);
                throw new RuntimeException("Homing failed: Limit switches not triggered within timeout.");
            }
        }
        setMotorVelocity(0);

        moveToPosition(minExtension, opModeIsActive);
    }

    /** Checks limit switch docking state as well as the motor velocity to determine if the slide is docked and resetting encoders. */
    public void checkAndResetIfDocked() {
        boolean isMotor1Docked = Math.abs(slideMotor1.getVelocity()) < DOCK_VELOCITY_THRESHOLD;
        boolean isMotor2Docked = (slideMotor2 == null) || Math.abs(slideMotor2.getVelocity()) < DOCK_VELOCITY_THRESHOLD;
        boolean currentlyDocked = limitSwitchUtils.areBothSwitchesPressed() && (isMotor1Docked || isMotor2Docked);

        if (currentlyDocked) {
            slideMotor1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            slideMotor1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            if (slideMotor2 != null) {
                slideMotor2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                slideMotor2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            }
        }
    }

    public void setMotorVelocity(double velocity) {
        slideMotor1.setVelocity(velocity);
        if (slideMotor2 != null) {
            slideMotor2.setVelocity(velocity);
        }
    }
}


