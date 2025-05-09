/**
 * LimitSwitchUtilities
 * .
 * Overview:
 * This class provides utility methods for managing digital limit switches in an FTC robot.
 * It handles checking the state of left and right limit switches and can be used to implement
 * docking logic, including resetting a motor's encoder when both switches are pressed and the motor
 * velocity is below a specified threshold.
 * .
 * Features:
 * - Initializes and configures digital limit switches.
 * - Checks the state of individual limit switches.
 * - Implements docking logic based on the state of both limit switches and motor velocity.
 * - Resets a motor encoder when a docking condition is met.
 * .
 * Usage:
 * Used during Autonomous and TeleOp to monitor the state of limit switches and manage
 * motor encoder resets based on docking conditions.
 * .
 * --------------------------------------------------------------------------------
 * Created On:          May 6, 2025
 * Last Updated:        May 9, 2025
 * Original Author:     Daniel Carrillo
 * Contributors:        [List of other contributors]
 * Documentation:       Generated with assistance from Gemini.
 * Organization:        Venus Robot - 10265 Force Green 2024-2025
 * --------------------------------------------------------------------------------
 */

package Venus.boilerplate.helpers;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/**
 * LimitSwitchUtilities
 * (Updated to handle single and dual motor logic)
 */
public class LimitSwitchUtilities {

    private final DigitalChannel leftLimitSwitch;
    private final DigitalChannel rightLimitSwitch;
    private boolean wasPreviouslyDocked = false;

    /** Constructs a new LimitSwitchUtilities object. */
    public LimitSwitchUtilities(HardwareMap hardwareMap, String leftName, String rightName) {
        leftLimitSwitch = hardwareMap.get(DigitalChannel.class, leftName);
        rightLimitSwitch = hardwareMap.get(DigitalChannel.class, rightName);
        configureLimitSwitches();
    }

    /** Configures the limit switches as digital input. */
    private void configureLimitSwitches() {
        leftLimitSwitch.setMode(DigitalChannel.Mode.INPUT);
        rightLimitSwitch.setMode(DigitalChannel.Mode.INPUT);
    }

    /** Single motor overload — delegates to the dual-motor method. */
    public boolean checkAndResetIfDocked(DcMotorEx motor1, double velocityThreshold) {
        return checkAndResetIfDocked(motor1, null, velocityThreshold);
    }

    /** Dual motor support — only resets second motor if it's not null. */
    public boolean checkAndResetIfDocked(DcMotorEx motor1, DcMotorEx motor2, double velocityThreshold) {
        boolean isMotor1Docked = Math.abs(motor1.getVelocity()) < velocityThreshold;
        boolean isMotor2Docked = (motor2 == null) || Math.abs(motor2.getVelocity()) < velocityThreshold;

        boolean currentlyDocked = areBothSwitchesPressed() && isMotor1Docked && isMotor2Docked;

        if (currentlyDocked && !wasPreviouslyDocked) {
            resetMotorEncoder(motor1);
            if (motor2 != null) resetMotorEncoder(motor2);
        }

        wasPreviouslyDocked = currentlyDocked;
        return currentlyDocked;
    }

    private void resetMotorEncoder(DcMotorEx motor) {
        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    public boolean isLeftSwitchPressed() {
        return !leftLimitSwitch.getState(); // false = PRESSED (active low)
    }

    public boolean isRightSwitchPressed() {
        return !rightLimitSwitch.getState(); // false = PRESSED (active low)
    }

    public boolean areBothSwitchesPressed() {
        return isLeftSwitchPressed() && isRightSwitchPressed();
    }
}
