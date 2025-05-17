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
 * Last Updated:        May 15, 2025
 * Original Author:     Daniel Carrillo
 * Contributors:        [Add others if applicable]
 * Documentation:       Generated with assistance from OpenAI's ChatGPT
 * Organization:        Venus Robot - 10265 Force Green 2024-2025
 * --------------------------------------------------------------------------------
 */

package Venus.boilerplate.helpers;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LimitSwitchUtilities {

    private final DigitalChannel leftLimitSwitch;
    private final DigitalChannel rightLimitSwitch;

    public boolean isLeftSwitchPressed() {return leftLimitSwitch.getState();} // true = PRESSED
    public boolean isRightSwitchPressed() {return rightLimitSwitch.getState();} // true = PRESSED
    public boolean areBothSwitchesPressed() {return isLeftSwitchPressed() && isRightSwitchPressed();}

    private void configureLimitSwitches() {
        leftLimitSwitch.setMode(DigitalChannel.Mode.INPUT);
        rightLimitSwitch.setMode(DigitalChannel.Mode.INPUT);
    }

    public LimitSwitchUtilities(HardwareMap hardwareMap, String leftName, String rightName) {
        leftLimitSwitch = hardwareMap.get(DigitalChannel.class, leftName);
        rightLimitSwitch = hardwareMap.get(DigitalChannel.class, rightName);
        configureLimitSwitches();
    }
}
