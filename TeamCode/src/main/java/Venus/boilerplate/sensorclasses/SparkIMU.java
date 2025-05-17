/**
 * SparkIMU.java
 * .
 * Overview:
 * This class provides a structured and modular interface to the SparkFunOTOS IMU sensor used
 * for pose estimation and angular tracking in FTC robots. It supports unit configuration,
 * calibration, and tracking resets with applied pose offsets and scaling constants.
 * .
 * Features:
 * - IMU initialization with unit settings and hardware-specific offsets
 * - Tracking reset and recalibration methods
 * - Pose2D offsetting for mounting compensation
 * - Angular and linear scalar adjustments for drift correction
 * .
 * Usage:
 * Used during Both TeleOp and Autonomous modes to track robot orientation and pose using
 * the SparkFunOTOS IMU. Provides a reusable wrapper to standardize pose sensor management.
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

package Venus.boilerplate.sensorclasses;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/** SparkIMU class wraps configuration and management of the SparkFunOTOS IMU sensor. */
public class SparkIMU {

    /** The underlying SparkFunOTOS hardware object used for pose and heading tracking. */
    public SparkFunOTOS otos;

    /**
     * Constructs the SparkIMU object, initializing hardware and configuring calibration.
     * @param hardwareMap the HardwareMap provided by the active OpMode
     */
    public SparkIMU(HardwareMap hardwareMap) {
        otos = hardwareMap.get(SparkFunOTOS.class, "SparkFunIMU");
        initializeIMU();
        resetIMU();
    }

    /**
     * Initializes the IMU with standardized angle units, positional offsets,
     * scalar multipliers, and calibration procedures.
     */
    public void initializeIMU() {
        otos.setAngularUnit(AngleUnit.RADIANS);

        // Apply positional offset to account for sensor's physical location on robot
        SparkFunOTOS.Pose2D imuOffset = new SparkFunOTOS.Pose2D(1.375, -3.625, 0);
        otos.setOffset(imuOffset);

        // Apply scale factors to correct tracking and rotation drift
        otos.setLinearScalar(1.0);
        otos.setAngularScalar(1.002823409);

        // Calibrate sensor for accurate angular readings
        otos.calibrateImu();

        // Reset internal pose tracking to start at origin
        otos.resetTracking();
    }

    /**
     * Resets the internal pose tracking algorithm, returning position and heading to (0, 0, 0).
     * Useful for autonomous resets or during field initialization.
     */
    public void resetIMU() {
        otos.resetTracking();
    }
}
