package frc.robot.trajectory;

import static frc.robot.trajectory.TrajectoryConfig.MODEL_COEFFICIENT_A;
import static frc.robot.trajectory.TrajectoryConfig.MODEL_COEFFICIENT_L;
import static frc.robot.trajectory.TrajectoryConfig.MODEL_COEFFICIENT_R;

public class LaunchSpeed {
    /** Estimate the launch speed given a flywheel speed and field launch pitch. */
    public static double estimateSpeed(double flywheelSpeed, double launcherPitch) {
        return MODEL_COEFFICIENT_L
                / (1.0 + MODEL_COEFFICIENT_A * Math.exp(-MODEL_COEFFICIENT_R * flywheelSpeed));
    }

    /** Estimate the flywheel speed given a launch speed and field launch pitch. */
    public static double estimateFlywheelSpeed(double launchSpeed, double launcherPitch) {
        return Math.log(MODEL_COEFFICIENT_A * launchSpeed / (MODEL_COEFFICIENT_L - launchSpeed))
                / MODEL_COEFFICIENT_R;
    }
}
