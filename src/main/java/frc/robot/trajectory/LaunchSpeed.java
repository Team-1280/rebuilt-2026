package frc.robot.trajectory;

import static frc.robot.trajectory.TrajectoryConfig.MODEL_C1;
import static frc.robot.trajectory.TrajectoryConfig.MODEL_C2;
import static frc.robot.trajectory.TrajectoryConfig.MODEL_C3;
import static frc.robot.trajectory.TrajectoryConfig.MODEL_C4;
import static frc.robot.trajectory.TrajectoryConfig.MODEL_C5;

/**
 * A calculator for launch speed as a function of flywheel speed and launch pitch.
 *
 * <p>The model is a modified logistic equation with extra coefficients for accounting for pitch.
 *
 * <p>Suppose x = flywheel speed (rad/s), y = launch pitch (rad), z = launch speed (m/s)
 *
 * <p>Define model coefficients C1 to C5 determined via regression with a graphing utility, of a
 * collected dataset of many different flywheel speeds and pitches.
 *
 * <p>z = (C1 + C2*y) / (1 + C3*y + e^-(C4*x + C5))
 *
 * <p>x = (ln((C1 + C2*y)/z - C3*y - 1) + C5) / -C4
 */
public class LaunchSpeed {
    /** Estimate the launch speed given a flywheel speed (rad/s) and field launch pitch (rad). */
    public static double estimateSpeed(double flywheelSpeed, double launchPitch) {
        return (MODEL_C1 + MODEL_C2 * launchPitch)
                / (1 + MODEL_C3 * launchPitch + Math.exp(-(MODEL_C4 * flywheelSpeed + MODEL_C5)));
    }

    /** Estimate the flywheel speed given a launch speed (m/s) and field launch pitch (rad). */
    public static double estimateFlywheelSpeed(double launchSpeed, double launchPitch) {
        return (Math.log(
                                (MODEL_C1 + MODEL_C2 * launchPitch) / launchSpeed
                                        - MODEL_C3 * launchPitch
                                        - 1)
                        + MODEL_C5)
                / -MODEL_C4;
    }
}
