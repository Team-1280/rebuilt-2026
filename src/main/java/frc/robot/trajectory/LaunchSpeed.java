package frc.robot.trajectory;

public class LaunchSpeed {
    /** Estimate the launch speed given a flywheel speed and field launch pitch. */
    public static double estimateSpeed(double flywheelSpeed, double launcherPitch) {
        // TODO: estimateSpeed
        return flywheelSpeed / 10.0;
        // throw new UnsupportedOperationException("not implemented");
    }

    /** Estimate the flywheel speed given a launch speed and field launch pitch. */
    public static double estimateFlywheelSpeed(double launchSpeed, double launcherPitch) {
        // TODO: estimateFlywheelSpeed
        return launchSpeed * 10.0;
        // throw new UnsupportedOperationException("not implemented");
    }
}
