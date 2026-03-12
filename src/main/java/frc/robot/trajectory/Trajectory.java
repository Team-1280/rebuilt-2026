package frc.robot.trajectory;

import edu.wpi.first.math.geometry.Rotation3d;

/**
 * A trajectory describing all variables and calculated unknowns for use in launcher control. These
 * are given as a result of trajectory calculations.
 *
 * <p>A trajectory may be invalid, and this must be checked. Invalid trajectories still always have
 * logical values for all fields, but these values may sometimes be mechanically unachievable.
 */
public class Trajectory {
    private final TrajectoryParameters parameters;
    private final double pitch;
    private final double yaw;
    private final double speed;
    private final double time;
    private final Rotation3d launcherRotation;

    private boolean valid;

    /**
     * Create a trajectory with the given parameters and calculated values.
     *
     * @param parameters the parameters used to calculate this trajectory, for reference
     * @param pitch the pitch for this trajectory, relative to the field
     * @param yaw the yaw for this trajectory, relative to the field
     * @param speed the launch speed for this trajectory
     * @param time the time of flight or duration of the trajectory from launcher to target
     * @param valid whether this trajectory is currently known as valid or invalid
     */
    public Trajectory(
            TrajectoryParameters parameters,
            double pitch,
            double yaw,
            double speed,
            double time,
            boolean valid) {
        this.parameters = parameters;
        this.pitch = pitch;
        this.yaw = yaw;
        this.speed = speed;
        this.time = time;
        this.valid = valid;
        launcherRotation =
                new Rotation3d(0.0, -getPitch(), getYaw())
                        .relativeTo(parameters.getLauncherRotation());
    }

    /** Get the flywheel speed needed to achieve the launch speed, including error compensation. */
    public double getFlywheelSpeed() {
        return LaunchSpeed.estimateFlywheelSpeed(getCompensatedSpeed(), pitch);
    }

    /** Get the parameters used to calculate this trajectory. */
    public TrajectoryParameters getParameters() {
        return parameters;
    }

    /** Get the pitch for this trajectory, relative to the field. */
    public double getPitch() {
        return pitch;
    }

    /** Get the yaw for this trajectory, relative to the field. */
    public double getYaw() {
        return yaw;
    }

    /** Get the launch speed for this trajectory. */
    public double getSpeed() {
        return speed;
    }

    /** Get the launch speed multiplied by a compensation multiplier to counteract error. */
    public double getCompensatedSpeed() {
        return speed * parameters.getSpeedMultiplier();
    }

    /** Get the time of flight or duration of the trajectory from launcher to target. */
    public double getTime() {
        return time;
    }

    /**
     * Get whether the trajectory is currently known as valid. By the end of the trajectory solving,
     * this should be complete and reliable.
     */
    public boolean isValid() {
        return valid;
    }

    /** Invalidate this trajectory, due to new information (constraints) being discovered. */
    public void invalidate() {
        valid = false;
    }

    /** Get the launcher pitch for this trajectory, relative to the launcher's base rotation. */
    public double getLauncherPitch() {
        return -launcherRotation.getY();
    }

    /** Get the launcher yaw for this trajectory, relative to the launcher's base rotation. */
    public double getLauncherYaw() {
        return launcherRotation.getZ();
    }

    /**
     * Get the launcher 3D rotation for this trajectory, relative to the launcher's base rotation.
     */
    public Rotation3d getLauncherRotation() {
        return launcherRotation;
    }

    @Override
    public String toString() {
        return String.format(
                "Trajectory[pitch=%.4f, yaw=%.4f, speed=%.4f, time=%.4f, valid=%b,"
                        + " launcherPitch=%.4f, launcherYaw=%.4f, parameters=%s]",
                pitch, yaw, speed, time, valid, getLauncherPitch(), getLauncherYaw(), parameters);
    }
}
