package frc.robot.trajectory;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * A collection of the given variables that describe the state of the launching system.
 *
 * <p>Note that all numeric values used within trajectory calculations are in the standard SI units
 * of meters, seconds, radians.
 */
public class TrajectoryParameters {
    private final Pose3d launcherPose;
    private final Translation2d launcherVelocity;

    // Cached fields, avoiding repeated calculations or object creations
    private final Translation3d displacement;
    private final Translation2d horizontalDisplacement;
    private final double verticalDisplacement;
    private final double horizontalDistance;
    private final double elevationAngle;

    /**
     * Construct the set of trajectory calculation parameters.
     *
     * @param robotPose the pose of the robot in the field coordinate system
     * @param launcherTransform the transform from the robot to the launcher's launcher (fuel exit)
     * @param targetTranslation the position of the target in the field coordinate system
     * @param robotVelocity the horizontal velocity of the robot in the field coordinate system
     */
    public TrajectoryParameters(
            Pose3d robotPose,
            Transform3d launcherTransform,
            Translation3d targetTranslation,
            Translation2d robotVelocity) {
        launcherPose = robotPose.transformBy(launcherTransform);
        displacement = targetTranslation.minus(getLauncherTranslation());
        // TODO: consider accounting for robot angular velocity in the lanucher velocity
        this.launcherVelocity = robotVelocity;

        horizontalDisplacement = new Translation2d(displacement.getX(), displacement.getY());
        verticalDisplacement = displacement.getZ();
        horizontalDistance = horizontalDisplacement.getNorm();
        elevationAngle = Math.atan2(verticalDisplacement, horizontalDistance);
    }

    /** Get the launcher position (fuel exit point; muzzle) in field coordinates. */
    public Translation3d getLauncherTranslation() {
        return launcherPose.getTranslation();
    }

    /** Get the rotation of the launcher's 0 pitch, 0 yaw direction, relative to the field. */
    public Rotation3d getLauncherRotation() {
        return launcherPose.getRotation();
    }

    /** Get the displacement vector from the launcher to the target in field coordinates. */
    public Translation3d getDisplacement() {
        return displacement;
    }

    /** Get the horizontal velocity of the launcher, which affects the fuel exit velocity. */
    public Translation2d getLauncherVelocity() {
        return launcherVelocity;
    }

    /** Get the 2D horizontal (x and y) displacement vector from the launcher to the target. */
    public Translation2d getHorizontalDisplacement() {
        return horizontalDisplacement;
    }

    /** Get the vertical (z) displacement from the launcher to the target; the net height. */
    public double getVerticalDisplacement() {
        return verticalDisplacement;
    }

    /**
     * Get the magnitude or length of the horizontal displacement from the launcher to the target.
     */
    public double getHorizontalDistance() {
        return horizontalDistance;
    }

    /** Get the pitch angle from the horizontal x-y plane up to the displacement vector. */
    public double getElevationAngle() {
        return elevationAngle;
    }

    /**
     * Get the pitch whose trajectory requires the minimum projectile speed to reach the target.
     *
     * <p>This angle is the average of the elevation angle and 90 degrees.
     */
    public double getMinimalSpeedPitch() {
        return (elevationAngle + Math.PI / 2) / 2;
    }

    /**
     * Get the maximum tilt of the launcher from the launcher roll and pitch. When the robot is flat
     * on the ground, this tilt is 0.
     *
     * <p>This maximum tilt is the maximum extra pitch that can be added or subtracted from the
     * field pitch.
     *
     * <p>The tilt is from 0.0 to pi.
     *
     * @return the maximum tilt angle of the launcher
     */
    public double getLauncherTilt() {
        return Math.acos(
                Math.cos(getLauncherRotation().getX()) * Math.cos(getLauncherRotation().getY()));
    }

    @Override
    public String toString() {
        Rotation3d launcherRotation = getLauncherRotation();
        return String.format(
                "TrajectoryParameters[launcherTranslation=%s, launcherRotation=Rotation3d(%.4f,"
                    + " %.4f, %.4f), displacement=%s, launcherVelocity=%s, elevationAngle=%.4f]",
                getLauncherTranslation(),
                launcherRotation.getX(),
                launcherRotation.getY(),
                launcherRotation.getZ(),
                displacement,
                launcherVelocity,
                elevationAngle);
    }
}
