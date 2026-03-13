package frc.robot.field;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import java.util.Optional;
import java.util.function.Supplier;

public final class FieldZoning implements Sendable { // in meters
    private final Supplier<Pose2d> robotPoseSupplier;

    /** Private constructor for creating a sendable. */
    private FieldZoning(Supplier<Pose2d> robotPoseSupplier) {
        this.robotPoseSupplier = robotPoseSupplier;
    }

    // rectangle rotation helpers
    /*
     * for a square the max y extent is center + projection of half diagonal and the angle that
     * extends the furthest is at the top of the square that is rotated and this happens at >45
     * degrees from one of the sides
     */

    /** Get how far the robot bumpers extend, axis-aligned, from the center. */
    private static double getExtent(Rotation2d rotation) {
        return FieldConst.ROBOT_HALF_SIZE.in(Meters)
                * (Math.abs(rotation.getSin()) + Math.abs(rotation.getCos()));
    }

    private static double getMaxXExtent(Pose2d pose) {
        return pose.getX() + getExtent(pose.getRotation());
    }

    private static double getMinXExtent(Pose2d pose) {
        return pose.getX() - getExtent(pose.getRotation());
    }

    private static double getMaxYExtent(Pose2d pose) {
        return pose.getY() + getExtent(pose.getRotation());
    }

    private static double getMinYExtent(Pose2d pose) {
        return pose.getY() - getExtent(pose.getRotation());
    }

    public static boolean isInBlueAllianceZone(Pose2d pose) {
        return getMinXExtent(pose) <= FieldConst.ALLIANCE_ZONE_DEPTH.in(Meters);
    }

    public static boolean isInRedAllianceZone(Pose2d pose) {
        return getMaxXExtent(pose)
                >= FieldConst.FIELD_LENGTH.minus(FieldConst.ALLIANCE_ZONE_DEPTH).in(Meters);
    }

    public static boolean isInNeutralZone(Pose2d pose) {
        return !isInBlueAllianceZone(pose) && !isInRedAllianceZone(pose);
    }

    public static boolean isInTeamAllianceZone(Pose2d pose) {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) {
            return false; // Return false for unknown alliance
        }
        return alliance.get() == Alliance.Blue
                ? isInBlueAllianceZone(pose)
                : isInRedAllianceZone(pose);
    }

    // helper method to check collision with a single bump
    private static boolean isCollidingWithBump(
            Pose2d pose, double bumpCenterX, double bumpCenterY) {
        double robotMinX = getMinXExtent(pose);
        double robotMaxX = getMaxXExtent(pose);
        double robotMinY = getMinYExtent(pose);
        double robotMaxY = getMaxYExtent(pose);

        double bumpMinX = bumpCenterX - FieldConst.BUMP_HALF_DEPTH;
        double bumpMaxX = bumpCenterX + FieldConst.BUMP_HALF_DEPTH;
        double bumpMinY = bumpCenterY - FieldConst.BUMP_HALF_WIDTH;
        double bumpMaxY = bumpCenterY + FieldConst.BUMP_HALF_WIDTH;

        // AABB collision, check if rectangles overlap
        return robotMaxX >= bumpMinX
                && robotMinX <= bumpMaxX
                && robotMaxY >= bumpMinY
                && robotMinY <= bumpMaxY;
    }

    // check if robot is colliding with any bump on the field
    public static boolean isOnBump(Pose2d pose) {
        // TODO: since this uses AABB collision, there are false positives at bump corners
        return isCollidingWithBump(
                        pose, FieldConst.BLUE_BUMP_CENTER_X, FieldConst.LEFT_BUMP_CENTER_Y)
                || isCollidingWithBump(
                        pose, FieldConst.BLUE_BUMP_CENTER_X, FieldConst.RIGHT_BUMP_CENTER_Y)
                || isCollidingWithBump(
                        pose, FieldConst.RED_BUMP_CENTER_X, FieldConst.LEFT_BUMP_CENTER_Y)
                || isCollidingWithBump(
                        pose, FieldConst.RED_BUMP_CENTER_X, FieldConst.RIGHT_BUMP_CENTER_Y);
    }

    /**
     * Check if the exact position of the translation, ignoring robot size and rotation, is near the
     * trench within the given maximum x distance.
     */
    public static boolean isNearTrench(Translation2d point, Distance maxDistanceX) {
        if (Math.min(point.getY(), FieldConst.FIELD_WIDTH.in(Meters) - point.getY())
                <= FieldConst.TRENCH_OPENING_WIDTH.in(Meters)) {
            if (Math.min(
                            Math.abs(point.getX() - FieldConst.BLUE_TRENCH_CENTER_X.in(Meters)),
                            Math.abs(point.getX() - FieldConst.RED_TRENCH_CENTER_X.in(Meters)))
                    <= maxDistanceX.in(Meters)) {
                return true;
            }
        }
        return false;
    }

    public static Sendable getSendable(Supplier<Pose2d> robotPoseSupplier) {
        return new FieldZoning(robotPoseSupplier);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty(
                "zone",
                () -> {
                    Pose2d pose = robotPoseSupplier.get();
                    if (FieldZoning.isInRedAllianceZone(pose)) return "Red";
                    if (FieldZoning.isInBlueAllianceZone(pose)) return "Blue";
                    return "Neutral";
                },
                null);
        builder.addBooleanProperty(
                "team alliance zone",
                () -> FieldZoning.isInTeamAllianceZone(robotPoseSupplier.get()),
                null);
        builder.addBooleanProperty(
                "red alliance zone",
                () -> FieldZoning.isInRedAllianceZone(robotPoseSupplier.get()),
                null);
        builder.addBooleanProperty(
                "blue alliance zone",
                () -> FieldZoning.isInBlueAllianceZone(robotPoseSupplier.get()),
                null);
        builder.addBooleanProperty(
                "neutral zone", () -> FieldZoning.isInNeutralZone(robotPoseSupplier.get()), null);
        builder.addBooleanProperty(
                "on bump", () -> FieldZoning.isOnBump(robotPoseSupplier.get()), null);
        builder.addBooleanProperty(
                "near trench (~0.6m)",
                () ->
                        isNearTrench(
                                robotPoseSupplier.get().getTranslation(),
                                FieldConst.TRENCH_DEPTH.div(2)),
                null);
    }
}
