package frc.robot.field;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import java.util.Optional;

public final class FieldZoning { // in meters
    // field dimensions
    private static final double FIELD_WIDTH = 8.07; // 317.7 inches
    private static final double FIELD_LENGTH = 16.54; // 651.2 inches
    // zone boundaries measured from alliance wall along x-axis
    private static final double BLUE_ALLIANCE_ZONE_DEPTH = 4.03; // 158.6 inches
    private static final double NEUTRAL_ALLIANCE_ZONE_DEPTH = 7.19; // 283 inches
    private static final double RED_ALLIANCE_ZONE_START = 12.51; // 492.6 inches
    // robot dimensions
    private static final double ROBOT_SIZE = 0.6858; // 27 inches square robot
    private static final double ROBOT_HALF_SIZE = ROBOT_SIZE / (2.0);
    // bump dimensions
    private static final double BUMP_WIDTH = 1.854; // 73.0 inches
    private static final double BUMP_DEPTH = 1.128; // 44.4 inches
    private static final double BUMP_HEIGHT = 0.1654; // 6.513 inches
    private static final double BUMP_HALF_WIDTH = BUMP_WIDTH / 2.0;
    private static final double BUMP_HALF_DEPTH = BUMP_DEPTH / 2.0;
    // bumps center offset by half depth
    private static final double RED_BUMP_CENTER_X = 11.946; // 12.51 - 0.564
    private static final double BLUE_BUMP_CENTER_X = 4.594; // 4.03 + 0.564
    // From field dimension drawing sheet 4: HUB center at 158.84 inches
    // Pattern: (73.00) (47.00) (73.00) - BUMP HUB BUMP
    // Left bump: 158.84 - 23.5 - 3 6.5 = 98.84 inches = 2.510m
    // Right bump: 158.84 + 23.5 + 36.5 = 218.84 inches = 5.558m
    private static final double LEFT_BUMP_CENTER_Y = 2.510; // 98.84 inches
    private static final double RIGHT_BUMP_CENTER_Y = 5.558; // 218.84 inches

    // helper
    private static double getMaxXExtent(Pose2d pose) {
        double centerX = pose.getX();
        double angle = pose.getRotation().getRadians();
        /**
         * for a square the max y extent is center + projection of half diagonal and the angle that
         * extends the furthest is at the top of the square that is rotated and this happens at >45
         * degrees from one of the sides
         */
        return centerX + ROBOT_HALF_SIZE * (Math.abs(Math.sin(angle)) + Math.abs(Math.cos(angle)));
    }

    private static double getMinXExtent(Pose2d pose) {
        // for a minimum y of a square this is the y position of whatever corner points
        // most downwards
        double centerX = pose.getX();
        double angle = pose.getRotation().getRadians();
        return centerX - ROBOT_HALF_SIZE * (Math.abs(Math.sin(angle)) + Math.abs(Math.cos(angle)));
    }

    private static double getMaxYExtent(Pose2d pose) {
        double centerY = pose.getY();
        double angle = pose.getRotation().getRadians();
        return centerY + ROBOT_HALF_SIZE * (Math.abs(Math.sin(angle)) + Math.abs(Math.cos(angle)));
    }

    private static double getMinYExtent(Pose2d pose) {
        double centerY = pose.getY();
        double angle = pose.getRotation().getRadians();
        return centerY - ROBOT_HALF_SIZE * (Math.abs(Math.sin(angle)) + Math.abs(Math.cos(angle)));
    }

    public static boolean isInBlueAllianceZone(Pose2d pose) {
        return getMinXExtent(pose) <= BLUE_ALLIANCE_ZONE_DEPTH;
    }

    public static boolean isInRedAllianceZone(Pose2d pose) {
        return getMaxXExtent(pose) >= RED_ALLIANCE_ZONE_START;
    }

    public static boolean isInNeutralZone(Pose2d pose) {
        return !isInBlueAllianceZone(pose) && !isInRedAllianceZone(pose);
    }

    public static boolean isInTeamAllianceZone(Pose2d pose) {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) {
            return false;
        }
        if (alliance.get() == Alliance.Blue) {
            return isInBlueAllianceZone(pose);
        } else {
            return isInRedAllianceZone(pose);
        }
    }

    // helper method to check collision with a single bump
    private static boolean isCollidingWithBump(
            Pose2d pose, double bumpCenterX, double bumpCenterY) {
        double robotMinX = getMinXExtent(pose);
        double robotMaxX = getMaxXExtent(pose);
        double robotMinY = getMinYExtent(pose);
        double robotMaxY = getMaxYExtent(pose);

        double bumpMinX = bumpCenterX - BUMP_HALF_DEPTH;
        double bumpMaxX = bumpCenterX + BUMP_HALF_DEPTH;
        double bumpMinY = bumpCenterY - BUMP_HALF_WIDTH;
        double bumpMaxY = bumpCenterY + BUMP_HALF_WIDTH;

        // AABB collision, check if rectangles overlap
        return robotMaxX >= bumpMinX
                && robotMinX <= bumpMaxX
                && robotMaxY >= bumpMinY
                && robotMinY <= bumpMaxY;
    }

    // check if robot is colliding with any bump on the field
    public static boolean isOnBump(Pose2d pose) {
        // TODO: since this uses AABB collision, there are false positives at bump corners
        return isCollidingWithBump(pose, BLUE_BUMP_CENTER_X, LEFT_BUMP_CENTER_Y)
                || isCollidingWithBump(pose, BLUE_BUMP_CENTER_X, RIGHT_BUMP_CENTER_Y)
                || isCollidingWithBump(pose, RED_BUMP_CENTER_X, LEFT_BUMP_CENTER_Y)
                || isCollidingWithBump(pose, RED_BUMP_CENTER_X, RIGHT_BUMP_CENTER_Y);
    }
}
