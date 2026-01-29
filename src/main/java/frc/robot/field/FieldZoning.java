package frc.robot.field;

import edu.wpi.first.math.geometry.Pose2d;

public final class FieldZoning {
    // field dimensions in meters
    private static final double FIELD_WIDTH = 8.07; // 317.7 inches
    private static final double FIELD_LENGTH = 16.54; // 651.2 inch
    // zone boundaries measured from aliance wall y-axis
    private static final double BLUE_ALLIANCE_ZONE_DEPTH = 4.03; // 158.6 inches
    private static final double NEUTRAL_ALLIANCE_ZONE_DEPTH = 7.19; // 283 inches
    private static final double RED_ALLIANCE_ZONE_START = 12.51; // 492.6 inches (651.2 - 158.6 too)
    // robot dimensions
    private static final double ROBOT_SIZE = 0.6858; // 27 inches square robot
    private static final double ROBOT_HALF_SIZE = ROBOT_SIZE / (2.0);

    // helper
    private static double getMaxYExtent(Pose2d pose) {
        double centerY = pose.getY();
        double angle = pose.getRotation().getRadians();
        /**
         * for a square the max y extent is center + projection of half diagonal and the angle that
         * extends the furthest is at the top of the square that is rotated and this happens at >45
         * degrees from one of the sides
         */
        return centerY + ROBOT_HALF_SIZE * (Math.abs(Math.sin(angle)) + Math.abs(Math.cos(angle)));
    }

    private static double getMinYExtent(Pose2d pose) {
        // for a minimum y of a square this is the y position of whatever corner points
        // most downwards
        double centerY = pose.getY();
        double angle = pose.getRotation().getRadians();
        return centerY - ROBOT_HALF_SIZE * (Math.abs(Math.sin(angle)) + Math.abs(Math.cos(angle)));
    }

    // check if robot is touching or inside blue alliance
    public static boolean isInBlueAllianceZone(Pose2d pose) {
        return (pose.getY() + ROBOT_HALF_SIZE) <= BLUE_ALLIANCE_ZONE_DEPTH;
    }

    public static boolean isInBlueAllianceZone(Pose2d pose, boolean takeInRotation) {
        if (!takeInRotation) {
            return isInBlueAllianceZone(pose);
        }
        return getMaxYExtent(pose) <= BLUE_ALLIANCE_ZONE_DEPTH;
    }

    // check if robot is touching or inside red alliance
    public static boolean isInRedAllianceZone(Pose2d pose) {
        return (pose.getY() - ROBOT_HALF_SIZE) >= RED_ALLIANCE_ZONE_START;
    }

    public static boolean isInRedAllianceZone(Pose2d pose, boolean takeInRotation) {
        if (!takeInRotation) {
            return isInRedAllianceZone(pose);
        }
        return getMinYExtent(pose) >= RED_ALLIANCE_ZONE_START;
    }

    // check if robot is touching or inside neutral zone
    public static boolean isInNeutralZone(Pose2d pose) {
        double minY = pose.getY() - ROBOT_HALF_SIZE;
        double maxY = pose.getY() + ROBOT_HALF_SIZE;
        return maxY >= BLUE_ALLIANCE_ZONE_DEPTH && minY <= RED_ALLIANCE_ZONE_START;
    }

    public static boolean isInNeutralZone(Pose2d pose, boolean takeInRotation) {
        if (!takeInRotation) {
            return isInNeutralZone(pose);
        }

        double minY = getMinYExtent(pose);
        double maxY = getMaxYExtent(pose);
        return maxY >= BLUE_ALLIANCE_ZONE_DEPTH && minY <= RED_ALLIANCE_ZONE_START;
    }
}
