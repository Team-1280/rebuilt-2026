package frc.robot.field;

public final class FieldConst {
    // field dimensions
    public static final double FIELD_WIDTH = 8.07; // 317.7 inches
    public static final double FIELD_LENGTH = 16.54; // 651.2 inches

    // zone boundaries measured from alliance wall along x-axis
    public static final double BLUE_ALLIANCE_ZONE_DEPTH = 4.03; // 158.6 inches
    public static final double NEUTRAL_ALLIANCE_ZONE_DEPTH = 7.19; // 283 inches
    public static final double RED_ALLIANCE_ZONE_START = 12.51; // 492.6 inches
    // robot dimensions
    public static final double ROBOT_SIZE = 0.6858; // 27 inches square robot
    public static final double ROBOT_HALF_SIZE = ROBOT_SIZE / (2.0);
    // bump dimensions
    public static final double BUMP_WIDTH = 1.854; // 73.0 inches
    public static final double BUMP_DEPTH = 1.128; // 44.4 inches
    public static final double BUMP_HEIGHT = 0.1654; // 6.513 inches
    public static final double BUMP_HALF_WIDTH = BUMP_WIDTH / 2.0;
    public static final double BUMP_HALF_DEPTH = BUMP_DEPTH / 2.0;
    // bumps center offset by half depth
    public static final double RED_BUMP_CENTER_X = 11.946; // 12.51 - 0.564
    public static final double BLUE_BUMP_CENTER_X = 4.594; // 4.03 + 0.564
    // From field dimension drawing sheet 4: HUB center at 158.84 inches
    // Pattern: (73.00) (47.00) (73.00) - BUMP HUB BUMP
    // Left bump: 158.84 - 23.5 - 3 6.5 = 98.84 inches = 2.510m
    // Right bump: 158.84 + 23.5 + 36.5 = 218.84 inches = 5.558m
    public static final double LEFT_BUMP_CENTER_Y = 2.510; // 98.84 inches
    public static final double RIGHT_BUMP_CENTER_Y = 5.558; // 218.84 inches
}
