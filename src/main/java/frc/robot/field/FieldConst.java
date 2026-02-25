package frc.robot.field;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;

public final class FieldConst {
    // Field dimensions
    public static final Distance FIELD_WIDTH = Inches.of(317.7);
    public static final Distance FIELD_LENGTH = Inches.of(651.2);

    /** Depth of each alliance zone along the x axis */
    public static final Distance ALLIANCE_ZONE_DEPTH = Inches.of(158.6);

    /* Side length of the square robot bumpers */
    public static final Distance ROBOT_SIZE = Inches.of(27);
    public static final Distance ROBOT_HALF_SIZE = ROBOT_SIZE.div(2);

    // bump dimensions (for FieldZoning)
    protected static final double BUMP_WIDTH = Inches.of(73.0).in(Meters);
    protected static final double BUMP_DEPTH = Inches.of(44.4).in(Meters);
    protected static final double BUMP_HEIGHT = Inches.of(6.513).in(Meters);
    protected static final double BUMP_HALF_WIDTH = BUMP_WIDTH / 2.0;
    protected static final double BUMP_HALF_DEPTH = BUMP_DEPTH / 2.0;
    // bumps center offset by half depth
    protected static final double RED_BUMP_CENTER_X = 11.946; // 12.51 - 0.564
    protected static final double BLUE_BUMP_CENTER_X = 4.594; // 4.03 + 0.564
    // From field dimension drawing sheet 4: HUB center at 158.84 inches
    // Pattern: (73.00) (47.00) (73.00) - BUMP HUB BUMP
    // Right bump: 158.84 - 23.5 - 36.5 = 98.84 inches = 2.510m
    // left bump: 158.84 + 23.5 + 36.5 = 218.84 inches = 5.558m
    protected static final double LEFT_BUMP_CENTER_Y = 5.558; // 218.84 inches
    protected static final double RIGHT_BUMP_CENTER_Y = 2.510; // 98.84 inches
}
