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

    // Hub dimensions
    public static final Distance BLUE_HUB_X = Inches.of(182.2);
    public static final Distance RED_HUB_X = FIELD_LENGTH.minus(BLUE_HUB_X);
    public static final Distance HUB_Y = FIELD_WIDTH.div(2);
    public static final Distance HUB_SIZE = Inches.of(47.0); // hub base is a square
    public static final Distance HUB_FUNNEL_INNER_RADIUS = Inches.of(41.7).div(2);
    public static final Distance HUB_FUNNEL_OUTER_RADIUS =
            HUB_FUNNEL_INNER_RADIUS.times(2 / Math.sqrt(3)); // by hexagon geometry
    public static final Distance HUB_HEIGHT = Inches.of(72.0); // to top of funnel

    /* Side length of the square robot bumpers */
    public static final Distance ROBOT_SIZE = Inches.of(27);
    public static final Distance ROBOT_HALF_SIZE = ROBOT_SIZE.div(2);

    // Bump dimensions (for FieldZoning)
    protected static final double BUMP_WIDTH = Inches.of(73.0).in(Meters);
    protected static final double BUMP_DEPTH = Inches.of(44.4).in(Meters);
    protected static final double BUMP_HEIGHT = Inches.of(6.513).in(Meters);
    protected static final double BUMP_HALF_WIDTH = BUMP_WIDTH / 2.0;
    protected static final double BUMP_HALF_DEPTH = BUMP_DEPTH / 2.0;
    protected static final double BLUE_BUMP_CENTER_X = BLUE_HUB_X.in(Meters);
    protected static final double RED_BUMP_CENTER_X = RED_HUB_X.in(Meters);
    protected static final double RIGHT_BUMP_CENTER_Y =
            HUB_Y.in(Meters) - HUB_SIZE.in(Meters) / 2 - BUMP_HALF_WIDTH;
    protected static final double LEFT_BUMP_CENTER_Y = FIELD_WIDTH.in(Meters) - RIGHT_BUMP_CENTER_Y;

    // Trench dimensions (for FieldZoning)
    public static final Distance TRENCH_OPENING_WIDTH = Inches.of(50.34);
    public static final Distance TRENCH_OPENING_HEIGHT = Inches.of(22.25);
    public static final Distance TRENCH_WIDTH = Inches.of(65.65);
    public static final Distance TRENCH_DEPTH = Inches.of(47.0);
    public static final Distance BLUE_TRENCH_CENTER_X = BLUE_HUB_X;
    public static final Distance RED_TRENCH_CENTER_X = RED_HUB_X;

    // See: https://firstfrc.blob.core.windows.net/frc2026/FieldAssets/2026-field-dimension-dwgs.pdf
}
