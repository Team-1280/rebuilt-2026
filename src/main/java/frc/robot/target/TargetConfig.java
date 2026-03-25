package frc.robot.target;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

import frc.robot.field.FieldConst;
import frc.robot.trajectory.TrajectoryConstraints.SoftConstraint;

public class TargetConfig {
    // TODO: tune all

    /**
     * How long it takes for the fuel to be assessed as scored by the hub after the fuel reaches the
     * end of the calculated trajectory.
     */
    public static final double FUEL_SCORING_LATENCY = 1.0;

    /** Extra time placed on both sides of the calculated time window of hub activation. */
    public static final double HUB_STATUS_TIME_BUFFER = 2.0;

    public static final Distance MAX_HEIGHT = Meters.of(Double.POSITIVE_INFINITY);

    // Hub target
    public static final Distance HUB_TARGET_HEIGHT = FieldConst.HUB_HEIGHT.plus(Inches.of(0.0));
    public static final SoftConstraint HUB_SOFT_CONSTRAINT = SoftConstraint.MINIMIZE_SPEED;
    public static final Distance HUB_OBSTACLE_HEIGHT_LEEWAY = Inches.of(24.0);

    // Zone target fuel passing in general
    public static final Angle PASSING_PITCH_GRADIENT_LOW = Degrees.of(15.0);
    public static final Angle PASSING_PITCH_GRADIENT_HIGH = Degrees.of(45.0);

    // Team alliance zone target closest to blue origin
    public static final Distance TEAM_ALLIANCE_ZONE_TARGET_X = FieldConst.BLUE_HUB_X;
    public static final Distance TEAM_ALLIANCE_ZONE_TARGET_Y =
            FieldConst.HUB_Y.minus(FieldConst.HUB_SIZE.div(2)).minus(Meters.of(0.5));
    public static final double TEAM_ALLIANCE_ZONE_SPEED_FRACTION = 0.8;

    // Neutral zone target closest to blue origin (when robot is in red alliance zone)
    public static final Distance NEUTRAL_ZONE_TARGET_X = FieldConst.RED_HUB_X;
    public static final Distance NEUTRAL_ZONE_TARGET_Y =
            FieldConst.HUB_Y.minus(FieldConst.HUB_SIZE.div(2)).minus(Meters.of(0.4));
    public static final double NEUTRAL_ZONE_SPEED_FRACTION = 0.8;
}
