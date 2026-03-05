package frc.robot.target;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;

import frc.robot.field.FieldConst;
import frc.robot.trajectory.TrajectoryConstraints.SoftConstraint;

public class TargetConfig {
    // TODO: tune all

    public static final Distance MAX_HEIGHT = Meters.of(Double.POSITIVE_INFINITY);

    // Hub target
    public static final Distance HUB_TARGET_HEIGHT = FieldConst.HUB_HEIGHT.plus(Meters.of(0.0));
    public static final SoftConstraint HUB_SOFT_CONSTRAINT = SoftConstraint.MINIMIZE_SPEED;
    public static final Distance HUB_OBSTACLE_HEIGHT_LEEWAY = Meters.of(0.1);

    // Team alliance zone target closest to blue origin
    public static final Distance TEAM_ALLIANCE_ZONE_TARGET_X = FieldConst.BLUE_HUB_X;
    public static final Distance TEAM_ALLIANCE_ZONE_TARGET_Y =
            FieldConst.HUB_Y.minus(FieldConst.HUB_SIZE.div(2)).minus(Meters.of(0.5));

    // Neutral zone target closest to blue origin (when robot is in red alliance zone)
    public static final Distance NEUTRAL_ZONE_TARGET_X = FieldConst.RED_HUB_X;
    public static final Distance NEUTRAL_ZONE_TARGET_Y =
            FieldConst.HUB_Y.minus(FieldConst.HUB_SIZE.div(2)).minus(Meters.of(0.4));
}
