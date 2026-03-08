package frc.robot.target;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import frc.robot.field.FieldConst;
import frc.robot.field.FieldZoning;
import frc.robot.trajectory.TrajectoryConstraints;
import frc.robot.trajectory.TrajectoryConstraints.Obstacle;
import frc.robot.trajectory.TrajectoryConstraints.SoftConstraint;

import java.util.Optional;

public class TargetSelector {
    public static Optional<LaunchTarget> selectTarget(Pose3d robotPose) {
        Optional<Alliance> optionalAlliance = DriverStation.getAlliance();
        if (optionalAlliance.isEmpty()) {
            return Optional.empty();
        }
        Alliance alliance = optionalAlliance.get();
        Pose2d robotPose2d = robotPose.toPose2d();
        Translation3d translation;
        boolean ignoresVertical;
        TrajectoryConstraints constraints;

        if (FieldZoning.isInTeamAllianceZone(robotPose2d)) {
            // Launch at hub from team alliance zone
            Distance hubX =
                    switch (alliance) {
                        case Blue -> FieldConst.BLUE_HUB_X;
                        case Red -> FieldConst.RED_HUB_X;
                    };
            translation = new Translation3d(hubX, FieldConst.HUB_Y, TargetConfig.HUB_TARGET_HEIGHT);
            ignoresVertical = false;
            double distance =
                    Math.hypot(
                            translation.getX() - robotPose2d.getX(),
                            translation.getY() - robotPose2d.getY());
            Obstacle hubFunnelObstacle =
                    new Obstacle(
                            distance - FieldConst.HUB_FUNNEL_OUTER_RADIUS.in(Meters),
                            FieldConst.HUB_HEIGHT
                                    .plus(TargetConfig.HUB_OBSTACLE_HEIGHT_LEEWAY)
                                    .in(Meters));
            // TODO: add hub activation time constraints
            constraints =
                    new TrajectoryConstraints(TargetConfig.HUB_SOFT_CONSTRAINT)
                            .withLowerObstacle(hubFunnelObstacle);
        } else {
            // Launch at a zone. Do team mirroring and field width mirroring
            ignoresVertical = true;
            constraints = new TrajectoryConstraints(SoftConstraint.MINIMIZE_PITCH);
            Distance targetX;
            Distance targetY;
            if (FieldZoning.isInNeutralZone(robotPose2d)) {
                // Launch at team alliance zone from neutral zone
                targetX = TargetConfig.TEAM_ALLIANCE_ZONE_TARGET_X;
                targetY = TargetConfig.TEAM_ALLIANCE_ZONE_TARGET_Y;
            } else {
                // Launch at neutral zone from opponent alliance zone
                targetX = TargetConfig.NEUTRAL_ZONE_TARGET_X;
                targetY = TargetConfig.NEUTRAL_ZONE_TARGET_Y;
            }
            // TODO: add field wall width and length upper obstacles (2)

            if (alliance == Alliance.Red) {
                // Mirror x coordinate if on red alliance
                targetX = FieldConst.FIELD_LENGTH.minus(targetX);
            }
            if (robotPose2d.getY() > FieldConst.FIELD_WIDTH.in(Meters) / 2) {
                // Mirror y coordinate if on the further half of the field width
                targetY = FieldConst.FIELD_WIDTH.minus(targetY);
            }
            translation = new Translation3d(targetX.in(Meters), targetY.in(Meters), 0.0);
        }
        constraints = constraints.withMaxHeight(TargetConfig.MAX_HEIGHT.in(Meters));
        return Optional.of(new LaunchTarget(translation, ignoresVertical, constraints));
    }
}
