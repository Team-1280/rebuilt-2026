package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import frc.robot.field.FieldConst;

public final class DriveConfig implements Sendable {
    /** Pose to reset to for a binding. Default: lined up against team hub and facing it. */
    public static final Pose2d RESET_POSE =
            DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                    ? new Pose2d(
                            FieldConst.BLUE_HUB_X
                                    .minus(FieldConst.HUB_SIZE.div(2))
                                    .minus(FieldConst.ROBOT_HALF_SIZE),
                            FieldConst.FIELD_WIDTH.div(2),
                            Rotation2d.fromDegrees(0.0))
                    : new Pose2d(
                            FieldConst.RED_HUB_X
                                    .plus(FieldConst.HUB_SIZE.div(2))
                                    .plus(FieldConst.ROBOT_HALF_SIZE),
                            FieldConst.FIELD_WIDTH.div(2),
                            Rotation2d.fromDegrees(180.0));

    /** Enable or disable driving. */
    public static boolean enableDriving = true;

    /** Maximum throttle linear drive speed. */
    public static LinearVelocity maxSpeed = MetersPerSecond.of(2.5);

    /** Maximum throttle angular drive speed. */
    public static AngularVelocity maxAngularSpeed = RotationsPerSecond.of(0.75);

    /** Minimum registerable linear drive speed. */
    public static LinearVelocity speedDeadband = maxSpeed.times(0.1);

    /** Minimum registerable angular drive speed. */
    public static AngularVelocity angularSpeedDeadband = maxAngularSpeed.times(0.1);

    /** Maximum X distance from trench bar to center of robot to automatically stow launcher. */
    public static Distance trenchLauncherStowDistance = Meters.of(1.0);

    public static final SwerveRequest.FieldCentric swerveRequest =
            new SwerveRequest.FieldCentric()
                    .withDeadband(DriveConfig.speedDeadband)
                    .withRotationalDeadband(DriveConfig.angularSpeedDeadband)
                    .withDriveRequestType(DriveRequestType.Velocity);

    private DriveConfig() {}

    public static Sendable getSendable() {
        return new DriveConfig();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addBooleanProperty(
                "ENABLE DRIVING",
                () -> enableDriving,
                (enable) -> {
                    enableDriving = enable;
                });
        builder.addDoubleProperty(
                "max speed (m per s)",
                () -> maxSpeed.in(MetersPerSecond),
                (speed) -> {
                    maxSpeed = MetersPerSecond.of(speed);
                });
        builder.addDoubleProperty(
                "max angular speed (rot per s)",
                () -> maxAngularSpeed.in(RotationsPerSecond),
                (angularSpeed) -> {
                    maxAngularSpeed = RotationsPerSecond.of(angularSpeed);
                });
        builder.addDoubleProperty(
                "speed deadband (m per s)",
                () -> speedDeadband.in(MetersPerSecond),
                (deadband) -> {
                    speedDeadband = MetersPerSecond.of(deadband);
                    swerveRequest.withDeadband(speedDeadband);
                });
        builder.addDoubleProperty(
                "angular speed deadband (rot per s)",
                () -> angularSpeedDeadband.in(RotationsPerSecond),
                (deadband) -> {
                    angularSpeedDeadband = RotationsPerSecond.of(deadband);
                    swerveRequest.withRotationalDeadband(angularSpeedDeadband);
                });
        builder.addDoubleProperty(
                "trench launcher stow x distance (m)",
                () -> trenchLauncherStowDistance.in(Meters),
                (x) -> {
                    trenchLauncherStowDistance = Meters.of(x);
                });
    }
}
