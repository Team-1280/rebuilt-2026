package frc.robot.launcher;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.launcher.feeder.FeederSubsystem;
import frc.robot.launcher.hood.HoodSubsystem;
import frc.robot.launcher.shooter.ShooterSubsystem;
import frc.robot.launcher.turret.TurretSubsystem;

import java.util.Optional;
import java.util.function.Supplier;

/** Class containing and managing all launcher subsystems */
public class LauncherAssembly {
    public final ShooterSubsystem shooter = new ShooterSubsystem();
    public final FeederSubsystem feeder = new FeederSubsystem();
    public final HoodSubsystem hood = new HoodSubsystem();
    public final TurretSubsystem turret = new TurretSubsystem();

    public Command runAutoAim(
            Supplier<Pose3d> robotPoseSupplier, Supplier<Translation3d> targetPositionSupplier) {
        Runnable run =
                () -> {
                    trajectoryAutoAim(robotPoseSupplier.get(), targetPositionSupplier.get());
                };
        Runnable end =
                () -> {
                    shooter.stop();
                    feeder.stop();
                };
        return Commands.runEnd(run, end, shooter, feeder, hood, turret);
    }

    private void trajectoryAutoAim(Pose3d robotPose, Translation3d targetPosition) {
        // TODO: use robot to muzzle translation instead of null
        // The muzzle reference frame (pointing with 0 yaw and pitch), with translation and rotation
        Pose3d muzzlePose = robotPose.transformBy(new Transform3d(null, Rotation3d.kZero));
        Translation3d displacement = targetPosition.minus(muzzlePose.getTranslation());

        // TODO: trajectory calculations
        boolean shouldLaunch = true; // TODO
        Rotation3d launchFieldDirection = null; // TODO
        Optional<LinearVelocity> launchVelocity = Optional.empty(); // TODO

        Rotation3d launchRobotDirection = launchFieldDirection.relativeTo(muzzlePose.getRotation());
        Optional<AngularVelocity> shooterAngularVelocity = launchVelocity.map(v -> null); // TODO

        if (shouldLaunch) {
            feeder.start();
        } else {
            feeder.stop();
        }
        shooterAngularVelocity.ifPresentOrElse(shooter::moveAngularVelocity, shooter::stop);
        aimDirection(launchRobotDirection);
    }

    public void aimDirection(Angle yaw, Angle pitch) {
        turret.moveYaw(yaw);
        hood.movePitch(pitch);
    }

    public void aimDirection(Rotation3d rotation) {
        // Note: roll is ignored since it doesn't affect direction
        aimDirection(rotation.getMeasureZ(), rotation.getMeasureY());
    }

    public void aimDirection(Translation3d direction) {
        if (direction.getX() == 0.0 && direction.getY() == 0.0) {
            // Gimbal lock: can't determine yaw
            return;
        }
        double yawRad = Math.atan2(direction.getY(), direction.getX());
        double pitchRad = Math.asin(direction.getZ() / direction.getNorm());
        aimDirection(Radians.of(yawRad), Radians.of(pitchRad));
    }

    public void stow() {
        shooter.stop();
        feeder.stop();
        hood.stow();
        turret.stow();
    }
}
