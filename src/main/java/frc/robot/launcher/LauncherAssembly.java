package frc.robot.launcher;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.launcher.feeder.FeederSubsystem;
import frc.robot.launcher.hood.HoodSubsystem;
import frc.robot.launcher.shooter.ShooterSubsystem;
import frc.robot.launcher.turret.TurretSubsystem;

/** Class containing and managing all launcher subsystems */
public class LauncherAssembly {
    public final ShooterSubsystem shooter = new ShooterSubsystem();
    public final FeederSubsystem feeder = new FeederSubsystem();
    public final HoodSubsystem hood = new HoodSubsystem();
    public final TurretSubsystem turret = new TurretSubsystem();

    public Command runAutoAim() { // TODO
        Runnable run =
                () -> {
                    // TODO
                };
        Runnable end =
                () -> {
                    shooter.stop();
                    feeder.stop();
                };
        return Commands.runEnd(run, end, shooter, feeder, hood, turret);
    }

    private void trajectoryAutoAim() {
        // TODO
    }

    public void aimDirection(Angle pitch, Angle yaw) {
        hood.movePitch(pitch);
        turret.moveYaw(yaw);
    }

    public void aimDirection(Translation3d direction) {
        if (direction.getX() == 0.0 && direction.getY() == 0.0) {
            // Gimbal lock: can't determine yaw
            return;
        }
        double pitchRad = Math.asin(direction.getZ() / direction.getNorm());
        double yawRad = Math.atan2(direction.getY(), direction.getX());
        aimDirection(Radians.of(pitchRad), Radians.of(yawRad));
    }

    public void stow() {
        shooter.stop();
        feeder.stop();
        hood.stow();
        turret.stow();
    }
}
