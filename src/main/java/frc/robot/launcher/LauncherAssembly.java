package frc.robot.launcher;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.launcher.feeder.FeederSubsystem;
import frc.robot.launcher.hood.HoodConst;
import frc.robot.launcher.hood.HoodSubsystem;
import frc.robot.launcher.shooter.ShooterConfig;
import frc.robot.launcher.shooter.ShooterSubsystem;
import frc.robot.launcher.turret.TurretSubsystem;
import frc.robot.target.LaunchTarget;
import frc.robot.trajectory.Trajectory;
import frc.robot.trajectory.TrajectoryConstraints;
import frc.robot.trajectory.TrajectoryParameters;
import frc.robot.trajectory.TrajectorySolver;

import java.util.Optional;
import java.util.function.Supplier;

/** Class containing and managing all launcher subsystems */
public class LauncherAssembly {
    public final ShooterSubsystem shooter = new ShooterSubsystem();
    public final FeederSubsystem feeder = new FeederSubsystem();
    public final HoodSubsystem hood = new HoodSubsystem();
    public final TurretSubsystem turret = new TurretSubsystem();

    private boolean launchingEnabled = true;

    /** Allow fuel to be launched */
    public void enableLaunching() {
        launchingEnabled = true;
    }

    /** Stop and don't allow fuel to be launched */
    public void disableLaunching() {
        stopLaunching();
        launchingEnabled = false;
    }

    /** Stop launching fuel */
    private void stopLaunching() {
        shooter.stop();
        feeder.stop();
    }

    /** Stop flywheels and stow mechanisms */
    public void stow() {
        stopLaunching();
        hood.stow();
        turret.stow();
    }

    /** Set the launcher direction to the given robot pitch and yaw */
    public void aimDirection(Angle pitch, Angle yaw) {
        hood.movePitch(pitch);
        turret.moveYaw(yaw);
    }

    /** Set the launcher direction to the given 3D vector */
    public void aimDirection(Translation3d direction) {
        if (direction.getX() == 0.0 && direction.getY() == 0.0) {
            // Gimbal lock: can't determine yaw
            return;
        }
        double pitchRad = Math.asin(direction.getZ() / direction.getNorm());
        double yawRad = Math.atan2(direction.getY(), direction.getX());
        aimDirection(Radians.of(pitchRad), Radians.of(yawRad));
    }

    /** Set the launcher to aim and, if possible, launch, with the given trajectory */
    private void setTrajectory(Trajectory trajectory) {
        aimDirection(
                Radians.of(trajectory.getLauncherPitch()), Radians.of(trajectory.getLauncherYaw()));
        if (launchingEnabled) {
            if (trajectory.isValid()) {
                feeder.start();
            } else {
                feeder.stop();
            }
            shooter.moveAngularVelocity(RadiansPerSecond.of(trajectory.getFlywheelSpeed()));
        }
    }

    /** Set the launcher to aim and, if possible, launch, at the given target */
    private void setTarget(Pose3d robotPose, Translation2d robotVelocity, LaunchTarget target) {
        TrajectoryParameters parameters =
                new TrajectoryParameters(
                        robotPose,
                        robotVelocity,
                        LauncherConst.ROBOT_TO_LAUNCHER_TRANSFORM,
                        target.translation());
        TrajectoryConstraints constraints =
                target.constraints()
                        .withMinLauncherPitch(HoodConst.MIN_PITCH.in(Radians))
                        .withMaxLauncherPitch(HoodConst.MAX_PITCH.in(Radians))
                        .withMaxFlywheelSpeed(
                                ShooterConfig.MAX_ANGULAR_VELOCITY.in(RadiansPerSecond));
        Trajectory trajectory =
                target.ignoresVertical()
                        ? TrajectorySolver.solveIgnoringVertical(parameters, constraints)
                        : TrajectorySolver.solve(parameters, constraints);
        setTrajectory(trajectory);
    }

    /**
     * Construct a command that sets the launcher subsystems to constantly launch at a provided
     * (optional) target, until interrupted. (By default, the launcher stows once this command
     * ends.)
     */
    public Command runAutomaticLaunching(
            Supplier<Pose3d> robotPoseSupplier,
            Supplier<Translation2d> robotVelocitySupplier,
            Supplier<Optional<LaunchTarget>> targetSupplier) {
        Runnable run =
                () -> {
                    Optional<LaunchTarget> target = targetSupplier.get();
                    if (target.isPresent()) {
                        setTarget(
                                robotPoseSupplier.get(), robotVelocitySupplier.get(), target.get());
                    } else {
                        // If no target, then stow
                        stow();
                    }
                };
        Runnable end = this::stow;
        return Commands.runEnd(run, end, shooter, feeder, hood, turret);
    }
}
