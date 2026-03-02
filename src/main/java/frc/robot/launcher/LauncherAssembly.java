package frc.robot.launcher;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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

/** Class containing and managing all launcher subsystems. */
public class LauncherAssembly implements Sendable {
    public final ShooterSubsystem shooter = new ShooterSubsystem();
    public final FeederSubsystem feeder = new FeederSubsystem();
    public final HoodSubsystem hood = new HoodSubsystem();
    public final TurretSubsystem turret = new TurretSubsystem();

    private boolean shootingEnabled = true;

    /** Allow fuel to be shot. */
    public void enableShooting() {
        shootingEnabled = true;
    }

    /** Stop shooting and don't allow fuel to be shot. */
    public void disableShooting() {
        stopShooting();
        shootingEnabled = false;
    }

    /** Stop shooting fuel. */
    private void stopShooting() {
        shooter.stop();
        feeder.stop();
    }

    /** Stop flywheels and stow mechanisms. */
    public void stow() {
        stopShooting();
        hood.stow();
        turret.stow();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        SmartDashboard.putData("launcher/shooter", shooter);
        SmartDashboard.putData("launcher/feeder", feeder);
        SmartDashboard.putData("launcher/hood", hood);
        SmartDashboard.putData("launcher/turret", turret);
    }

    /** Set the launcher direction to the given robot pitch and yaw. */
    public void aimDirection(Angle pitch, Angle yaw) {
        hood.movePitch(pitch);
        turret.moveYaw(yaw);
    }

    /** Set the launcher direction to the given 3D vector. */
    public void aimDirection(Translation3d direction) {
        if (direction.getX() == 0.0 && direction.getY() == 0.0) {
            // Gimbal lock: can't determine yaw
            DriverStation.reportWarning(
                    "LauncherAssembly cannot aim direction " + direction + " (gimbal lock)", true);
            return;
        }
        double pitchRad = Math.asin(direction.getZ() / direction.getNorm());
        double yawRad = Math.atan2(direction.getY(), direction.getX());
        aimDirection(Radians.of(pitchRad), Radians.of(yawRad));
    }

    /** Aim the launcher pitch and yaw to the given trajectory's. */
    private void aimTrajectory(Trajectory trajectory) {
        aimDirection(
                Radians.of(trajectory.getLauncherPitch()), Radians.of(trajectory.getLauncherYaw()));
    }

    /**
     * Move shooter flywheel at trajectory's launch speed and start or stop feeding.
     *
     * <p>If shooting is disabled, does nothing.
     *
     * <p>Return a boolean of whether the launcher is feeding and shooting.
     */
    private boolean shootTrajectory(Trajectory trajectory) {
        if (!shootingEnabled) {
            return false;
        }
        boolean shoot = trajectory.isValid();
        if (shoot) {
            feeder.start();
        } else {
            feeder.stop();
        }
        shooter.moveAngularVelocity(RadiansPerSecond.of(trajectory.getFlywheelSpeed()));
        return shoot;
    }

    /**
     * Set the launcher to aim and, if possible, shoot, with the given trajectory.
     *
     * <p>Return a boolean of whether the launcher is feeding and shooting.
     */
    private boolean launchTrajectory(Trajectory trajectory) {
        aimTrajectory(trajectory);
        return shootTrajectory(trajectory);
    }

    /** Calculate the trajectory for the given target and parameters. Trajectory may be invalid. */
    private Trajectory calculateTargetTrajectory(
            LaunchTarget target, Pose3d robotPose, Translation2d robotVelocity) {
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
        return target.ignoresVertical()
                ? TrajectorySolver.solveIgnoringVertical(parameters, constraints)
                : TrajectorySolver.solve(parameters, constraints);
    }

    /**
     * Set the launcher to aim and, if possible, shoot, at the given target, for this instant.
     *
     * <p>Return a boolean of whether the launcher is feeding and shooting.
     */
    public boolean launchTargetTrajectory(
            LaunchTarget target, Pose3d robotPose, Translation2d robotVelocity) {
        Trajectory trajectory = calculateTargetTrajectory(target, robotPose, robotVelocity);
        return launchTrajectory(trajectory);
    }
}
