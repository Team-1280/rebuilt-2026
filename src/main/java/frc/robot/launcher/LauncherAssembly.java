package frc.robot.launcher;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;

import frc.robot.launcher.feeder.FeederSubsystem;
import frc.robot.launcher.hood.HoodConst;
import frc.robot.launcher.hood.HoodSubsystem;
import frc.robot.launcher.shooter.ShooterConfig;
import frc.robot.launcher.shooter.ShooterSubsystem;
import frc.robot.launcher.turret.TurretSubsystem;
import frc.robot.target.LaunchTarget;
import frc.robot.trajectory.LaunchSpeed;
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

    /** An array of all launcher subsystems, useful for command subsystem requirements. */
    public final Subsystem[] subsystems = {shooter, feeder, hood, turret};

    /** Launch speed multiplier for trajectory that can be used to correct error. */
    private double trajectorySpeedMultiplier = 0.95;

    /** An offset to apply to turret yaw after it is calculated in trajectory, to mitigate bias. */
    private Angle trajectoryYawOffset = Degrees.of(0.0);

    /** Launch speed for when doing fixed launching. */
    private LinearVelocity fixedLaunchSpeed = MetersPerSecond.of(7.5);

    /** Launch pitch for when doing fixed launching. */
    private Angle fixedLaunchPitch = HoodConst.MAX_PITCH;

    /** Launch yaw for when doing fixed launching. */
    private Angle fixedLaunchYaw = Degrees.of(0.0);

    /** Approximate time for the motor to receive and react to a control request, in seconds. */
    private double motorLatency = 0.0; // TODO: tune or remove

    /** Filter of yaw velocity for use in motor latency compensation. */
    private final LinearFilter trajectoryYawVelocityFilter = LinearFilter.movingAverage(4);

    /** Record of the previous trajectory yaw for use in motor latency compensation. */
    private Angle previousTrajectoryYaw = turret.getYaw();

    /** Record of the previous trajectory time for use in motor latency compensation. */
    private double previousTrajectoryTime = Timer.getFPGATimestamp();

    /** Stop flywheels and stow mechanisms. */
    public void stow() {
        shooter.stop();
        feeder.stop();
        hood.stow();
        turret.stow();
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

    /** Aim the launcher pitch and yaw to the given trajectory's, compensating for latency. */
    public void aimTrajectory(Trajectory trajectory) {
        Angle yaw = Radians.of(trajectory.getLauncherYaw()).plus(trajectoryYawOffset);

        double time = Timer.getFPGATimestamp();
        double yawDifference = yaw.minus(previousTrajectoryYaw).in(Radians);
        double timeDifference = time - previousTrajectoryTime;
        previousTrajectoryYaw = yaw;
        previousTrajectoryTime = time;

        if (yawDifference > Math.toRadians(15) || timeDifference > 0.10) {
            // reset the yaw velocity due to yaw or time discontinuity
            trajectoryYawVelocityFilter.reset();
        } else {
            double yawVelocity = yawDifference / timeDifference;
            double filteredYawVelocity = trajectoryYawVelocityFilter.calculate(yawVelocity);
            yaw = yaw.plus(Radians.of(filteredYawVelocity * motorLatency));
        }

        aimDirection(Radians.of(trajectory.getLauncherPitch()), yaw);
    }

    /**
     * Start or stop feeding fuel based on if the trajectory is valid and yaw is within tolerance.
     *
     * <p>Return a boolean of whether the launcher is feeding.
     */
    public boolean feedTrajectory(Trajectory trajectory) {
        boolean feed = trajectory.isValid() && turret.withinTolerance();
        if (feed) {
            feeder.start();
        } else {
            feeder.stop();
        }
        return feed;
    }

    /** Move shooter flywheel at trajectory's launch speed. */
    public void shootTrajectory(Trajectory trajectory) {
        shooter.moveAngularVelocity(RadiansPerSecond.of(trajectory.getFlywheelSpeed()));
    }

    /**
     * Set the launcher to aim and, if possible, shoot, with the given trajectory.
     *
     * <p>Return a boolean of whether the launcher is feeding (and shooting).
     */
    public boolean launchTrajectory(Trajectory trajectory) {
        aimTrajectory(trajectory);
        shootTrajectory(trajectory);
        return feedTrajectory(trajectory);
    }

    /** Calculate the trajectory for the given target and parameters. Trajectory may be invalid. */
    public Trajectory calculateTargetTrajectory(
            LaunchTarget target, Pose3d robotPose, Translation2d robotVelocity) {
        TrajectoryParameters parameters =
                new TrajectoryParameters(
                        robotPose,
                        robotVelocity,
                        LauncherConst.ROBOT_TO_LAUNCHER_TRANSFORM,
                        target.translation(),
                        trajectorySpeedMultiplier);
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

    /** Set the launcher to launch at locked turret and hood angles. */
    public void launchFixed() {
        final boolean feed = false;
        AngularVelocity shooterFlywheelSpeed =
                RadiansPerSecond.of(
                        LaunchSpeed.estimateFlywheelSpeed(
                                fixedLaunchSpeed.in(MetersPerSecond),
                                fixedLaunchPitch.in(Radians)));
        shooter.moveAngularVelocity(shooterFlywheelSpeed);
        if (feed) {
            feeder.start();
        }
        aimDirection(fixedLaunchPitch, fixedLaunchYaw);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        SmartDashboard.putData("Launcher/shooter", shooter);
        SmartDashboard.putData("Launcher/feeder", feeder);
        SmartDashboard.putData("Launcher/hood", hood);
        SmartDashboard.putData("Launcher/turret", turret);
        builder.addDoubleProperty(
                "trajectory/speed multiplier",
                () -> trajectorySpeedMultiplier,
                (multiplier) -> {
                    trajectorySpeedMultiplier = multiplier;
                });
        builder.addDoubleProperty(
                "trajectory/yaw offset (deg)",
                () -> trajectoryYawOffset.in(Degrees),
                (offset) -> {
                    trajectoryYawOffset = Degrees.of(offset);
                });
        builder.addDoubleProperty(
                "fixed launch/speed (m per s)",
                () -> fixedLaunchSpeed.in(MetersPerSecond),
                (speed) -> {
                    fixedLaunchSpeed = MetersPerSecond.of(speed);
                });
        builder.addDoubleProperty(
                "fixed launch/pitch (deg)",
                () -> fixedLaunchPitch.in(Degrees),
                (pitch) -> {
                    fixedLaunchPitch = Degrees.of(pitch);
                });
        builder.addDoubleProperty(
                "fixed launch/yaw (deg)",
                () -> fixedLaunchYaw.in(Degrees),
                (yaw) -> {
                    fixedLaunchYaw = Degrees.of(yaw);
                });
    }
}
