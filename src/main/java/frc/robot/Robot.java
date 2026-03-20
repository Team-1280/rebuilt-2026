// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Value;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.aesthetic.candle.CandleEffect;
import frc.robot.aesthetic.candle.CandleSubsystem;
import frc.robot.build.BuildConstants; // generated file: build to resolve
import frc.robot.drivetrain.OdometryDrivetrain;
import frc.robot.field.FieldZoning;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.launcher.LauncherAssembly;
import frc.robot.spindexer.SpindexerSubsystem;
import frc.robot.target.LaunchTarget;
import frc.robot.target.TargetSelector;
import frc.robot.time.HubStatus;
import frc.robot.time.MatchTime;
import frc.robot.trajectory.Trajectory;
import frc.robot.vision.VisionSubsystem;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import java.util.Optional;
import java.util.Set;

public class Robot extends LoggedRobot implements Sendable {

    private final Pigeon2 pigeon = new Pigeon2(26); // Also in TunerConstants.kPigeonId

    private final OdometryDrivetrain drivetrain = new OdometryDrivetrain();
    private final CandleSubsystem candle = new CandleSubsystem();
    private final VisionSubsystem vision = new VisionSubsystem(drivetrain::addVisionMeasurement);
    private final LauncherAssembly launcher = new LauncherAssembly();
    private final SpindexerSubsystem spindexer = new SpindexerSubsystem();
    private final IntakeSubsystem intake = new IntakeSubsystem();

    private final CommandXboxController controller = new CommandXboxController(0); // TODO

    private final Field2d field = new Field2d();
    private final StructPublisher<Pose2d> posePublisher =
            NetworkTableInstance.getDefault()
                    .getStructTopic("Robot Pose2d", Pose2d.struct)
                    .publish();
    private final StructPublisher<Pose3d> pose3dPublisher =
            NetworkTableInstance.getDefault()
                    .getStructTopic("Robot Pose3d", Pose3d.struct)
                    .publish();

    public Robot() {
        initLogger(); // must happen first
        initDashboard();
        initBindings();
    }

    private void initLogger() {
        Logger.recordMetadata("gitSHA", BuildConstants.GIT_SHA);
        if (isReal()) {
            Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
            Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
        } else {
            // TODO: add better way to choose if to replay
            // setUseTiming(false); // Run as fast as possible
            // // Pull the replay log from AdvantageScope (or prompt the user)
            // String logPath = LogFileUtil.findReplayLog();
            // // Read replay log
            // Logger.setReplaySource(new WPILOGReader(logPath));
            // // Save outputs to a new log
            // Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        }
        // Start logging! No more data receivers, replay sources, or metadata values may
        // be added.
        Logger.start();
    }

    private void initDashboard() {
        SmartDashboard.putData("Robot", this);
        SmartDashboard.putData("Drive Config", DriveConfig.getSendable());
        SmartDashboard.putData("Field", field);
        posePublisher.set(Pose2d.kZero);
        pose3dPublisher.set(Pose3d.kZero);
        SmartDashboard.putData(
                "Field Zoning",
                FieldZoning.getSendable(
                        drivetrain::getPose2d, () -> DriveConfig.trenchLauncherStowDistance));
        SmartDashboard.putData("Match Time", MatchTime.getSendable());
        SmartDashboard.putData("Hub Status", HubStatus.getSendable());
        SmartDashboard.putData("Target Selector", TargetSelector.getSendable());
        SmartDashboard.putData("Drivetrain", drivetrain);
        SmartDashboard.putData("Vision", vision);
        SmartDashboard.putData("Launcher", launcher);
        SmartDashboard.putData("Spindexer", spindexer);
        SmartDashboard.putData("Intake", intake);
    }

    private void initBindings() {
        // swerve drive joystick
        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(
                        () ->
                                getSwerveRequest(
                                        -controller.getLeftX(),
                                        -controller.getLeftY(),
                                        -controller.getRightX())));

        // constant drive hold
        controller
                .leftBumper()
                .whileTrue(
                        Commands.runEnd(
                                () -> {
                                    DriveConfig.constantDriveEnabled = true;
                                },
                                () -> {
                                    DriveConfig.constantDriveEnabled = false;
                                }));

        // reset pose press
        controller
                .back()
                .onTrue(
                        Commands.runOnce(() -> drivetrain.resetPose(DriveConfig.getResetPose()))
                                .ignoringDisable(true));

        // reset heading press
        controller
                .rightStick()
                .onTrue(
                        drivetrain
                                .runOnce(() -> drivetrain.resetRotation(Rotation2d.kZero))
                                .ignoringDisable(true));

        // stow robot press; until any subsystem activated
        controller
                .start()
                .onTrue(
                        Commands.run(
                                        this::stow,
                                        launcher.shooter,
                                        launcher.feeder,
                                        launcher.hood,
                                        launcher.turret,
                                        spindexer,
                                        intake)
                                .ignoringDisable(true));

        // intake deploy (down+start) press
        controller.povDown().onTrue(intake.runOnce(intake::deploy));

        // intake stow (up+stop) press
        controller.povUp().onTrue(intake.runOnce(intake::stow));

        // intake down+stop press
        controller
                .povRight()
                .onTrue(
                        intake.runOnce(
                                () -> {
                                    intake.moveDown();
                                    intake.rollersOff();
                                }));

        // reverse intake rollers hold
        controller.povLeft().whileTrue(intake.startEnd(intake::rollersReverse, intake::rollersOn));

        // feeding hold
        controller
                .rightTrigger()
                .whileTrue(launcher.feeder.startEnd(launcher.feeder::start, launcher.feeder::stop));

        // stow launcher hold
        controller
                .leftTrigger()
                .whileTrue(
                        Commands.run(launcher::stow, launcher.subsystems)
                                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
                                .ignoringDisable(true));

        // fixed launching hold
        controller
                .leftBumper()
                .whileTrue(
                        Commands.run(launcher::launchFixed, launcher.subsystems)
                                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

        // intake fuel unjam hold
        controller.a().whileTrue(runUnjamIntakeFuel());

        // hopper/launcher fuel unjam hold
        controller.b().whileTrue(runUnjamHopperLauncherFuel());

        // spindexer: on by default
        spindexer.setDefaultCommand(spindexer.run(spindexer::start));
    }

    /** Get the driving swerve request, from throttle values in WPILib robot coordinate axes. */
    private SwerveRequest getSwerveRequest(double xThrottle, double yThrottle, double rotThrottle) {
        if (!DriveConfig.enableDriving) {
            return new SwerveRequest.Idle();
        }
        if (DriveConfig.constantDriveEnabled) {
            double throttleMagnitude = Math.hypot(xThrottle, yThrottle);
            double multiplier =
                    throttleMagnitude > DriveConfig.constantDriveThrottleDeadband
                            ? DriveConfig.constantDriveSpeed.div(DriveConfig.maxSpeed).in(Value)
                                    / throttleMagnitude
                            : 0.0;
            xThrottle *= multiplier;
            yThrottle *= multiplier;
        }
        return DriveConfig.swerveRequest
                .withVelocityX(DriveConfig.maxSpeed.times(xThrottle))
                .withVelocityY(DriveConfig.maxSpeed.times(yThrottle))
                .withRotationalRate(DriveConfig.maxAngularSpeed.times(rotThrottle));
    }

    /** Stow or stop all subsystems except drivetrain. */
    private void stow() {
        launcher.stow();
        spindexer.stop();
        intake.stow();
    }

    private Command runUnjamIntakeFuel() {
        return intake.startEnd(
                () -> {
                    intake.moveUp();
                    intake.rollersReverse();
                },
                intake::deploy);
    }

    private Command runUnjamHopperLauncherFuel() {
        return Commands.startEnd(
                () -> {
                    launcher.feeder.reverse();
                    spindexer.reverse();
                },
                () -> {
                    launcher.feeder.stop();
                    spindexer.stop();
                },
                launcher.feeder,
                spindexer);
    }

    /** Get a command that aims the launcher at the appropriate target. */
    private Command runAutomaticTargeting() {
        // is split into multiple commands to separate requirements
        return Commands.defer(
                () -> {
                    Pose3d robotPose = drivetrain.getPose3d();
                    Optional<LaunchTarget> target = TargetSelector.selectTarget(robotPose);
                    if (target.isEmpty()) {
                        return Commands.parallel(
                                asDefault(launcher.shooter.runOnce(launcher.shooter::stop)),
                                asDefault(launcher.hood.runOnce(launcher.hood::stow)),
                                asDefault(launcher.turret.runOnce(launcher.turret::stow)));
                    }
                    Trajectory trajectory =
                            launcher.calculateTargetTrajectory(
                                    target.get(), robotPose, drivetrain.getFieldVelocity());
                    return Commands.parallel(
                            asDefault(
                                    Commands.runOnce(
                                            () -> launcher.aimTrajectory(trajectory),
                                            launcher.hood,
                                            launcher.turret)),
                            asDefault(
                                    launcher.shooter.runOnce(
                                            () -> launcher.shootTrajectory(trajectory))));
                },
                Set.of());
    }

    /**
     * Utility method that decorates the command to only run if the command requirements aren't
     * occupied, and to not extend the command's requirements to the command composition.
     */
    private Command asDefault(Command command) {
        return command.asProxy()
                .onlyIf(
                        () ->
                                command.getRequirements().stream()
                                        .allMatch(
                                                req ->
                                                        CommandScheduler.getInstance()
                                                                        .requiring(req)
                                                                == null));
    }

    @Override
    public void robotInit() {
        candle.animateCandle(CandleEffect.CHROMA);
    }

    @Override
    public void robotPeriodic() {
        field.setRobotPose(drivetrain.getPose2d());
        posePublisher.set(drivetrain.getPose2d());
        pose3dPublisher.set(drivetrain.getPose3d());

        if (isEnabled()) {
            // Runs before controller triggers are checked, which deprioritizes automatic launching
            CommandScheduler.getInstance().schedule(runAutomaticTargeting());
        }
        CommandScheduler.getInstance().run();
        if (isEnabled()) {
            // Occurs after CommandScheduler and binding triggers in order to have priority
            // Automatically stow launcher when robot is near a trench according to odometry
            if (FieldZoning.isNearTrench(
                    drivetrain.getPose2d().getTranslation(),
                    DriveConfig.trenchLauncherStowDistance)) {
                launcher.stow();
                // Additionally, schedule a command to interrupt other launcher commands
                CommandScheduler.getInstance()
                        .schedule(
                                Commands.runOnce(launcher::stow, launcher.subsystems)
                                        .withInterruptBehavior(
                                                InterruptionBehavior.kCancelIncoming));
            }
        }
    }

    @Override
    public void autonomousInit() {}

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {}

    @Override
    public void teleopPeriodic() {}

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void testInit() {}

    @Override
    public void testPeriodic() {}

    @Override
    public void simulationInit() {}

    @Override
    public void simulationPeriodic() {}

    @Override
    public void initSendable(SendableBuilder builder) {}
}
