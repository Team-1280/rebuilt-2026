// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Value;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

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

    private final CommandXboxController driverController =
            new CommandXboxController(DriveConfig.DRIVER_CONTROLLER_PORT);
    private final CommandXboxController operatorController =
            new CommandXboxController(DriveConfig.OPERATOR_CONTROLLER_PORT);

    private final Field2d field = new Field2d();
    private final StructPublisher<Pose2d> posePublisher =
            NetworkTableInstance.getDefault()
                    .getStructTopic("Robot Pose2d", Pose2d.struct)
                    .publish();
    private final StructPublisher<Pose3d> pose3dPublisher =
            NetworkTableInstance.getDefault()
                    .getStructTopic("Robot Pose3d", Pose3d.struct)
                    .publish();

    private final SendableChooser<Command> autoChooser;

    public Robot() {
        initLogger(); // must happen first
        initAuto(); // must happen before auto chooser built
        autoChooser = AutoBuilder.buildAutoChooser();
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
        SmartDashboard.putData("Auto Chooser", autoChooser);
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
                                        -driverController.getLeftY(),
                                        -driverController.getLeftX(),
                                        -driverController.getRightX())));

        // constant drive hold
        driverController
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
        driverController
                .back()
                .or(operatorController.back())
                .onTrue(
                        Commands.runOnce(() -> drivetrain.resetPose(DriveConfig.getResetPose()))
                                .ignoringDisable(true));

        // reset heading press
        driverController
                .rightStick()
                .or(operatorController.rightStick())
                .onTrue(
                        drivetrain
                                .runOnce(
                                        () ->
                                                drivetrain.resetRotation(
                                                        DriverStation.getAlliance()
                                                                                .orElse(
                                                                                        Alliance
                                                                                                .Blue)
                                                                        == Alliance.Blue
                                                                ? Rotation2d.kZero
                                                                : Rotation2d.fromDegrees(180.0)))
                                .ignoringDisable(true));

        // stow robot press; until any subsystem activated
        driverController
                .start()
                .or(operatorController.start())
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
        driverController
                .povDown()
                .or(operatorController.povDown())
                .onTrue(intake.runOnce(intake::deploy));

        // intake stow (up+stop) press
        driverController
                .povUp()
                .or(operatorController.povUp())
                .onTrue(intake.runOnce(intake::stow));

        // intake down+stop press
        driverController
                .povRight()
                .or(operatorController.povRight())
                .onTrue(
                        intake.runOnce(
                                () -> {
                                    intake.moveDown();
                                    intake.rollersOff();
                                }));

        // reverse intake rollers hold
        driverController
                .povLeft()
                .or(operatorController.povLeft())
                .whileTrue(intake.startEnd(intake::rollersReverse, intake::rollersOn));

        // shooting hold
        driverController
                .rightTrigger()
                .or(operatorController.rightTrigger())
                .whileTrue(runShooting());

        // fixed launching hold
        driverController
                .rightBumper()
                .or(operatorController.rightBumper())
                .whileTrue(Commands.run(launcher::launchFixed, launcher.hood, launcher.turret))
                .whileTrue(runShooting());

        // stow launcher hold
        driverController
                .leftTrigger()
                .or(operatorController.leftTrigger())
                .whileTrue(
                        Commands.run(launcher::stow, launcher.subsystems)
                                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
                                .ignoringDisable(true));

        // intake fuel unjam hold
        driverController.a().or(operatorController.a()).whileTrue(runUnjamIntakeFuel());

        // hopper/launcher fuel unjam hold
        driverController.b().or(operatorController.b()).whileTrue(runUnjamHopperLauncherFuel());

        // spindexer: on by default
        spindexer.setDefaultCommand(spindexer.run(spindexer::start));

        // shooter: off by default
        launcher.shooter.disable();
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

    private Command runShooting() {
        return Commands.startEnd(
                () -> {
                    launcher.feeder.start();
                    launcher.shooter.enable();
                },
                () -> {
                    launcher.feeder.stop();
                    launcher.shooter.disable();
                },
                launcher.feeder);
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

    /**
     * Utility method that gets a wrapper instant command that schedules the given command and
     * immediately finishes.
     */
    private Command instantProxy(Command command) {
        return Commands.runOnce(() -> CommandScheduler.getInstance().schedule(command));
    }

    public void initAuto() {
        // Note: all named commands must finish instantly

        NamedCommands.registerCommand("deployIntake", intake.runOnce(intake::deploy));
        NamedCommands.registerCommand("stowIntake", intake.runOnce(intake::stow));

        final Command runStowLauncher =
                Commands.run(launcher::stow, launcher.subsystems)
                        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
        NamedCommands.registerCommand("stowLauncher", instantProxy(runStowLauncher));
        NamedCommands.registerCommand("unstowLauncher", Commands.runOnce(runStowLauncher::cancel));

        final Command runShooting = runShooting();
        NamedCommands.registerCommand("startShooting", instantProxy(runShooting));
        NamedCommands.registerCommand("stopShooting", Commands.runOnce(runShooting::cancel));

        RobotModeTriggers.autonomous()
                .onFalse(
                        Commands.runOnce(
                                () -> {
                                    runStowLauncher.cancel();
                                    runShooting.cancel();
                                }));
    }

    @Override
    public void robotInit() {}

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
        stow();
    }

    @Override
    public void autonomousInit() {
        CommandScheduler.getInstance().schedule(Commands.idle(launcher.subsystems));
        Command selectedAuto = autoChooser.getSelected();
        if (selectedAuto != null) {
            CommandScheduler.getInstance().schedule(selectedAuto);
            SmartDashboard.putData("Auto Chooser/" + selectedAuto.getName(), selectedAuto);
            RobotModeTriggers.autonomous().onFalse(Commands.runOnce(selectedAuto::cancel));
        }
    }

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
