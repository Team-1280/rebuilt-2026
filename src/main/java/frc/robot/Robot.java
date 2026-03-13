// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import choreo.auto.AutoFactory;
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
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.aesthetic.candle.CandleSubsystem;
import frc.robot.drivetrain.OdometryDrivetrain;
import frc.robot.field.FieldZoning;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.spindexer.SpindexerSubsystem;
import frc.robot.time.HubStatus;
import frc.robot.time.MatchTime;
import frc.robot.vision.VisionSubsystem;

public class Robot extends LoggedRobot implements Sendable {

    private final Pigeon2 pigeon = new Pigeon2(26); // Also in TunerConstants.kPigeonId

    private final OdometryDrivetrain drivetrain = new OdometryDrivetrain();
    private final CandleSubsystem candle = new CandleSubsystem();
    private final VisionSubsystem vision = new VisionSubsystem(drivetrain::addVisionMeasurement);
    private final SpindexerSubsystem spindexer = new SpindexerSubsystem();
    private final IntakeSubsystem intake = new IntakeSubsystem();
    
    private final SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser("test");

    private final CommandXboxController controller = new CommandXboxController(0); // TODO

    private final Field2d field = new Field2d();
    StructPublisher<Pose2d> posePublisher =
            NetworkTableInstance.getDefault().getStructTopic("Robot Pose", Pose2d.struct).publish();

    // Choreo setup:
    private final AutoFactory autoFactory;
    private Command autonomousCommand;

    public Robot() {
        initLogger(); // must happen first
        initDashboard();
        initBindings();

        autoFactory = new AutoFactory(
            drivetrain::getPose, // A function that returns the current robot pose
            drivetrain::resetOdometry, // A function that resets the current robot pose to the provided Pose2d
            drivetrain::followTrajectory, // The drive subsystem trajectory follower 
            true, // If alliance flipping should be enabled 
            drivetrain, // The drive subsystem
        );
        /* 
        autoFactory
            .bind("deployIntake", intakeSubsystem.intake())
            .bind("retractIntake", intakeSubsystem.intake())
            .bind("startIntake", scoringSubsystem.score())
            .bind("stopIntake", scoringSubsystem.score())
            .bind("startShootFuel", scoringSubsystem.score());
            */
    }

    private void initLogger() {
        Logger.recordMetadata("Heatseeker", "Haru Urara"); // Set a metadata value
        //Logger.recordMetadata("gitSHA", BuildConstants.GIT_SHA);
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
        SmartDashboard.putData("Field Zoning", FieldZoning.getSendable(drivetrain::getPose2d));
        SmartDashboard.putData("Match Time", MatchTime.getSendable());
        SmartDashboard.putData("Hub Status", HubStatus.getSendable());
        SmartDashboard.putData("Drivetrain", drivetrain);
        SmartDashboard.putData("Vision", vision);
        SmartDashboard.putData("Spindexer", spindexer);
        SmartDashboard.putData("Intake", intake);
    }

    private void initBindings() {
        // swerve drive
        final SwerveRequest.FieldCentric driveRequest =
                new SwerveRequest.FieldCentric()
                        .withDeadband(DriveConfig.speedDeadband)
                        .withRotationalDeadband(DriveConfig.angularSpeedDeadband)
                        .withDriveRequestType(DriveRequestType.Velocity);
        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(
                        () ->
                                !DriveConfig.enableDriving
                                        ? null
                                        : driveRequest
                                                .withVelocityX(
                                                        DriveConfig.maxSpeed.times(
                                                                -controller.getLeftY()))
                                                .withVelocityY(
                                                        DriveConfig.maxSpeed.times(
                                                                -controller.getLeftX()))
                                                .withRotationalRate(
                                                        DriveConfig.maxAngularSpeed.times(
                                                                -controller.getRightX()))));

        // reset heading
        controller
                .rightStick()
                .onTrue(drivetrain.runOnce(() -> drivetrain.resetRotation(Rotation2d.kZero)));
    }

    @Override
    public void robotInit() {
        autoChooser.addOption("None", Commands.none());
        autoChooser.addOption("LeftSideStart", LeftSideStartAuto());
        autoChooser.addOption("MidSideStart", MidSideStartAuto());
        autoChooser.addOption("RightSideStart", RightSideStartAuto());
        autoChooser.addOption("SimpleForwardAuto", SimpleForwardAuto());
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        field.setRobotPose(drivetrain.getPose2d());
        posePublisher.set(drivetrain.getPose2d());
        pose3dPublisher.set(drivetrain.getPose3d());
    }

    @Override
    public void autonomousInit() {
        autonomousCommand = autoChooser.getSelected();

        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {}

    private boolean isRedAlliance() {
        return DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red);
    }

    @Override
    public void teleopInit() {
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

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

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    @Override
    public void initSendable(SendableBuilder builder) {}

    
    private Command leftSideStartAuto() {
        return autoFactory.trajectoryCmd("LeftSideStartAuto");
    }
    private Command midSideStartAuto() {
        return autoFactory.trajectoryCmd("MidSideStartAuto");
    }
    private Command rightSideStartAuto() {
        return autoFactory.trajectoryCmd("RightSideStartAuto");
    }  
    private Command simpleForwardAuto() {
        return autoFactory.trajectoryCmd("SimpleForwardAuto");
    }
}
