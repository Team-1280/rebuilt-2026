// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

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
import frc.robot.drivetrain.CommandSwerveDrivetrain;
import frc.robot.drivetrain.OdometryDrivetrain;
import frc.robot.field.FieldZoning;
import frc.robot.vision.VisionSubsystem;

public class Robot extends LoggedRobot implements Sendable {

    private final Pigeon2 pigeon = new Pigeon2(26); // Also in TunerConstants.kPigeonId

    private final CommandSwerveDrivetrain drivetrain =
            new OdometryDrivetrain(
                    () -> pigeon.getAngularVelocityZDevice().getValue().in(RadiansPerSecond),
                    () -> 0.0 // TODO: slip ratio supplier
                    );
    private final VisionSubsystem vision = new VisionSubsystem(drivetrain::addVisionMeasurement);
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
        SmartDashboard.putData("Field", field);
        posePublisher.set(Pose2d.kZero);
        SmartDashboard.putData("Vision", vision);
    }

    private void initBindings() {
        // Drive bindings
        double speed = MetersPerSecond.of(1.60).in(MetersPerSecond);
        double angularSpeed = RotationsPerSecond.of(0.5).in(RadiansPerSecond);
        final SwerveRequest.FieldCentric driveRequest =
                new SwerveRequest.FieldCentric()
                        .withDeadband(speed * 0.1)
                        .withRotationalDeadband(angularSpeed * 0.1)
                        .withDriveRequestType(DriveRequestType.Velocity);
        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(
                        () ->
                                driveRequest
                                        .withVelocityX(-controller.getLeftY() * speed)
                                        .withVelocityY(-controller.getLeftX() * speed)
                                        .withRotationalRate(
                                                -controller.getRightX() * angularSpeed)));

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
        field.setRobotPose(drivetrain.getState().Pose);
        posePublisher.set(drivetrain.getState().Pose);
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
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("robot pose", () -> drivetrain.getState().Pose.toString(), null);
        builder.addStringProperty(
                "robot speeds", () -> drivetrain.getState().Speeds.toString(), null);

        // Field zoning
        builder.addStringProperty(
                "zoning/zone",
                () -> {
                    Pose2d pose = drivetrain.getState().Pose;
                    if (FieldZoning.isInRedAllianceZone(pose)) return "Red";
                    if (FieldZoning.isInBlueAllianceZone(pose)) return "Blue";
                    return "Neutral";
                },
                null);
        builder.addBooleanProperty(
                "zoning/team alliance zone",
                () -> FieldZoning.isInTeamAllianceZone(drivetrain.getState().Pose),
                null);
        builder.addBooleanProperty(
                "zoning/red alliance zone",
                () -> FieldZoning.isInRedAllianceZone(drivetrain.getState().Pose),
                null);
        builder.addBooleanProperty(
                "zoning/blue alliance zone",
                () -> FieldZoning.isInBlueAllianceZone(drivetrain.getState().Pose),
                null);
        builder.addBooleanProperty(
                "zoning/neutral zone",
                () -> FieldZoning.isInNeutralZone(drivetrain.getState().Pose),
                null);
        builder.addBooleanProperty(
                "zoning/on bump", () -> FieldZoning.isOnBump(drivetrain.getState().Pose), null);
    }

    private Command LeftSideStartAuto() {
        return autoFactory.trajectoryCmd("LeftSideStartAuto");
    }
    private Command MidSideStartAuto() {
        return autoFactory.trajectoryCmd("MidSideStartAuto");
    }
    private Command RightSideStartAuto() {
        return autoFactory.trajectoryCmd("RightSideStartAuto");
    }  
    private Command SimpleForwardAuto() {
        return autoFactory.trajectoryCmd("SimpleForwardAuto");
    }
}
