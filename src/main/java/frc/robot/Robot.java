// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.build.BuildConstants; // generated file: build to resolve
import frc.robot.drivetrain.CommandSwerveDrivetrain;
import frc.robot.drivetrain.OdometryDrivetrain;
import frc.robot.vision.VisionSubsystem;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot implements Sendable {

    private final CommandSwerveDrivetrain drivetrain = new OdometryDrivetrain();
    private final VisionSubsystem vision = new VisionSubsystem(drivetrain::addVisionMeasurement);

    private final CommandXboxController controller = new CommandXboxController(0); // TODO

    private final Field2d field = new Field2d();
    StructPublisher<Pose2d> posePublisher =
            NetworkTableInstance.getDefault().getStructTopic("Robot Pose", Pose2d.struct).publish();

    public Robot() {
        initLogger(); // must happen first
        initDashboard();
        initBindings();
    }

    private void initLogger() {
        Logger.recordMetadata("Heatseeker", "Haru Urara"); // Set a metadata value
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
    public void robotInit() {}

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        field.setRobotPose(drivetrain.getState().Pose);
        posePublisher.set(drivetrain.getState().Pose);
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
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("robot pose", () -> drivetrain.getState().Pose.toString(), null);
        builder.addStringProperty(
                "robot speeds", () -> drivetrain.getState().Speeds.toString(), null);
    }
}
