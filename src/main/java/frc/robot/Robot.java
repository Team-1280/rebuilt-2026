// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.drivetrain.CommandSwerveDrivetrain;
import frc.robot.drivetrain.OdometryDrivetrain;
import frc.robot.vision.VisionSubsystem;

public class Robot extends TimedRobot implements Sendable {
    private final Pigeon2 pigeon = new Pigeon2(26); // Also in TunerConstants.kPigeonId

    private final CommandSwerveDrivetrain drivetrain =
            new OdometryDrivetrain(
                    () -> pigeon.getAngularVelocityZDevice().getValue().in(RadiansPerSecond),
                    () -> 0.0 // TODO: slip ratio supplier
                    );
    private final VisionSubsystem vision = new VisionSubsystem(drivetrain::addVisionMeasurement);

    private final CommandXboxController controller = new CommandXboxController(0); // TODO

    private final Field2d field = new Field2d();

    public Robot() {
        initDashboard();
        initBindings();
    }

    private void initDashboard() {
        SmartDashboard.putData("Robot", this);
        SmartDashboard.putData("Field", field);
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
