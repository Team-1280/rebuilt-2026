// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.drivetrain.CommandSwerveDrivetrain;
import frc.robot.drivetrain.TunerConstants;

public class Robot extends TimedRobot {

    private final CommandSwerveDrivetrain drivetrain =
        TunerConstants.createDrivetrain();

    private final CommandXboxController controller = new CommandXboxController(
        0
    ); // TODO

    private final Field2d field = new Field2d();

    public Robot() {
        initDashboard();
        initBindings();
    }

    private void initDashboard() {
        SmartDashboard.putData("Field", field);
    }

    private void initBindings() {
        // Drive bindings
        double speed =
            TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.15;
        double angularSpeed = RotationsPerSecond.of(1.0).in(RadiansPerSecond);
        final SwerveRequest.FieldCentric driveRequest =
            new SwerveRequest.FieldCentric()
                .withDeadband(speed * 0.1)
                .withRotationalDeadband(angularSpeed * 0.1)
                .withDriveRequestType(DriveRequestType.Velocity);
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                driveRequest
                    .withVelocityX(controller.getLeftY() * speed)
                    .withVelocityY(controller.getLeftX() * speed)
                    .withRotationalRate(controller.getRightX() * angularSpeed)
            )
        );
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
}
