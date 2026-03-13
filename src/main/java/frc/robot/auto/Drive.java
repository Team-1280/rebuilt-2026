package frc.robot.auto;

import choreo.trajectory.SwerveSample;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.drivetrain.OdometryDrivetrain;

public class Drive extends SubsystemBase {
    private final PIDController xController = new PIDController(10.0, 0.0, 0.0);
    private final PIDController yController = new PIDController(10.0, 0.0, 0.0);
    private final PIDController headingController = new PIDController(7.5, 0.0, 0.0);

    public Drive() {
        // Other subsystem initialization code
        // ...

        headingController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public void followTrajectory(SwerveSample sample) {
        // Get the current pose of the robot
        Pose2d pose =
                OdometryDrivetrain.getState()
                        .Pose; // TODO: use actual working getPose function (also use Pose3d??)

        // Generate the next speeds for the robot
        ChassisSpeeds speeds =
                new ChassisSpeeds(
                        sample.vx + xController.calculate(pose.getX(), sample.x),
                        sample.vy + yController.calculate(pose.getY(), sample.y),
                        sample.omega
                                + headingController.calculate(
                                        pose.getRotation().getRadians(), sample.heading));

        // Apply the generated speeds
        driveFieldRelative(speeds); // TODO: learn what choreo wants me to do
    }
}
