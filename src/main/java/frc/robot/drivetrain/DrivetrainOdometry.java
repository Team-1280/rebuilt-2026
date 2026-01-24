package frc.robot.drivetrain;

import edu.wpi.first.apriltag.AprilTagPoseEstimate;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;

import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

public class DrivetrainOdometry extends CommandSwerveDrivetrain {

    private Pose2d lastPose = new Pose2d();
    private double lastTimestamp = 0.0;

    public DrivetrainOdometry(
            SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, modules);
        lastTimestamp = Timer.getFPGATimestamp();
    }

    public DrivetrainOdometry(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryFrequency,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryFrequency, modules);
        lastTimestamp = Timer.getFPGATimestamp();
    }

    public DrivetrainOdometry(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryFrequency,
            Matrix<N3, N1> odometryStdDevs,
            Matrix<N3, N1> visionStdDevs,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryFrequency, odometryStdDevs, visionStdDevs, modules);
        lastTimestamp = Timer.getFPGATimestamp();
    }

    // Returns the Pose2d of the robot's location
    public Pose2d getEstimatedPose() {
        return getState().Pose;
    }

    // Returns a Rotation2d location of where the HEAD of robot is to it's "zeroed"
    // angle
    public Rotation2d getEstimatedHeading() {
        return getState().Pose.getRotation();
    }

    public void resetOdometry(Pose2d pose) {
        seedFieldCentric(getEstimatedHeading());
        lastPose = pose;
        
    }

    public void resetHeading(Rotation2d heading) {
        Pose2d current = getEstimatedPose();
        resetOdometry(new Pose2d(current.getTranslation(), heading));
    }

    public void addVisionPose(Pose2d visionPose, double timestampSeconds) {
        addVisionMeasurement(visionPose, timestampSeconds);
    }

    public double getPoseUpdatePeriod() {
        return Timer.getFPGATimestamp() - lastTimestamp;
    }

    public void updateCache() {
        lastPose = getEstimatedPose();
        lastTimestamp = Timer.getFPGATimestamp();
    }

    @Override
    public void periodic() {
        super.periodic();
        updateCache();
    }
}
