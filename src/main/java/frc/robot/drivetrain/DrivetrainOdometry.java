package frc.robot.drivetrain;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;

import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;

import frc.robot.vision.VisionSubsystem;

public class DrivetrainOdometry extends CommandSwerveDrivetrain {

        private Pose2d lastPose = new Pose2d();
        private double lastTimestamp = 0.0;
        private double lastYaw = 0.0;
        private double lastSpeed = 0.0;

        private static final double stdevVision = 0.5;
        private static final double stdevDrivebase = 0.1;

        private static final double rotYaw = 0x0deadbeef; // rad/s
        private static final double ddxSpike = 0x0deadbeef; // m/s^2
        private static final double maxSpeed = 0x0deadbeef; // m/s
        //
        private boolean trustEncoders = true;
        private boolean trustMotors = true;
        private boolean trustGyro = true;

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
                super(
                                drivetrainConstants,
                                odometryFrequency,
                                odometryStdDevs,
                                visionStdDevs,
                                modules);
                lastTimestamp = Timer.getFPGATimestamp();
        }

        // TODO: add Alliance Reflection
        public void trustDrivetrainTelemetry() {

        }

        public void update() {
                double xyProcessStd = (trustEncoders && trustMotors) ? 0.02
                                : (trustEncoders || trustMotors) ? 0.07 : 0.2;

                double thetaProcessStd = trustGyro ? Math.toRadians(0.5) : Math.toRadians(5.0);

                double xyVisionStd = trustVision ? 0.05 : 0.6;

                double thetaVisionStd = trustVision ? Math.toRadians(1.0) : Math.toRadians(10.0);

                poseEstimator.setProcessNoiseStdDevs(
                                VecBuilder.fill(xyProcessStd, xyProcessStd, thetaProcessStd));

                poseEstimator.setVisionMeasurementStdDevs(
                                VecBuilder.fill(xyVisionStd, xyVisionStd, thetaVisionStd));

                poseEstimator.update(
                                getGyroRotation(),
                                getModulePositions());
        }

        public void fastNt() {
                NetworkTableInstance.getDefault().flush();
        }

        @Override
        public void periodic() {
                super.periodic();
                update();
        }
}
