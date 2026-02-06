package frc.robot.drivetrain;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.vision.VisionSubsystem;

import java.util.function.DoubleSupplier;

public final class DrivetrainOdometry extends CommandSwerveDrivetrain {

        private static final double ODOMETRY_HZ = 250.0;

        private static final Matrix<N3, ?> VISION_STD_BEST = VecBuilder.fill(0.05, 0.05, 0.04);
        private static final Matrix<N3, ?> VISION_STD_WORST = VecBuilder.fill(0.60, 0.60, 0.40);

        private static final double TRUST_IDENTITY = 1.0;
        private static final double TRUST_GYRO_LIMIT = 6.0; // rad/s
        private static final double TRUST_SLIP_LIMIT = 0.35; // slip ratio threshold
        private static final double TRUST_VISION_RANGE_MIN = 0.25;
        private static final double TRUST_VISION_RANGE_MAX = 3.5;

        private final DoubleSupplier gyroRateRadPerSec;
        private final DoubleSupplier slipRatio;

        private Pose2d lastPose = new Pose2d();
        private double lastTimeSec = Timer.getFPGATimestamp();

        public DrivetrainOdometry(DoubleSupplier gyroRateRadPerSec, DoubleSupplier slipRatio) {
                super(
                                TunerConstants.DrivetrainConstants,
                                ODOMETRY_HZ,
                                TunerConstants.FrontLeft,
                                TunerConstants.FrontRight,
                                TunerConstants.BackLeft,
                                TunerConstants.BackRight);
                this.gyroRateRadPerSec = gyroRateRadPerSec;
                this.slipRatio = slipRatio;
        }

        private static double trust(boolean predicate) {
                return predicate ? 1.0 : 0.0;
        }

        private static double combine(double a, double b) {
                return a * b;
        }

        private static Matrix<N3, ?> interpolate(Matrix<N3, ?> best, Matrix<N3, ?> worst, double alpha) {
                return VecBuilder.fill(
                                best.get(0, 0) * alpha + worst.get(0, 0) * (1.0 - alpha),
                                best.get(1, 0) * alpha + worst.get(1, 0) * (1.0 - alpha),
                                best.get(2, 0) * alpha + worst.get(2, 0) * (1.0 - alpha));
        }

        @Override
        public void periodic() {
                double now = Timer.getFPGATimestamp();
                double dt = now - lastTimeSec;
                lastTimeSec = now;

                Pose2d pose = getState().Pose;
                double translationDelta = pose.getTranslation().getDistance(lastPose.getTranslation());
                lastPose = pose;

                double gyroTrust = trust(Math.abs(gyroRateRadPerSec.getAsDouble()) < TRUST_GYRO_LIMIT);
                double slipTrust = trust(slipRatio.getAsDouble() < TRUST_SLIP_LIMIT);

                double odomTrust = combine(TRUST_IDENTITY, combine(gyroTrust, slipTrust));
        }

        public void applyVisionMeasurement(
                        Pose2d pose,
                        double timestampSeconds,
                        double distanceMeters,
                        boolean noisy) {
                double rangeTrust = trust(
                                distanceMeters > TRUST_VISION_RANGE_MIN && distanceMeters < TRUST_VISION_RANGE_MAX);
                double ambiguityTrust = trust(!noisy);
                double visionTrust = combine(rangeTrust, ambiguityTrust);

                addVisionMeasurement(pose, Utils.fpgaToCurrentTime(timestampSeconds));
        }
}
