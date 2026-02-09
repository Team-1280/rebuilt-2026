package frc.robot.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

public final class OdometryDrivetrain extends CommandSwerveDrivetrain {

                private static final double ODOMETRY_UPDATE_FREQUENCY = CommandSwerveIO.ODOMETRY_FREQ;// hz if the robot
                                                                                                      // is using a
                                                                                                      // CANFD or CAN2.0

                // vision standard deviations in the form [x, y, theta], for use in the
                // drivetrain constructor
                private static final Matrix<N3, N1> VISION_STD_BEST = VecBuilder.fill(0.05, 0.05, 0.04);
                private static final Matrix<N3, N1> VISION_STD_WORST = VecBuilder.fill(0.60, 0.60, 0.40);

                private static final double GYRO_AGREEMENT_SIGMA = 1.5; // rad/s
                private static final double ODOMETRY_OMEGA_SIGMA = 2.0; // rad/s

                private static final double TRUST_VISION_RANGE_MIN = 0.25;
                private static final double TRUST_VISION_RANGE_MAX = 3.5;

                private final DoubleSupplier pigeonRateSupplier;
                private final DoubleSupplier rioRateSupplier;

                private Pose2d lastPose = Pose2d.kZero;
                private double lastTimeSec = Timer.getFPGATimestamp();

                public OdometryDrivetrain(
                                                DoubleSupplier pigeonRateSupplier,
                                                DoubleSupplier rioRateSupplier) {
                                super(
                                                                TunerConstants.DrivetrainConstants,
                                                                ODOMETRY_UPDATE_FREQUENCY,
                                                                TunerConstants.FrontLeft,
                                                                TunerConstants.FrontRight,
                                                                TunerConstants.BackLeft,
                                                                TunerConstants.BackRight);

                                this.pigeonRateSupplier = pigeonRateSupplier;
                                this.rioRateSupplier = rioRateSupplier;
                }

                private static double gaussianTrust(double error, double sigma) {
                                return Math.exp(-Math.abs(error) / sigma);
                }

                private static Matrix<N3, N1> interpolateMatrices(
                                                Matrix<N3, N1> worst,
                                                Matrix<N3, N1> best,
                                                double alpha) {
                                return VecBuilder.fill(
                                                                MathUtil.interpolate(worst.get(0, 0), best.get(0, 0),
                                                                                                alpha),
                                                                MathUtil.interpolate(worst.get(1, 0), best.get(1, 0),
                                                                                                alpha),
                                                                MathUtil.interpolate(worst.get(2, 0), best.get(2, 0),
                                                                                                alpha));
                }

                @Override
                public void periodic() {
                                super.periodic();

                                double now = Timer.getFPGATimestamp();
                                double dt = now - lastTimeSec;
                                lastTimeSec = now;

                                if (dt <= 1e-4)
                                                return;

                                Pose2d currentPose = getState().Pose;

                                double omegaPigeon = pigeonRateSupplier.getAsDouble();
                                double omegaRio = rioRateSupplier.getAsDouble();

                                double gyroAgreementTrust = gaussianTrust(omegaPigeon - omegaRio, GYRO_AGREEMENT_SIGMA);

                                double omegaInertial = gyroAgreementTrust * omegaPigeon
                                                                + (1.0 - gyroAgreementTrust) * omegaRio;

                                double odomDeltaTheta = MathUtil.angleModulus(
                                                                currentPose.getRotation().getRadians()
                                                                                                - lastPose.getRotation().getRadians());
                                double omegaOdometry = odomDeltaTheta / dt;

                                lastPose = currentPose;

                                double odometryTrust = gaussianTrust(
                                                                omegaOdometry - omegaInertial,
                                                                ODOMETRY_OMEGA_SIGMA);
                                Logger.recordOutput("Odo/Trust", odometryTrust);
                                cachedOdometryTrust = odometryTrust;
                }

                private double cachedOdometryTrust = 1.0;

                public void addVisionMeasurement(
                                                Pose2d pose,
                                                double timestampSeconds,
                                                double distanceMeters,
                                                boolean noisy) {
                                if (noisy
                                                                || distanceMeters < TRUST_VISION_RANGE_MIN
                                                                || distanceMeters > TRUST_VISION_RANGE_MAX) {
                                                return;
                                }

                                Matrix<N3, N1> visionStd = interpolateMatrices(
                                                                VISION_STD_WORST,
                                                                VISION_STD_BEST,
                                                                1.0 - cachedOdometryTrust);

                                addVisionMeasurement(pose, timestampSeconds, visionStd);
                }
}
