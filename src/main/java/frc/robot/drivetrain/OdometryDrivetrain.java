package frc.robot.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * OdometryDrivetrain extends the base swerve drivetrain with logic that
 * dynamically determines how
 * much the robot should trust odometry vs vision.
 *
 * <p>
 * This class solves a specific mechanical problem: when swerve modules lift off
 * the ground
 * (e.g., over bumps), wheel encoders report false motion ("slip"). This
 * implementation detects slip
 * by comparing rotation rates from:
 * <ul>
 * <li>Gyroscopes (inertial, reliable during lift)</li>
 * <li>Encoder-based odometry (unreliable during lift)</li>
 * </ul>
 * and dynamically adjusts trust in vision measurements accordingly.
 *
 * <p>
 * This class is designed to be readable by rookies:
 * <ul>
 * <li>All math is explained at a high level</li>
 * <li>No assumptions about probability or estimation theory</li>
 * <li>Comments explain "why", not just "what"</li>
 * </ul>
 */
public final class OdometryDrivetrain extends CommandSwerveDrivetrain {

        /**
         * How often odometry is updated, in Hz.
         *
         * <p>
         * This should match the CAN update rate of the drivetrain hardware.
         */
        private static final double ODOMETRY_UPDATE_FREQUENCY = CommandSwerveIO.ODOMETRY_FREQ;

        /**
         * Vision standard deviations when vision is performing very well.
         *
         * <p>
         * Lower values mean "trust vision more".
         *
         * <p>
         * Order is: [x meters, y meters, rotation radians]
         */
        private static final Matrix<N3, N1> VISION_STD_BEST = VecBuilder.fill(0.05, 0.05, 0.04);

        /**
         * Vision standard deviations when vision is performing very poorly.
         *
         * <p>
         * Higher values mean "trust vision less".
         */
        private static final Matrix<N3, N1> VISION_STD_WORST = VecBuilder.fill(0.60, 0.60, 0.40);

        /**
         * Acceptable disagreement between two gyro angular velocity measurements before
         * trust is
         * reduced.
         *
         * <p>
         * Units are radians per second.
         */
        private static final double GYRO_AGREEMENT_SIGMA = 1.5;

        /**
         * Acceptable disagreement between odometry-derived rotation rate and gyro
         * rotation rate.
         *
         * <p>
         * When wheels lift off ground, encoder odometry reports false rotation. This
         * threshold
         * detects that slip condition.
         *
         * <p>
         * Units are radians per second.
         */
        private static final double SLIP_DETECTION_THRESHOLD = 2.0;

        /** Minimum distance at which vision measurements are trusted, in meters. */
        private static final double TRUST_VISION_RANGE_MIN = 0.25;

        /** Maximum distance at which vision measurements are trusted, in meters. */
        private static final double TRUST_VISION_RANGE_MAX = 3.5;

        /**
         * Angular velocity from the Pigeon gyro.
         *
         * <p>
         * Supplied as a function so this class does not care how the value is computed.
         */
        private final DoubleSupplier pigeonRateSupplier;

        /** Angular velocity estimated by the roboRIO (for example from kinematics). */
        private final DoubleSupplier rioRateSupplier;

        /**
         * Pose from the previous loop, used to compute angular velocity from odometry.
         */
        private Pose2d lastPose = new Pose2d();

        /** FPGA timestamp from the previous loop iteration. */
        private double lastTimeSec = Timer.getFPGATimestamp();

        /**
         * Cached trust value for odometry.
         *
         * <p>
         * 1.0 means "trust odometry fully" (no slip detected), 0.0 means "do not trust
         * odometry"
         * (severe slip detected).
         */
        private double cachedOdometryTrust = 1.0;

        /**
         * Whether slip was detected in the most recent update cycle.
         *
         * <p>
         * Used for telemetry and potential future recovery logic.
         */
        private boolean slipDetected = false;

        /**
         * Constructs the drivetrain with odometry trust logic.
         *
         * @param pigeonRateSupplier angular velocity from the Pigeon gyro
         * @param rioRateSupplier    angular velocity estimated by the roboRIO
         */
        public OdometryDrivetrain(DoubleSupplier pigeonRateSupplier, DoubleSupplier rioRateSupplier) {
                super(
                                TunerConstants.DrivetrainConstants,
                                ODOMETRY_UPDATE_FREQUENCY,
                                TunerConstants.FrontLeft,
                                TunerConstants.FrontRight,
                                TunerConstants.BackLeft,
                                TunerConstants.BackRight);
                this.pigeonRateSupplier = pigeonRateSupplier;
                this.rioRateSupplier = rioRateSupplier;
                lastPose = getState().Pose; // Initialize with actual starting pose
        }

        /**
         * Computes a trust value using exponential decay.
         *
         * <p>
         * If error is small, trust is near 1. If error is large, trust approaches 0.
         *
         * @param error absolute difference between two measurements
         * @param sigma how tolerant we are of error (larger = more tolerant)
         * @return trust value between 0 and 1
         */
        private static double gaussianTrust(double error, double sigma) {
                return Math.exp(-Math.abs(error) / sigma);
        }

        /**
         * Linearly interpolates between two standard deviation matrices.
         *
         * <p>
         * alpha = 0 returns worst, alpha = 1 returns best.
         */
        private static Matrix<N3, N1> interpolateMatrices(
                        Matrix<N3, N1> worst, Matrix<N3, N1> best, double alpha) {
                return VecBuilder.fill(
                                MathUtil.interpolate(worst.get(0, 0), best.get(0, 0), alpha),
                                MathUtil.interpolate(worst.get(1, 0), best.get(1, 0), alpha),
                                MathUtil.interpolate(worst.get(2, 0), best.get(2, 0), alpha));
        }

        @Override
        public void periodic() {
                super.periodic();

                // Compute time since last update
                double now = Timer.getFPGATimestamp();
                double dt = now - lastTimeSec;
                lastTimeSec = now;

                // Protect against divide-by-zero and extremely small timesteps
                if (dt <= 1e-4 || Double.isNaN(dt)) {
                        return;
                }

                Pose2d currentPose = getState().Pose;

                // Read angular velocities from both gyro sources
                double omegaPigeon = pigeonRateSupplier.getAsDouble(); // Pigeon Gyro
                double omegaRio = rioRateSupplier.getAsDouble(); // roboRIO estimated

                // Determine how much the two gyro sources agree (both should be reliable)
                double gyroAgreement = Math.abs(omegaPigeon - omegaRio);
                double gyroAgreementTrust = gaussianTrust(gyroAgreement, GYRO_AGREEMENT_SIGMA);

                // Blend gyro readings based on agreement - this gives us our "ground truth"
                // rotation rate
                double omegaInertial = gyroAgreementTrust * omegaPigeon + (1.0 - gyroAgreementTrust) * omegaRio;

                // Compute rotation rate derived purely from encoder odometry (unreliable during
                // slip)
                double deltaTheta = MathUtil.angleModulus(
                                currentPose.getRotation().getRadians()
                                                - lastPose.getRotation().getRadians());
                double omegaOdometry = deltaTheta / dt;

                // DETECT SLIP: When wheels lift, encoder odometry reports false rotation
                // while gyros (inertial sensors) remain accurate. Large disagreement = slip.
                double slipError = Math.abs(omegaOdometry - omegaInertial);
                slipDetected = slipError > SLIP_DETECTION_THRESHOLD;

                // Trust odometry less when slip is detected
                double odometryTrust = gaussianTrust(slipError, SLIP_DETECTION_THRESHOLD);
                cachedOdometryTrust = odometryTrust;

                // Update telemetry - AdvantageKit friendly direct logging
                Logger.recordOutput("Odometry/Pose", currentPose);
                Logger.recordOutput("Odometry/Trust", odometryTrust);
                Logger.recordOutput("Odometry/SlipDetected", slipDetected);
                Logger.recordOutput("Odometry/Omega/Inertial", omegaInertial);
                Logger.recordOutput("Odometry/Omega/Odometry", omegaOdometry);
                Logger.recordOutput("Odometry/Omega/Pigeon", omegaPigeon);
                Logger.recordOutput("Odometry/Omega/Rio", omegaRio);
                Logger.recordOutput("Odometry/Omega/Disagreement", slipError);

                // Prepare for next iteration
                lastPose = currentPose;
        }

        /**
         * Adds a vision measurement with dynamically adjusted trust based on current
         * odometry reliability.
         *
         * <p>
         * Vision is rejected if:
         * <ul>
         * <li>The measurement is flagged as noisy</li>
         * <li>The target is too close (parallax errors dominate)</li>
         * <li>The target is too far (low resolution, high noise)</li>
         * <li>Odometry is unreliable (slip detected) AND the vision timestamp is old
         * (we can't accurately correct for latency without trustworthy odometry)</li>
         * </ul>
         *
         * <p>
         * When odometry is unreliable due to slip, we increase vision standard
         * deviations
         * (reduce trust) because:
         * <ul>
         * <li>Vision timestamp correction relies on odometry to estimate robot motion
         * during latency</li>
         * <li>If odometry is lying due to wheel slip, timestamp correction becomes
         * inaccurate</li>
         * </ul>
         *
         * @param pose             estimated robot pose from vision
         * @param timestampSeconds timestamp of the measurement (FPGA timestamp in
         *                         seconds)
         * @param distanceMeters   distance to the vision target
         * @param noisy            whether the measurement is known to be unreliable
         */
        public void addVisionMeasurement(
                        Pose2d pose, double timestampSeconds, double distanceMeters, boolean noisy) {
                // REJECT: Obviously bad measurements
                if (noisy
                                || distanceMeters < TRUST_VISION_RANGE_MIN
                                || distanceMeters > TRUST_VISION_RANGE_MAX) {
                        Logger.recordOutput("Vision/Rejected/Reason", "RangeOrNoisy");
                        return;
                }

                // REJECT: Vision measurements that arrived too late when odometry is unreliable
                // Reason: We can't accurately correct for latency without trustworthy odometry
                double latency = Timer.getFPGATimestamp() - timestampSeconds;
                if (latency > 0.2 && cachedOdometryTrust < 0.3) {
                        Logger.recordOutput("Vision/Rejected/Reason", "HighLatencyDuringSlip");
                        return;
                }

                // DYNAMIC TRUST: When odometry is unreliable (slip), reduce vision trust
                // because timestamp correction depends on odometry. When odometry is
                // trustworthy,
                // we can fully trust well-conditioned vision measurements.
                double visionTrustFactor = cachedOdometryTrust; // 1.0 = full trust, 0.0 = minimal trust
                Matrix<N3, N1> visionStd = interpolateMatrices(
                                VISION_STD_WORST, VISION_STD_BEST, 0.2 + 0.8 * visionTrustFactor);

                // ACCEPT: Inject into Kalman filter with appropriate uncertainty
                super.addVisionMeasurement(pose, timestampSeconds, visionStd);
                Logger.recordOutput("Vision/Accepted/Pose", pose);
                Logger.recordOutput("Vision/Accepted/StdDev/X", visionStd.get(0, 0));
                Logger.recordOutput("Vision/Accepted/StdDev/Y", visionStd.get(1, 0));
                Logger.recordOutput("Vision/Accepted/StdDev/Theta", visionStd.get(2, 0));
                Logger.recordOutput("Vision/Accepted/TrustFactor", visionTrustFactor);
        }

        /**
         * Returns the current trust level in encoder-based odometry.
         *
         * <p>
         * 1.0 = fully trustworthy (no slip), 0.0 = completely untrustworthy (severe
         * slip).
         *
         * <p>
         * Useful for other subsystems that need to know if odometry is currently
         * reliable.
         *
         * @return odometry trust value between 0 and 1
         */
        public double getOdometryTrust() {
                return cachedOdometryTrust;
        }

        /**
         * Returns whether wheel slip was detected in the most recent update cycle.
         *
         * <p>
         * Slip detection indicates that one or more swerve modules have lifted off the
         * ground,
         * causing encoder-based odometry to report false motion.
         *
         * @return true if slip was detected
         */
        public boolean isSlipDetected() {
                return slipDetected;
        }
}
