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

/**
 * OdometryDrivetrain extends the base swerve drivetrain with logic that
 * dynamically determines how much the robot should trust odometry vs vision.
 *
 * <p>
 * This class is designed to be readable by rookies:
 *
 * <ul>
 * <li>All math is explained at a high level
 * <li>No assumptions about probability or estimation theory
 * <li>Comments explain "why", not just "what"
 * </ul>
 */
public final class OdometryDrivetrain extends CommandSwerveDrivetrain {

    /**
     * How often odometry is updated, in Hz.
     *
     * <p>
     * This should match the CAN update rate of the drivetrain hardware.
     */
    private static final double ODOMETRY_UPDATE_FREQUENCY =
        CommandSwerveIO.ODOMETRY_FREQ;

    /**
     * Vision standard deviations when vision is performing very well.
     *
     * <p>
     * Lower values mean "trust vision more".
     *
     * <p>
     * Order is: [x meters, y meters, rotation radians]
     */
    private static final Matrix<N3, N1> VISION_STD_BEST = VecBuilder.fill(
        0.05,
        0.05,
        0.04
    );

    /**
     * Vision standard deviations when vision is performing very poorly.
     *
     * <p>
     * Higher values mean "trust vision less".
     */
    private static final Matrix<N3, N1> VISION_STD_WORST = VecBuilder.fill(
        0.60,
        0.60,
        0.40
    );

    /**
     * Acceptable disagreement between two gyro angular velocity measurements
     * before trust is reduced.
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
     * Units are radians per second.
     */
    private static final double ODOMETRY_OMEGA_SIGMA = 2.0;

    /** Minimum distance at which vision measurements are trusted, in meters. */
    private static final double TRUST_VISION_RANGE_MIN = 0.25;

    /** Maximum distance at which vision measurements are trusted, in meters. */
    private static final double TRUST_VISION_RANGE_MAX = 3.5;

    /**
     * Angular velocity from the Pigeon gyro.
     *
     * <p>
     * Supplied as a function so this class does not care how the value is
     * computed.
     */
    private final DoubleSupplier pigeonRateSupplier;

    /**
     * Angular velocity estimated by the roboRIO (for example from kinematics).
     */
    private final DoubleSupplier rioRateSupplier;

    /** Pose from the previous loop, used to compute angular velocity. */
    private Pose2d lastPose = Pose2d.kZero;

    /** FPGA timestamp from the previous loop iteration. */
    private double lastTimeSec = Timer.getFPGATimestamp();

    /**
     * Cached trust value for odometry.
     *
     * <p>
     * 1.0 means "trust odometry fully", 0.0 means "do not trust odometry".
     */
    private double cachedOdometryTrust = 1.0;

    /**
     * Constructs the drivetrain with odometry trust logic.
     *
     * @param pigeonRateSupplier angular velocity from the Pigeon gyro
     * @param rioRateSupplier    angular velocity estimated by the roboRIO
     */
    public OdometryDrivetrain(
        DoubleSupplier pigeonRateSupplier,
        DoubleSupplier rioRateSupplier
    ) {
        super(
            TunerConstants.DrivetrainConstants,
            ODOMETRY_UPDATE_FREQUENCY,
            TunerConstants.FrontLeft,
            TunerConstants.FrontRight,
            TunerConstants.BackLeft,
            TunerConstants.BackRight
        );
        this.pigeonRateSupplier = pigeonRateSupplier;
        this.rioRateSupplier = rioRateSupplier;
    }

    /**
     * Computes a trust value using a simple exponential decay.
     *
     * <p>
     * If error is small, trust is near 1. If error is large, trust approaches 0.
     *
     * @param error difference between two measurements
     * @param sigma how tolerant we are of error
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
        Matrix<N3, N1> worst,
        Matrix<N3, N1> best,
        double alpha
    ) {
        return VecBuilder.fill(
            MathUtil.interpolate(worst.get(0, 0), best.get(0, 0), alpha),
            MathUtil.interpolate(worst.get(1, 0), best.get(1, 0), alpha),
            MathUtil.interpolate(worst.get(2, 0), best.get(2, 0), alpha)
        );
    }

    @Override
    public void periodic() {
        super.periodic();

        // Compute time since last update
        double now = Timer.getFPGATimestamp();
        double dt = now - lastTimeSec;
        lastTimeSec = now;

        // Protect against divide-by-zero and extremely small
        // timesteps
        if (dt <= 1e-4) {
            return;
        }

        Pose2d currentPose = getState().Pose;

        // Read angular velocities from both sources
        double omegaPigeon = pigeonRateSupplier.getAsDouble();
        double omegaRio = rioRateSupplier.getAsDouble();

        // Determine how much the two gyro sources agree
        double gyroAgreementTrust = gaussianTrust(
            omegaPigeon - omegaRio,
            GYRO_AGREEMENT_SIGMA
        );

        // Blend gyro readings based on agreement
        double omegaInertial =
            gyroAgreementTrust * omegaPigeon +
            (1.0 - gyroAgreementTrust) * omegaRio;

        // Compute angular velocity from odometry pose change
        double deltaTheta = MathUtil.angleModulus(
            currentPose.getRotation().getRadians() -
                lastPose.getRotation().getRadians()
        );
        double omegaOdometry = deltaTheta / dt;

        lastPose = currentPose;

        // Compare odometry rotation rate to inertial rotation
        // rate
        double odometryTrust = gaussianTrust(
            omegaOdometry - omegaInertial,
            ODOMETRY_OMEGA_SIGMA
        );

        Logger.recordOutput("Odo/Trust", odometryTrust);
        cachedOdometryTrust = odometryTrust;
    }

    /**
     * Adds a vision measurement with dynamically adjusted trust.
     *
     * <p>
     * Vision is ignored if:
     *
     * <ul>
     * <li>The measurement is flagged as noisy
     * <li>The target is too close
     * <li>The target is too far away
     * </ul>
     *
     * @param pose             estimated robot pose from vision
     * @param timestampSeconds timestamp of the measurement
     * @param distanceMeters   distance to the vision target
     * @param noisy            whether the measurement is known to be unreliable
     */
    public void addVisionMeasurement(
        Pose2d pose,
        double timestampSeconds,
        double distanceMeters,
        boolean noisy
    ) {
        if (
            noisy ||
            distanceMeters < TRUST_VISION_RANGE_MIN ||
            distanceMeters > TRUST_VISION_RANGE_MAX
        ) {
            return;
        }

        // Reduce vision trust when odometry trust is low
        Matrix<N3, N1> visionStd = interpolateMatrices(
            VISION_STD_WORST,
            VISION_STD_BEST,
            1.0 - cachedOdometryTrust
        );

        addVisionMeasurement(pose, timestampSeconds, visionStd);
    }
}
