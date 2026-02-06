package frc.robot.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;

import java.util.function.DoubleSupplier;

public final class DrivetrainOdometry extends CommandSwerveDrivetrain {
    private static final double ODOMETRY_UPDATE_FREQUENCY = 250.0; // hertz

    // vision standard deviations in the form [x, y, theta], for use in the drivetrain constructor
    private static final Matrix<N3, N1> VISION_STD_BEST = VecBuilder.fill(0.05, 0.05, 0.04);
    private static final Matrix<N3, N1> VISION_STD_WORST = VecBuilder.fill(0.60, 0.60, 0.40);

    /*
     * Trust values represent how trustworthy some observation or measurement is.
     * They are unitless scalars between 0.0 and 1.0.
     * A trust value of 0.0 means "do not trust at all" and a trust value of 1.0 means "fully trust".
     */
    /** A trust value that has no impact on the overall trust when combining */
    private static final double TRUST_IDENTITY = 1.0;

    // thresholds for determining trust
    private static final double TRUST_GYRO_LIMIT = 6.0; // rad/s
    private static final double TRUST_SLIP_LIMIT = 0.35; // slip ratio threshold
    private static final double TRUST_VISION_RANGE_MIN = 0.25; // meters
    private static final double TRUST_VISION_RANGE_MAX = 3.5; // meters

    private final DoubleSupplier gyroRateRadPerSecSupplier;
    private final DoubleSupplier slipRatioSupplier;

    private Pose2d lastPose = Pose2d.kZero;
    private double lastTimeSec = Timer.getFPGATimestamp();

    public DrivetrainOdometry(
            DoubleSupplier gyroRateRadPerSecSupplier, DoubleSupplier slipRatioSupplier) {
        super(
                TunerConstants.DrivetrainConstants,
                ODOMETRY_UPDATE_FREQUENCY,
                // TODO:
                // <odometry standard deviation>,
                // <vision standard deviation>:
                // interpolateMatrices(VISION_STD_WORST, VISION_STD_BEST, 0.5),
                TunerConstants.FrontLeft,
                TunerConstants.FrontRight,
                TunerConstants.BackLeft,
                TunerConstants.BackRight);
        this.gyroRateRadPerSecSupplier = gyroRateRadPerSecSupplier;
        this.slipRatioSupplier = slipRatioSupplier;
    }

    /**
     * Converts a boolean condition into a trust value (1.0 if true, 0.0 if false).
     *
     * @param isTrusted a boolean condition of whether to trust something
     * @return a trust value corresponding to the boolean condition
     */
    private static double booleanToTrust(boolean isTrusted) {
        return isTrusted ? 1.0 : 0.0;
    }

    /**
     * Combines multiple trusts into a single trust (order does not matter).
     *
     * <p>For simplicity, the current implementation is just their product.
     *
     * @param trust1 one of the trusts
     * @param trusts the other trusts to combine
     * @return the combined trust
     */
    private static double combineTrusts(double trust1, double... trusts) {
        double combinedTrust = trust1;
        for (double trust : trusts) {
            combinedTrust *= trust;
        }
        return combinedTrust;
    }

    private static Matrix<N3, N1> interpolateMatrices(
            Matrix<N3, N1> start, Matrix<N3, N1> end, double alpha) {
        // TODO: Move to a math utility class
        return VecBuilder.fill(
                MathUtil.interpolate(start.get(0, 0), end.get(0, 0), alpha),
                MathUtil.interpolate(start.get(1, 0), end.get(1, 0), alpha),
                MathUtil.interpolate(start.get(2, 0), end.get(2, 0), alpha));
    }

    @Override
    public void periodic() {
        super.periodic();

        double now = Timer.getFPGATimestamp();
        double dt = now - lastTimeSec;
        lastTimeSec = now;

        Pose2d pose = getState().Pose;
        double translationDelta = pose.getTranslation().getDistance(lastPose.getTranslation());
        lastPose = pose;

        double gyroTrust =
                booleanToTrust(
                        Math.abs(gyroRateRadPerSecSupplier.getAsDouble()) < TRUST_GYRO_LIMIT);
        double slipTrust = booleanToTrust(slipRatioSupplier.getAsDouble() < TRUST_SLIP_LIMIT);

        double odometryTrust = combineTrusts(TRUST_IDENTITY, gyroTrust, slipTrust);
    }

    public void addVisionMeasurement(
            Pose2d pose, double timestampSeconds, double distanceMeters, boolean noisy) {
        double rangeTrust =
                booleanToTrust(
                        distanceMeters > TRUST_VISION_RANGE_MIN
                                && distanceMeters < TRUST_VISION_RANGE_MAX);
        double ambiguityTrust = booleanToTrust(!noisy);
        double visionTrust = combineTrusts(rangeTrust, ambiguityTrust);

        addVisionMeasurement(pose, timestampSeconds);
    }
}
