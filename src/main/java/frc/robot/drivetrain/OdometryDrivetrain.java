package frc.robot.drivetrain;

import com.ctre.phoenix6.hardware.TalonFX;
import com.studica.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;

import frc.robot.vision.VisionConst;

import org.littletonrobotics.junction.Logger;

/**
 * OdometryDrivetrain extends the base swerve drivetrain with a homotopy evidence fusion pipeline
 * that determines how much the robot should trust odometry vs vision.
 *
 * <p>The trust system is modeled using an {@link Evidence} monoid: each sensor disagreement maps to
 * an evidence weight via a Gaussian morphism, and composing evidence values via {@code and()}
 * yields their categorical product --the intersection of all trust "proofs" (analogous to shared
 * homotopy classes in HoTT). The composite evidence drives vision covariance interpolation.
 *
 * <p>Trust is reduced by:
 *
 * <ul>
 *   <li>Gyro consensus failure (three-source pairwise disagreement)
 *   <li>Angular jerk (sudden rotation change --bumps or collisions)
 *   <li>Linear bump acceleration (NavX2 world-linear accelerometers)
 *   <li>Input correlation failure (robot moving against commanded direction -- defense)
 *   <li>Drive motor CAN fault (all motors report -1 A supply current)
 * </ul>
 */
public final class OdometryDrivetrain extends CommandSwerveDrivetrain implements Sendable {

    // TODO: move constants and config to different files

    /** Odometry thread frequency, must match CAN update rate. */
    private static final double ODOMETRY_UPDATE_FREQUENCY = CommandSwerveIO.ODOMETRY_FREQ;

    /** Vision standard deviations when fully trusted (low = trust more). */
    private static final Matrix<N3, N1> VISION_STD_BEST = VecBuilder.fill(0.05, 0.05, 0.04);

    /** Vision standard deviations when minimally trusted (high = trust less). */
    private static final Matrix<N3, N1> VISION_STD_WORST = VecBuilder.fill(0.60, 0.60, 0.40);

    /**
     * Gaussian sigma for pairwise gyro agreement, in rad/s.
     *
     * <p>Disagreements larger than this reduce per-source weights.
     */
    private static final double GYRO_AGREEMENT_SIGMA = 1.5;

    /**
     * Gaussian sigma for wheel-vs-inertial omega disagreement, in rad/s.
     *
     * <p>Large disagreements indicate wheel slip.
     */
    // TODO: Tune const
    private static final double SLIP_DETECTION_THRESHOLD = 2.0;

    /**
     * Gaussian sigma for angular jerk, in rad/s^2.
     *
     * <p>High jerk indicates sudden rotation change (bump, collision, or tipping).
     */
    private static final double JERK_SIGMA = 8.0;

    /**
     * Gaussian sigma for linear bump acceleration, in m/s^2.
     *
     * <p>Values from NavX2 world-linear accelerometers (g -> m/s^2).
     */
    private static final double BUMP_ACCEL_SIGMA = 4.0;

    /** Threshold below which commanded speed is treated as zero (being pushed / parked). */
    private static final double COMMAND_DEADBAND = 0.05;

    /**
     * Gaussian sigma for unexpected robot motion under zero command, in m/s.
     *
     * <p>Motion above this level while commanding zero indicates external force (defense push).
     */
    private static final double PUSH_DETECTION_SIGMA = 0.3;

    /** Supply current value reported by TalonFX when CAN is lost or motor is disconnected. */
    private static final double SUPPLY_CURRENT_FAULT_VALUE = -1.0;

    /**
     * Gaussian sigma for stator current slip evidence, in amperes.
     *
     * <p>At 80 A (configured slip current), evidence = exp(-80/40) ≈ 0.14.
     */
    private static final double STATOR_SLIP_SIGMA = 40.0;

    /** Minimum target distance for trusted vision measurements, in meters. */
    private static final double TRUST_VISION_RANGE_MIN = 0.1;

    /** Maximum target distance for trusted vision measurements, in meters. */
    private static final double TRUST_VISION_RANGE_MAX = 3.5;

    /** Maximum tilt, in radians, for the robot to still be considered flat on the ground. */
    private static final double TILT_THRESHOLD = Math.toRadians(15);

    /**
     * Commutative monoid for categorical trust fusion.
     *
     * <p>{@code Evidence} represents a proof that sensor readings are consistent. The unit element
     * ({@code weight = 1}) means "no evidence against". Composing two evidences with {@code and()}
     * takes their product --the categorical AND / intersection of proofs. In HoTT terms this is the
     * shared homotopy class of multiple sensor paths.
     *
     * <p>The Gaussian morphism {@code of(error, sigma)} lifts an absolute sensor disagreement into
     * evidence space: small errors -> evidence near 1, large errors -> evidence near 0.
     */
    private record Evidence(double weight) {
        /** Gaussian morphism: maps absolute error to evidence weight in [0, 1]. */
        static Evidence of(double error, double sigma) {
            return new Evidence(Math.exp(-Math.abs(error) / sigma));
        }

        /** Categorical product (AND): the intersection of two proofs. */
        Evidence and(Evidence other) {
            return new Evidence(this.weight * other.weight);
        }

        /** Monoid unit: full trust, no evidence against. */
        static Evidence unit() {
            return new Evidence(1.0);
        }
    }

    /** Pose from the previous loop iteration, used to derive odometry omega. */
    private Pose2d lastPose = new Pose2d();

    /** FPGA timestamp from the previous loop iteration. */
    private double lastTimeSec = Timer.getFPGATimestamp();

    /** Consensus omega from the previous loop iteration, used to compute angular jerk. */
    private double lastOmegaInertial = 0.0;

    /** Pitch rate from the previous loop iteration, used to compute pitch jerk. */
    private double lastPitchRateRadPerSec = 0.0;

    /** Roll rate from the previous loop iteration, used to compute roll jerk. */
    private double lastRollRateRadPerSec = 0.0;

    /** Cached tilt state, computed once per loop in periodic(). */
    private boolean cachedIsTilted = false;

    /**
     * Cached composite trust value for encoder-based odometry.
     *
     * <p>1.0 = fully trustworthy, 0.0 = completely untrustworthy.
     */
    private double cachedOdometryTrust = 1.0;

    /** Whether slip was detected in the most recent update cycle. */
    private boolean slipDetected = false;

    /** Whether all drive motors report a CAN-fault supply current. */
    private boolean driveCurrentFault = false;

    /**
     * Most recently commanded chassis speeds, used for push-detection.
     *
     * <p>Updated each loop by {@link #setCommandedSpeeds(ChassisSpeeds)}.
     */
    private ChassisSpeeds commandedSpeeds = new ChassisSpeeds();

    /** Third gyroscope --NavX2 connected via USB, used for inertial cross-checking. */
    private final AHRS navX2 = new AHRS(AHRS.NavXComType.kMXP_SPI);

    /** TalonFX references for the 4 drive motors (modules 0-3: FL, FR, BL, BR). */
    private final TalonFX[] driveMotors;

    /** Constructs the drivetrain with odometry trust logic. */
    public OdometryDrivetrain() {
        super(
                TunerConstants.DrivetrainConstants,
                ODOMETRY_UPDATE_FREQUENCY,
                TunerConstants.FrontLeft,
                TunerConstants.FrontRight,
                TunerConstants.BackLeft,
                TunerConstants.BackRight);
        lastPose = getPose2d();

        driveMotors = new TalonFX[4];
        for (int i = 0; i < 4; i++) {
            driveMotors[i] = getModule(i).getDriveMotor();
        }

        getPigeon2().getAngularVelocityXWorld().setUpdateFrequency(50.0);
        getPigeon2().getAngularVelocityYWorld().setUpdateFrequency(50.0);
    }

    /**
     * Updates the commanded chassis speeds for push-detection.
     *
     * <p>Call this each loop from teleop or auto with the speeds the controller is requesting. When
     * the robot moves opposite to the command, {@code inputCorrelationTrust} drops, reducing
     * odometry trust.
     *
     * @param speeds commanded chassis speeds (field-relative or robot-relative, consistent with
     *     {@code getState().Speeds})
     */
    public void setCommandedSpeeds(ChassisSpeeds speeds) {
        this.commandedSpeeds = speeds;
    }

    /**
     * Returns the current composite trust level in encoder-based odometry.
     *
     * <p>1.0 = fully trustworthy, 0.0 = completely untrustworthy.
     */
    public double getOdometryTrust() {
        return cachedOdometryTrust;
    }

    /**
     * Returns whether wheel slip was detected in the most recent update cycle.
     *
     * <p>Slip indicates one or more swerve modules have lifted, causing encoder odometry to report
     * false motion.
     */
    public boolean isSlipDetected() {
        return slipDetected;
    }

    /**
     * Returns whether the robot is considered to be tilted, according to NavX2 roll and pitch and
     * the tilt threshold constant.
     */
    public boolean isTilted() {
        return cachedIsTilted;
    }

    /** Returns the latest 2D pose. */
    public Pose2d getPose2d() {
        return getState().Pose;
    }

    /**
     * Returns the latest 3D pose with NavX2 tilt (roll/pitch) applied.
     *
     * <p>If the robot is not tilted, then this is equivalent to the 2D pose in 3D.
     *
     * <p>X/Y match the 2D odometry pose; Z is 0; rotation includes measured roll and pitch from the
     * NavX2 for 3D visualization in AdvantageScope.
     */
    public Pose3d getPose3d() {
        Pose2d pose2d = getPose2d();
        if (!cachedIsTilted) {
            // Return a flat robot pose if not tilted
            return new Pose3d(pose2d);
        }
        double roll = Units.degreesToRadians(getPigeon2().getRoll().getValueAsDouble());
        double pitch = Units.degreesToRadians(getPigeon2().getPitch().getValueAsDouble());
        return new Pose3d(
                pose2d.getX(),
                pose2d.getY(),
                0.0,
                new Rotation3d(roll, pitch, pose2d.getRotation().getRadians()));
    }

    /** Get the robot velocity on the 2D field (field coordinate system) as a Translation2d. */
    public Translation2d getFieldVelocity() {
        ChassisSpeeds chassisSpeeds = getState().Speeds;
        Rotation2d robotRotation = getPose2d().getRotation();
        return new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond)
                .rotateBy(robotRotation.unaryMinus());
    }

    /**
     * Adds a vision measurement with trust dynamically adjusted by the composite evidence pipeline.
     *
     * <p>Vision is rejected when:
     *
     * <ul>
     *   <li>The target is too close or too far (range filter)
     *   <li>The measurement arrived late while odometry is unreliable (latency + slip filter)
     * </ul>
     *
     * <p>When two or more targets are visible (multi-tag PnP), the trust boost is maximal because
     * multi-tag PnP is unambiguous. With a single tag, the boost scales inversely with ambiguity.
     *
     * @param pose estimated robot pose from vision
     * @param timestampSeconds FPGA timestamp of the measurement
     * @param distanceMeters average distance to the targets used
     * @param ambiguity pose ambiguity of the best target (0 = unambiguous, 1 = fully ambiguous)
     * @param numTargets number of AprilTag targets used in the estimate
     */
    public void addVisionMeasurement(
            Pose2d pose,
            double timestampSeconds,
            double distanceMeters,
            double ambiguity,
            int numTargets) {
        if (distanceMeters < TRUST_VISION_RANGE_MIN || distanceMeters > TRUST_VISION_RANGE_MAX) {
            Logger.recordOutput("Vision/Rejected/Reason", "Range");
            return;
        }

        double latency = Timer.getFPGATimestamp() - timestampSeconds;
        if (latency > 0.2 && cachedOdometryTrust < 0.3) {
            Logger.recordOutput("Vision/Rejected/Reason", "HighLatencyDuringSlip");
            return;
        }

        // Multi-tag PnP is unambiguous; single-tag trust scales with ambiguity
        double targetTrustBoost =
                numTargets >= 2 ? 1.0 : (1.0 - ambiguity / VisionConst.MAX_AMBIGUITY);
        double visionTrustFactor = cachedOdometryTrust * targetTrustBoost;

        Matrix<N3, N1> visionStd =
                interpolateMatrices(
                        VISION_STD_WORST, VISION_STD_BEST, 0.2 + 0.8 * visionTrustFactor);

        super.addVisionMeasurement(pose, timestampSeconds, visionStd);
        Logger.recordOutput("Vision/Accepted/Pose", pose);
        Logger.recordOutput("Vision/Accepted/StdDev/X", visionStd.get(0, 0));
        Logger.recordOutput("Vision/Accepted/StdDev/Y", visionStd.get(1, 0));
        Logger.recordOutput("Vision/Accepted/StdDev/Theta", visionStd.get(2, 0));
        Logger.recordOutput("Vision/Accepted/TrustFactor", visionTrustFactor);
        Logger.recordOutput("Vision/Accepted/NumTargets", numTargets);
        Logger.recordOutput("Vision/Accepted/Ambiguity", ambiguity);
    }

    @Override
    public void periodic() {
        super.periodic(); // extends the drivetrain perodic

        double now = Timer.getFPGATimestamp();
        double dt = now - lastTimeSec;
        lastTimeSec = now;

        if (dt <= 1e-4 || Double.isNaN(dt)) {
            return;
        }

        Pose2d currentPose = getPose2d();

        double G1 =
                Units.degreesToRadians(
                        getPigeon2().getAngularVelocityZWorld().getValueAsDouble()); // pigeon2
        double G2 =
                -Units.degreesToRadians(navX2.getRate()); // navx2usb (negated: upside-down Z-axis)
        double G3 = getState().Speeds.omegaRadiansPerSecond; // kinematics

        Evidence e12 = Evidence.of(G1 - G2, GYRO_AGREEMENT_SIGMA); // Pigeon2 vs NavX2
        Evidence e13 = Evidence.of(G1 - G3, GYRO_AGREEMENT_SIGMA); // Pigeon2 vs kinematics
        Evidence e23 = Evidence.of(G2 - G3, GYRO_AGREEMENT_SIGMA); // NavX2 vs kinematics

        double w1 = e12.weight() * e13.weight(); // G1 trusted when it agrees with G2 and G3
        double w2 = e12.weight() * e23.weight(); // G2 trusted when it agrees with G1 and G3
        double w3 = e13.weight() * e23.weight(); // G3 trusted when it agrees with G1 and G2
        double wTotal = w1 + w2 + w3;

        // Weighted consensus omega; fallback to Pigeon2 if all three badly disagree
        double omegaInertial = wTotal < 1e-9 ? G1 : (w1 * G1 + w2 * G2 + w3 * G3) / wTotal;

        // Categorical product of all pairwise agreements -> overall gyro consensus
        // trust
        double gyroConsensusTrust = e12.and(e13).and(e23).weight();

        double yawJerk = (omegaInertial - lastOmegaInertial) / dt;
        lastOmegaInertial = omegaInertial;

        double pitchRateRadPerSec =
                Units.degreesToRadians(getPigeon2().getAngularVelocityYWorld().getValueAsDouble());
        double rollRateRadPerSec =
                Units.degreesToRadians(getPigeon2().getAngularVelocityXWorld().getValueAsDouble());
        double pitchJerk = (pitchRateRadPerSec - lastPitchRateRadPerSec) / dt;
        double rollJerk = (rollRateRadPerSec - lastRollRateRadPerSec) / dt;
        lastPitchRateRadPerSec = pitchRateRadPerSec;
        lastRollRateRadPerSec = rollRateRadPerSec;

        double totalAngularJerk = Math.hypot(Math.hypot(yawJerk, pitchJerk), rollJerk);
        Evidence bumpJerkEvidence = Evidence.of(totalAngularJerk, JERK_SIGMA);

        // getWorldLinearAccel* return values in g; convert to m/s^2; Z captures vertical bounce
        double bumpAccel =
                Math.hypot(
                                Math.hypot(
                                        navX2.getWorldLinearAccelX(), navX2.getWorldLinearAccelY()),
                                navX2.getWorldLinearAccelZ())
                        * 9.8;
        Evidence bumpEvidence = Evidence.of(bumpAccel, BUMP_ACCEL_SIGMA);

        double pigeonRollRad = Units.degreesToRadians(getPigeon2().getRoll().getValueAsDouble());
        double pigeonPitchRad = Units.degreesToRadians(getPigeon2().getPitch().getValueAsDouble());
        double tiltRad =
                Math.acos(
                        MathUtil.clamp(
                                Math.cos(pigeonRollRad) * Math.cos(pigeonPitchRad), -1.0, 1.0));
        cachedIsTilted = tiltRad > TILT_THRESHOLD;

        double cmdLinear =
                Math.hypot(commandedSpeeds.vxMetersPerSecond, commandedSpeeds.vyMetersPerSecond);
        double actLinear =
                Math.hypot(
                        getState().Speeds.vxMetersPerSecond, getState().Speeds.vyMetersPerSecond);
        double cmdOmega = commandedSpeeds.omegaRadiansPerSecond;
        double actOmega = getState().Speeds.omegaRadiansPerSecond;

        double inputCorrelationTrust;
        if (cmdLinear < COMMAND_DEADBAND && Math.abs(cmdOmega) < COMMAND_DEADBAND) {
            // Zero command: motion means something external is pushing the robot
            inputCorrelationTrust = gaussianTrust(actLinear, PUSH_DETECTION_SIGMA);
        } else {
            // Non-zero command: check if velocity direction aligns with intent
            double linearDot =
                    (commandedSpeeds.vxMetersPerSecond * getState().Speeds.vxMetersPerSecond
                                    + commandedSpeeds.vyMetersPerSecond
                                            * getState().Speeds.vyMetersPerSecond)
                            / Math.max(cmdLinear * actLinear, 1e-6);
            // Punish opposite-sign rotation (being spun against command)
            double omegaCorr = cmdOmega * actOmega < 0 ? 0.2 : 1.0;
            inputCorrelationTrust = 0.5 + 0.5 * linearDot * omegaCorr;
        }

        double deltaTheta =
                MathUtil.angleModulus(
                        currentPose.getRotation().getRadians()
                                - lastPose.getRotation().getRadians());
        double omegaOdometry = deltaTheta / dt;
        double slipError = Math.abs(omegaOdometry - omegaInertial);
        slipDetected = slipError > SLIP_DETECTION_THRESHOLD;

        driveCurrentFault = true;
        for (TalonFX motor : driveMotors) {
            if (motor.getSupplyCurrent().getValueAsDouble() != SUPPLY_CURRENT_FAULT_VALUE) {
                driveCurrentFault = false;
                break;
            }
        }
        if (driveCurrentFault) {
            slipDetected = true;
        }

        double maxStatorCurrent = 0.0;
        for (TalonFX motor : driveMotors) {
            double statorA = motor.getStatorCurrent().getValueAsDouble();
            if (statorA > maxStatorCurrent) {
                maxStatorCurrent = statorA;
            }
        }
        Evidence statorEvidence = Evidence.of(maxStatorCurrent, STATOR_SLIP_SIGMA);

        Evidence wheelEvidence =
                Evidence.of(slipError, SLIP_DETECTION_THRESHOLD)
                        .and(bumpJerkEvidence)
                        .and(bumpEvidence)
                        .and(statorEvidence)
                        .and(new Evidence(inputCorrelationTrust));
        if (driveCurrentFault) {
            wheelEvidence = new Evidence(0.0);
        }
        cachedOdometryTrust = wheelEvidence.weight();

        // ----- Telemetry -----
        Logger.recordOutput("Odometry/Pose", currentPose);
        Logger.recordOutput("Odometry/Pose3d", getPose3d());
        Logger.recordOutput("Odometry/Tilted", cachedIsTilted);
        Logger.recordOutput("Odometry/Trust/Composite", cachedOdometryTrust);
        Logger.recordOutput("Odometry/Trust/Jerk", bumpJerkEvidence.weight());
        Logger.recordOutput("Odometry/Trust/Bump", bumpEvidence.weight());
        Logger.recordOutput("Odometry/Trust/Stator", statorEvidence.weight());
        Logger.recordOutput("Odometry/Trust/InputCorrelation", inputCorrelationTrust);
        Logger.recordOutput("Odometry/SlipDetected", slipDetected);
        Logger.recordOutput("Odometry/DriveCurrentFault", driveCurrentFault);
        Logger.recordOutput("Odometry/Gyro/Consensus", gyroConsensusTrust);
        Logger.recordOutput("Odometry/Gyro/Agreement/P2NavX", e12.weight());
        Logger.recordOutput("Odometry/Gyro/Agreement/P2Kin", e13.weight());
        Logger.recordOutput("Odometry/Gyro/Agreement/NavXKin", e23.weight());
        Logger.recordOutput("Odometry/Omega/Consensus", omegaInertial);
        Logger.recordOutput("Odometry/Omega/NavX2", G2);
        Logger.recordOutput("Odometry/Omega/Pigeon", G1);
        Logger.recordOutput("Odometry/Omega/Kinematics", G3);
        Logger.recordOutput("Odometry/Omega/Odometry", omegaOdometry);
        Logger.recordOutput("Odometry/Omega/Disagreement", slipError);
        Logger.recordOutput("Odometry/Jerk/PitchRate", pitchRateRadPerSec);
        Logger.recordOutput("Odometry/Jerk/RollRate", rollRateRadPerSec);
        Logger.recordOutput("Odometry/Jerk/Total3D", totalAngularJerk);
        Logger.recordOutput("Odometry/Jerk/LinearBump", bumpAccel);
        Logger.recordOutput("Odometry/Stator/MaxCurrent", maxStatorCurrent);
        Logger.recordOutput("Odometry/Tilted/PigeonPitch", pigeonPitchRad);
        Logger.recordOutput("Odometry/Tilted/PigeonRoll", pigeonRollRad);

        lastPose = currentPose;
    }

    /**
     * Computes trust via exponential decay.
     *
     * <p>Small error -> trust near 1; large error -> trust near 0. Equivalent to {@code
     * Evidence.of(error, sigma).weight()}.
     */
    private static double gaussianTrust(double error, double sigma) {
        return Math.exp(-Math.abs(error) / sigma);
    }

    /**
     * Linearly interpolates between two standard deviation matrices.
     *
     * <p>alpha = 0 returns {@code worst}, alpha = 1 returns {@code best}.
     */
    private static Matrix<N3, N1> interpolateMatrices(
            Matrix<N3, N1> worst, Matrix<N3, N1> best, double alpha) {
        return VecBuilder.fill(
                MathUtil.interpolate(worst.get(0, 0), best.get(0, 0), alpha),
                MathUtil.interpolate(worst.get(1, 0), best.get(1, 0), alpha),
                MathUtil.interpolate(worst.get(2, 0), best.get(2, 0), alpha));
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("2D pose", () -> getPose2d().toString(), null);
        builder.addStringProperty("3D pose", () -> getPose3d().toString(), null);
        builder.addStringProperty("field velocity", () -> getFieldVelocity().toString(), null);
        builder.addStringProperty("chassis speeds", () -> getState().Speeds.toString(), null);
        builder.addBooleanProperty("is tilted", this::isTilted, null);

        builder.addDoubleProperty(
                "reset pose/x",
                () -> getPose2d().getX(),
                (x) -> {
                    Pose2d pose = getPose2d();
                    resetPose(new Pose2d(x, pose.getY(), pose.getRotation()));
                });
        builder.addDoubleProperty(
                "reset pose/y",
                () -> getPose2d().getY(),
                (y) -> {
                    Pose2d pose = getPose2d();
                    resetPose(new Pose2d(pose.getX(), y, pose.getRotation()));
                });
        builder.addDoubleProperty(
                "reset pose/rotation (deg)",
                () -> getPose2d().getRotation().getDegrees(),
                (deg) -> {
                    Pose2d pose = getPose2d();
                    resetPose(new Pose2d(pose.getTranslation(), Rotation2d.fromDegrees(deg)));
                });
    }
}
