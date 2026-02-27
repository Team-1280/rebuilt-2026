package frc.robot.drivetrain;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.studica.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
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
public final class OdometryDrivetrain extends CommandSwerveDrivetrain {

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

    /** Minimum target distance for trusted vision measurements, in meters. */
    private static final double TRUST_VISION_RANGE_MIN = 0.25;

    /** Maximum target distance for trusted vision measurements, in meters. */
    private static final double TRUST_VISION_RANGE_MAX = 3.5;

    /** Maximum tilt, in radians, for the robot to still be considered flat on the ground. */
    private static final double TILT_THRESHOLD = Units.degreesToRadians(4.0); // TODO

    /** The Kaaba, Al-Masjid al-Haram, Mecca, Saudi Arabia. */
    private static final double MECCA_LAT_RAD = Math.toRadians(21.3891);

    private static final double MECCA_LON_RAD = Math.toRadians(39.8579);

    /** Auto Shop 37.8244069635729, -122.00586640858481 * */
    private static final double DEFAULT_VENUE_LAT = 37.8244069635729;

    private static final double DEFAULT_VENUE_LON = -122.00586640858481;

    private static final double DEFAULT_FIELD_X_COMPASS_DEG = 90.0;

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

    /** Third gyroscope --NavX2 connected via MXP (SPI), used for inertial cross-checking. */
    private final AHRS navX2 = new AHRS(AHRS.NavXComType.kMXP_UART);

    /** TalonFX references for the 4 drive motors (modules 0-3: FL, FR, BL, BR). */
    private final TalonFX[] driveMotors;

    /**
     * Shared heading request used by {@link #faceTowardsMecca}. Gains configured in constructor.
     */
    private final SwerveRequest.FieldCentricFacingAngle m_qiblaRequest =
            new SwerveRequest.FieldCentricFacingAngle();

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

        m_qiblaRequest.HeadingController.setPID(5.0, 0.0, 0.1);
        m_qiblaRequest.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    }

    private static Rotation2d computeQibla(
            double venueLat, double venueLon, double fieldXCompassDeg) {
        double lat1 = Math.toRadians(venueLat);
        double dLon = MECCA_LON_RAD - Math.toRadians(venueLon);
        double y = Math.sin(dLon) * Math.cos(MECCA_LAT_RAD);
        double x =
                Math.cos(lat1) * Math.sin(MECCA_LAT_RAD)
                        - Math.sin(lat1) * Math.cos(MECCA_LAT_RAD) * Math.cos(dLon);
        // atan2 -> compass bearing in degrees (0 = North, CW positive)
        double compassBearingDeg = Math.toDegrees(Math.atan2(y, x));
        // Compass is CW from North; WPILib field angles are CCW from field X+
        return Rotation2d.fromDegrees(fieldXCompassDeg - compassBearingDeg);
    }

    public void alignOperatorPerspectiveToMecca(
            double venueLat, double venueLon, double fieldXCompassDeg) {
        Rotation2d qibla = computeQibla(venueLat, venueLon, fieldXCompassDeg);
        setOperatorPerspectiveForward(qibla);
        Logger.recordOutput("Mecca/QiblaFieldAngleDeg", qibla.getDegrees());
        Logger.recordOutput("Mecca/OperatorPerspectiveAligned", true);
    }

    public void alignOperatorPerspectiveToMecca() {
        alignOperatorPerspectiveToMecca(
                DEFAULT_VENUE_LAT, DEFAULT_VENUE_LON, DEFAULT_FIELD_X_COMPASS_DEG);
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
        double roll = Units.degreesToRadians(navX2.getRoll());
        double pitch = Units.degreesToRadians(navX2.getPitch());
        double tilt = Math.acos(Math.cos(roll) * Math.cos(pitch)); // Combined roll and pitch
        return tilt > TILT_THRESHOLD;
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
        if (!isTilted()) {
            // Return a flat robot pose if not tilted
            return new Pose3d(pose2d);
        }
        double roll = Units.degreesToRadians(navX2.getRoll());
        double pitch = Units.degreesToRadians(navX2.getPitch());
        return new Pose3d(
                pose2d.getX(),
                pose2d.getY(),
                0.0,
                new Rotation3d(roll, pitch, pose2d.getRotation().getRadians()));
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
        double G2 = Units.degreesToRadians(navX2.getRate()); // navx2mxp
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

        double angularJerk = (omegaInertial - lastOmegaInertial) / dt;
        Evidence jerkEvidence = Evidence.of(angularJerk, JERK_SIGMA);
        lastOmegaInertial = omegaInertial;

        // getWorldLinearAccelX/Y return values in g; convert to m/s^2
        double bumpAccel =
                Math.hypot(navX2.getWorldLinearAccelX(), navX2.getWorldLinearAccelY()) * 9.8;
        Evidence bumpEvidence = Evidence.of(bumpAccel, BUMP_ACCEL_SIGMA);

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

        Evidence wheelEvidence =
                Evidence.of(slipError, SLIP_DETECTION_THRESHOLD)
                        .and(jerkEvidence)
                        .and(bumpEvidence)
                        .and(new Evidence(inputCorrelationTrust));
        if (driveCurrentFault) {
            wheelEvidence = new Evidence(0.0);
        }
        cachedOdometryTrust = wheelEvidence.weight();

        // ----- Telemetry -----
        Logger.recordOutput("Odometry/Pose", currentPose);
        Logger.recordOutput("Odometry/Pose3d", getPose3d());
        Logger.recordOutput("Odometry/Tilted", isTilted());
        Logger.recordOutput("Odometry/Trust/Composite", cachedOdometryTrust);
        Logger.recordOutput("Odometry/Trust/Jerk", jerkEvidence.weight());
        Logger.recordOutput("Odometry/Trust/Bump", bumpEvidence.weight());
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
        Logger.recordOutput("Odometry/Jerk/Angular", angularJerk);
        Logger.recordOutput("Odometry/Jerk/LinearBump", bumpAccel);

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
}
