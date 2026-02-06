package frc.robot.drivetrain;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.drivetrain.TunerConstants.TunerSwerveDrivetrain;
import java.util.function.Supplier;

/**
 * Class that extends the Phoenix 6 {@code SwerveDrivetrain} class and
 * implements
 * {@link Subsystem} so it can easily be used in command‑based projects.
 */
public class CommandSwerveDrivetrain
        extends TunerSwerveDrivetrain
        implements Subsystem {

    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    /* Blue alliance sees forward as 0° (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180° (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Tracks whether the operator perspective has ever been applied */
    private boolean m_hasAppliedOperatorPerspective = false;

    /** Swerve request to apply during robot‑centric path following */
    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    /* Swerve requests for SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    /*
     * SysId routine for characterizing translation. This is used to find PID gains
     * for the drive motors.
     */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null,
                    Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
                    null,
                    state -> SignalLogger.writeString(
                            "SysIdTranslation_State",
                            state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> setControl(m_translationCharacterization.withVolts(output)),
                    null,
                    this));

    /*
     * SysId routine for characterizing steer. This is used to find PID gains for
     * the steer motors.
     */
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null,
                    Volts.of(7), // Use 7 V for steer characterization
                    null,
                    state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    volts -> setControl(m_steerCharacterization.withVolts(volts)),
                    null,
                    this));

    /*
     * SysId routine for characterizing rotation. This is used to find PID gains
     * for the {@link FieldCentricFacingAngle} heading controller.
     */
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    Volts.of(Math.PI / 6).per(Second), // Rotational acceleration (rad/s²) expressed as volts/sec
                    Volts.of(Math.PI), // Rotational velocity (rad/s) expressed as volts
                    null,
                    state -> SignalLogger.writeString(
                            "SysIdRotation_State",
                            state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> {
                        // Output is actually rad/s, but SysId only supports “volts”
                        setControl(
                                m_rotationCharacterization.withRotationalRate(
                                        output.in(Volts)));
                        // Log the requested output for SysId
                        SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
                    },
                    null,
                    this));

    /* Array of SysId routines to test */
    private final SysIdRoutine[] m_sysIdRoutines = {
            m_sysIdRoutineTranslation,
            m_sysIdRoutineRotation,
            m_sysIdRoutineSteer,
    };
    private int m_sysIdRoutineToApply = 0;

    /**
     * Constructs a CTRE {@code SwerveDrivetrain} using the specified constants.
     *
     * @param drivetrainConstants Drivetrain‑wide constants for the swerve drive
     * @param modules             Constants for each specific module
     */
    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, 0, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
    }

    /**
     * Constructs a CTRE {@code SwerveDrivetrain} using the specified constants.
     *
     * @param drivetrainConstants     Drivetrain‑wide constants for the swerve drive
     * @param odometryUpdateFrequency Frequency to run the odometry loop. If
     *                                unspecified or set to 0 Hz, this defaults
     *                                to 250 Hz on CAN FD and 100 Hz on CAN 2.0.
     * @param modules                 Constants for each specific module
     */
    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
    }

    /**
     * Constructs a CTRE {@code SwerveDrivetrain} using the specified constants.
     *
     * @param drivetrainConstants       Drivetrain‑wide constants for the swerve
     *                                  drive
     * @param odometryUpdateFrequency   Frequency to run the odometry loop. If
     *                                  unspecified or set to 0 Hz, this defaults
     *                                  to 250 Hz on CAN FD and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation Standard deviation for odometry calculation
     *                                  (transpose of [x, y, θ]) in meters and
     *                                  radians.
     * @param visionStandardDeviation   Standard deviation for vision calculation
     *                                  (transpose of [x, y, θ]) in meters and
     *                                  radians.
     * @param modules                   Constants for each specific module
     */
    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            Matrix<N3, N1> odometryStandardDeviation,
            Matrix<N3, N1> visionStandardDeviation,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(
                drivetrainConstants,
                odometryUpdateFrequency,
                odometryStandardDeviation,
                visionStandardDeviation,
                modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
    }

    /** Configures the PathPlanner {@link AutoBuilder}. */
    public void configureAutoBuilder() {
        try {
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                    () -> getState().Pose, // Robot pose supplier
                    this::resetPose, // Reset odometry method
                    () -> getState().Speeds, // ChassisSpeeds supplier
                    (speeds, feedforwards) -> setControl(
                            m_pathApplyRobotSpeeds
                                    .withSpeeds(speeds)
                                    .withWheelForceFeedforwardsX(
                                            feedforwards.robotRelativeForcesXNewtons())
                                    .withWheelForceFeedforwardsY(
                                            feedforwards.robotRelativeForcesYNewtons())),
                    new PPHolonomicDriveController(
                            new PIDConstants(10, 0, 0), // Translation PID
                            new PIDConstants(10, 0, 0) // Rotation PID
                    ),
                    config,
                    () -> {
                        var alliance = DriverStation.getAlliance();
                        if (alliance.isPresent()) {
                            return alliance.get() == DriverStation.Alliance.Red;
                        }
                        return false;
                    },
                    this // Subsystem reference for requirements
            );
        } catch (Exception ex) {
            DriverStation.reportError(
                    "Failed to load PathPlanner config and configure AutoBuilder",
                    ex.getStackTrace());
        }
    }

    /**
     * Returns a command that applies the specified control request to this swerve
     * drivetrain.
     *
     * @param requestSupplier Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /** Advances to the next SysId routine in the array. */
    public void sysIdCycleRoutine() {
        m_sysIdRoutineToApply = (m_sysIdRoutineToApply + 1) % m_sysIdRoutines.length;
    }

    /**
     * Runs the SysId quasistatic test in the given direction for the routine
     * currently selected.
     *
     * @param direction Direction of the SysId quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutines[m_sysIdRoutineToApply].quasistatic(direction);
    }

    /**
     * Runs the SysId dynamic test in the given direction for the routine
     * currently selected.
     *
     * @param direction Direction of the SysId dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutines[m_sysIdRoutineToApply].dynamic(direction);
    }

    @Override
    public void periodic() {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied it before, we apply it regardless of DS state.
         * Otherwise we only apply it when the driver station is disabled.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red
                                ? kRedAlliancePerspectiveRotation
                                : kBlueAlliancePerspectiveRotation);
                m_hasAppliedOperatorPerspective = true;
            });
        }
    }

    final PIDController fieldPositionController = new PIDController(1, 0, 0.1);
    final FieldCentricFacingAngle alignDriveRequest = new FieldCentricFacingAngle()
            .withDeadband(0.1)
            .withRotationalDeadband(0.1)
            .withHeadingPID(1, 0, 0.1);

    /**
     * Returns a command that aligns the robot to a given field position with a
     * certain heading.
     *
     * @param targetPoseSupplier Supplies the desired pose
     */
    public Command getAlignToFieldPosition(
            Supplier<Pose2d> targetPoseSupplier) {
        fieldPositionController.setSetpoint(0);
        fieldPositionController.setTolerance(0.125);
        return run(() -> {
            var relPose = targetPoseSupplier.get().minus(getState().Pose);
            double velMag = fieldPositionController.calculate(
                    relPose.getTranslation().getNorm());
            var vel = relPose.times(velMag);
            setControl(
                    alignDriveRequest
                            .withVelocityX(vel.getX())
                            .withVelocityY(vel.getY())
                            .withTargetDirection(targetPoseSupplier.get().getRotation()));
        })
                .until(fieldPositionController::atSetpoint)
                .withTimeout(2.0);
    }

    /** Starts the simulation thread (used only in simulation). */
    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* Use the measured time delta and battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    /**
     * Adds a vision measurement to the Kalman filter. This will correct the
     * odometry pose estimate while still accounting for measurement noise.
     *
     * @param visionRobotPoseMeters Pose of the robot as measured by the vision
     *                              camera
     * @param timestampSeconds      Timestamp of the vision measurement (seconds)
     */
    @Override
    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters,
            double timestampSeconds) {
        super.addVisionMeasurement(
                visionRobotPoseMeters,
                Utils.fpgaToCurrentTime(timestampSeconds));
    }

    /**
     * Adds a vision measurement to the Kalman filter with custom standard
     * deviations.
     *
     * @param visionRobotPoseMeters    Pose of the robot as measured by the vision
     *                                 camera
     * @param timestampSeconds         Timestamp of the vision measurement (seconds)
     * @param visionMeasurementStdDevs Standard deviations of the vision pose
     *                                 measurement
     *                                 (transpose of [x, y, θ]) in meters and
     *                                 radians
     */
    @Override
    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters,
            double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        super.addVisionMeasurement(
                visionRobotPoseMeters,
                Utils.fpgaToCurrentTime(timestampSeconds),
                visionMeasurementStdDevs);
    }
}
