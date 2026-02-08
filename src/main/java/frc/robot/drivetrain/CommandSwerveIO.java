package frc.robot.drivetrain;

import static edu.wpi.first.units.Unit.*;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import java.util.List;
import java.util.ArrayList;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

public interface ModuleIO {
    @AutoLog
    public static class ModuleIOInputs {
        public boolean driveConnected = false;
        public double drivePositionRad = 0.0;
        public double driveVelocityRadPerSec = 0.0;
        public double driveAppliedVolts = 0.0;
        public double driveCurrentAmps = 0.0;

        public boolean turnConnected = false;
        public boolean turnEncoderConnected = false;
        public Rotation2d turnAbsolutePosition = Rotation2d.kZero;
        public Rotation2d turnPosition = Rotation2d.kZero;
        public double turnVelocityRadPerSec = 0.0;
        public double turnAppliedVolts = 0.0;
        public double turnCurrentAmps = 0.0;

        public double[] odometryTimestamps = new double[] {};
        public double[] odometryDrivePositionsRad = new double[] {};
        public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(ModuleIOInputs inputs) {
    }

    /** Run the drive motor at the specified open loop value. */
    public default void setDriveOpenLoop(double output) {
    }

    /** Run the turn motor at the specified open loop value. */
    public default void setTurnOpenLoop(double output) {
    }

    /** Run the drive motor at the specified velocity. */
    public default void setDriveVelocity(double velocityRadPerSec) {
    }

    /** Run the turn motor to the specified rotation. */
    public default void setTurnPosition(Rotation2d rotation) {
    }
}

public class CommandSwerveIO {

    static final double ODOMETRY_FREQ = TunerConstants.kCANBus.isNetworkFD()
            ? 250.0
            : 100.0;
    public static final double DRIVE_BASE_RADIUS = Math.max(
            Math.max(
                    Math.hypot(
                            TunerConstants.FrontLeft.LocationX,
                            TunerConstants.FrontLeft.LocationY),
                    Math.hypot(
                            TunerConstants.FrontRight.LocationX,
                            TunerConstants.FrontRight.LocationY)),
            Math.max(
                    Math.hypot(
                            TunerConstants.BackLeft.LocationX,
                            TunerConstants.BackLeft.LocationY),
                    Math.hypot(
                            TunerConstants.BackRight.LocationX,
                            TunerConstants.BackRight.LocationY)));
    private static final double DRIVE_MASS_KG = 0x0deadbeef; // TODO:add mass
    private static final double ROBOT_MOI = 6.088;
    private static final double WHEEL_COF = 1.02;

    // private static final RobotConfig PP_CONFIG = new RobotConfig(
    // ROBOT_MASS_KG,
    // ROBOT_MOI,
    // new ModuleConfig(
    // TunerConstants.FrontLeft.WheelRadius,
    // TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
    // WHEEL_COF,
    // DCMotor.getKrakenX60Foc(1).withReduction(
    // TunerConstants.FrontLeft.DriveMotorGearRatio
    // ),
    // TunerConstants.FrontLeft.SlipCurrent,
    // 1
    // ),
    // getModuleTranslations()
    // );
    public class PhoenixOdometryThread extends Thread {
        private final Lock signalsLock = new ReentrantLock(); // Prevents conflicts when registering signals
        private BaseStatusSignal[] phoenixSignals = new BaseStatusSignal[0];
        private final List<DoubleSupplier> genericSignals = new ArrayList<>();
        private final List<Queue<Double>> phoenixQueues = new ArrayList<>();
        private final List<Queue<Double>> genericQueues = new ArrayList<>();
        private final List<Queue<Double>> timestampQueues = new ArrayList<>();

        private static boolean isCANFD = TunerConstants.kCANBus.isNetworkFD();
        private static PhoenixOdometryThread instance = null;

        public static PhoenixOdometryThread getInstance() {
            if (instance == null) {
                instance = new PhoenixOdometryThread();
            }
            return instance;
        }

        private PhoenixOdometryThread() {
            setName("PhoenixOdometryThread");
            setDaemon(true);
        }

        @Override
        public void start() {
            if (timestampQueues.size() > 0) {
                super.start();
            }
        }

        /** Registers a Phoenix signal to be read from the thread. */
        public Queue<Double> registerSignal(StatusSignal<Angle> signal) {
            Queue<Double> queue = new ArrayBlockingQueue<>(20);
            signalsLock.lock();
            odometryLock.lock();
            try {
                BaseStatusSignal[] newSignals = new BaseStatusSignal[phoenixSignals.length + 1];
                System.arraycopy(phoenixSignals, 0, newSignals, 0, phoenixSignals.length);
                newSignals[phoenixSignals.length] = signal;
                phoenixSignals = newSignals;
                phoenixQueues.add(queue);
            } finally {
                signalsLock.unlock();
                odometryLock.unlock();
            }
            return queue;
        }

        /** Registers a generic signal to be read from the thread. */
        public Queue<Double> registerSignal(DoubleSupplier signal) {
            Queue<Double> queue = new ArrayBlockingQueue<>(20);
            signalsLock.lock();
            odometryLock.lock();
            try {
                genericSignals.add(signal);
                genericQueues.add(queue);
            } finally {
                signalsLock.unlock();
                odometryLock.unlock();
            }
            return queue;
        }

        /** Returns a new queue that returns timestamp values for each sample. */
        public Queue<Double> makeTimestampQueue() {
            Queue<Double> queue = new ArrayBlockingQueue<>(20);
            odometryLock.lock();
            try {
                timestampQueues.add(queue);
            } finally {
                odometryLock.unlock();
            }
            return queue;
        }

        @Override
        public void run() {
            while (true) {
                // Wait for updates from all signals
                signalsLock.lock();
                try {
                    if (isCANFD && phoenixSignals.length > 0) {
                        BaseStatusSignal.waitForAll(2.0 / ODOMETRY_FREQ, phoenixSignals);
                    } else {
                        // "waitForAll" does not support blocking on multiple signals with a bus
                        // that is not CAN FD, regardless of Pro licensing. No reasoning for this
                        // behavior is provided by the documentation.
                        Thread.sleep((long) (1000.0 / ODOMETRY_FREQ));
                        if (phoenixSignals.length > 0)
                            BaseStatusSignal.refreshAll(phoenixSignals);
                    }
                } catch (InterruptedException e) {
                    e.printStackTrace();
                } finally {
                    signalsLock.unlock();
                }

                // Save new data to queues
                odometryLock.lock();
                try {
                    // Sample timestamp is current FPGA time minus average CAN latency
                    // Default timestamps from Phoenix are NOT compatible with
                    // FPGA timestamps, this solution is imperfect but close
                    double timestamp = RobotController.getFPGATime() / 1e6;
                    double totalLatency = 0.0;
                    for (BaseStatusSignal signal : phoenixSignals) {
                        totalLatency += signal.getTimestamp().getLatency();
                    }
                    if (phoenixSignals.length > 0) {
                        timestamp -= totalLatency / phoenixSignals.length;
                    }

                    // Add new samples to queues
                    for (int i = 0; i < phoenixSignals.length; i++) {
                        phoenixQueues.get(i).offer(phoenixSignals[i].getValueAsDouble());
                    }
                    for (int i = 0; i < genericSignals.size(); i++) {
                        genericQueues.get(i).offer(genericSignals.get(i).getAsDouble());
                    }
                    for (int i = 0; i < timestampQueues.size(); i++) {
                        timestampQueues.get(i).offer(timestamp);
                    }
                } finally {
                    odometryLock.unlock();
                }
            }
        }
    }

    public static enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }

    public final class GyroIOPigeon2 implements GyroIO {
        private final Pigeon2 pigeon = new Pigeon2(
                TunerConstants.DrivetrainConstants.Pigeon2Id,
                TunerConstants.kCANBus);

        private final StatusSignal<Angle> yaw = pigeon.getYaw();
        private final StatusSignal<AngularVelocity> yawVelocity = pigeon.getAngularVelocityZWorld();

        private final Queue<Double> yawPosQueue = PhoenixOdometryThread.getInstance().registerSignal(yaw.clone());
        private final Queue<Double> yawTimeQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();

        public GyroIOPigeon2() {
            pigeon
                    .getConfigurator()
                    .apply(
                            TunerConstants.DrivetrainConstants.Pigeon2Configs != null
                                    ? TunerConstants.DrivetrainConstants.Pigeon2Configs
                                    : new Pigeon2Configuration());

            pigeon.getConfigurator().setYaw(0.0);
            yaw.setUpdateFrequency(ODOMETRY_FREQ);
            yawVelocity.setUpdateFrequency(50.0);
            pigeon.optimizeBusUtilization();
        }

        @Override
        public void updateInputs(GyroIOInputs inputs) {
            inputs.connected = BaseStatusSignal.refreshAll(yaw, yawVelocity) == StatusCode.OK;

            inputs.yaw = Rotation2d.fromDegrees(yaw.getValueAsDouble());
            inputs.yawRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());

            inputs.odoYawTimestamps = yawTimeQueue.stream().mapToDouble(Double::doubleValue).toArray();

            inputs.odoYawPositions = yawPosQueue.stream()
                    .map(Rotation2d::fromDegrees)
                    .toArray(Rotation2d[]::new);

            yawTimeQueue.clear();
            yawPosQueue.clear();
        }

    }

    static final Lock odoLock = new ReentrantLock();
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final Module[] modules = new Module[4]; // FL, FR, BL(Yaoi), BR
    private final SysIdRoutine sysId;
    private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
    private Rotation2d rawGyroRotation = Rotation2d.kZero;
    private SwerveModulePosition[] lastModulePositions = // For delta tracking
            new SwerveModulePosition[] {
                    new SwerveModulePosition(),
                    new SwerveModulePosition(),
                    new SwerveModulePosition(),
                    new SwerveModulePosition()
            };
    private SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(kinematics, rawGyroRotation,
            lastModulePositions, Pose2d.kZero);

    public CommandSwerveIO(
            GyroIO gyroIO,
            ModuleIO flModuleIO,
            ModuleIO frModuleIO,
            ModuleIO blModuleIO,
            ModuleIO brModuleIO) {
        this.gyroIO = gyroIO;
        modules[0] = new Module(flModuleIO, 0, TunerConstants.FrontLeft);
        modules[1] = new Module(frModuleIO, 1, TunerConstants.FrontRight);
        modules[2] = new Module(blModuleIO, 2, TunerConstants.BackLeft);
        modules[3] = new Module(brModuleIO, 3, TunerConstants.BackRight);

        // Usage reporting for swerve template
        HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

        // Start odometry thread
        PhoenixOdometryThread.getInstance().start();

        // Configure AutoBuilder for PathPlanner
        AutoBuilder.configure(
                this::getPose,
                this::setPose,
                this::getChassisSpeeds,
                this::runVelocity,
                new PPHolonomicDriveController(
                        new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
                PP_CONFIG,
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this);
        // Pathfinding.setPathfinder(new LocalADStarAK());
        // PathPlannerLogging.setLogActivePathCallback(
        // (activePath) -> {
        // Logger.recordOutput("Odometry/Trajectory", activePath.toArray(new
        // Pose2d[0]));
        // });
        // PathPlannerLogging.setLogTargetPoseCallback(
        // (targetPose) -> {
        // Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        // });

        // Configure SysId
        sysId = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null,
                        null,
                        null,
                        (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism(
                        (voltage) -> runCharacterization(voltage.in(Volts)), null, this));
    }

    @Override
    public void periodic() {
        odometryLock.lock(); // Prevents odometry updates while reading data
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Drive/Gyro", gyroInputs);
        for (var module : modules) {
            module.periodic();
        }
        odometryLock.unlock();

        // Stop moving when disabled
        if (DriverStation.isDisabled()) {
            for (var module : modules) {
                module.stop();
            }
        }

        // Log empty setpoint states when disabled
        if (DriverStation.isDisabled()) {
            Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
            Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
        }

        // Update odometry
        double[] sampleTimestamps = modules[0].getOdometryTimestamps(); // All signals are sampled together
        int sampleCount = sampleTimestamps.length;
        for (int i = 0; i < sampleCount; i++) {
            // Read wheel positions and deltas from each module
            SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
            SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
            for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
                modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
                moduleDeltas[moduleIndex] = new SwerveModulePosition(
                        modulePositions[moduleIndex].distanceMeters
                                - lastModulePositions[moduleIndex].distanceMeters,
                        modulePositions[moduleIndex].angle);
                lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
            }

            // Update gyro angle
            if (gyroInputs.connected) {
                // Use the real gyro angle
                rawGyroRotation = gyroInputs.odometryYawPositions[i];
            } else {
                // Use the angle delta from the kinematics and module deltas
                Twist2d twist = kinematics.toTwist2d(moduleDeltas);
                rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
            }

            // Apply update
            poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
        }

        // Update gyro alert
        new Alert("Oh nyo, your gyro has disconnected, i'm falling back to kinematics UwU")
                .set(!gyroInputs.connected && Constants.currentMode != Mode.SIM);
    }

    /**
     * Runs the drive at the desired velocity.
     *
     * @param speeds Speeds in meters/sec
     */
    public void runVelocity(ChassisSpeeds speeds) {
        // Calculate module setpoints
        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
        SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, TunerConstants.kSpeedAt12Volts);

        // Log unoptimized setpoints and setpoint speeds
        Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
        Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

        // Send setpoints to modules
        for (int i = 0; i < 4; i++) {
            modules[i].runSetpoint(setpointStates[i]);
        }

        // Log optimized setpoints (runSetpoint mutates each state)
        Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
    }

    /** Runs the drive in a straight line with the specified drive output. */
    public void runCharacterization(double output) {
        for (int i = 0; i < 4; i++) {
            modules[i].runCharacterization(output);
        }
    }

    /** Stops the drive. */
    public void stop() {
        runVelocity(new ChassisSpeeds());
    }

    /**
     * Stops the drive and turns the modules to an X arrangement to resist movement.
     * The modules will
     * return to their normal orientations the next time a nonzero velocity is
     * requested.
     */
    public void stopWithX() {
        Rotation2d[] headings = new Rotation2d[4];
        for (int i = 0; i < 4; i++) {
            headings[i] = getModuleTranslations()[i].getAngle();
        }
        kinematics.resetHeadings(headings);
        stop();
    }

    /** Returns a command to run a quasistatic test in the specified direction. */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0))
                .withTimeout(1.0)
                .andThen(sysId.quasistatic(direction));
    }

    /** Returns a command to run a dynamic test in the specified direction. */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
    }

    /**
     * Returns the module states (turn angles and drive velocities) for all of the
     * modules.
     */
    @AutoLogOutput(key = "SwerveStates/Measured")
    private SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    /**
     * Returns the module positions (turn angles and drive positions) for all of the
     * modules.
     */
    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] states = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getPosition();
        }
        return states;
    }

    /** Returns the measured chassis speeds of the robot. */
    @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
    private ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    /** Returns the position of each module in radians. */
    public double[] getWheelRadiusCharacterizationPositions() {
        double[] values = new double[4];
        for (int i = 0; i < 4; i++) {
            values[i] = modules[i].getWheelRadiusCharacterizationPosition();
        }
        return values;
    }

    /**
     * Returns the average velocity of the modules in rotations/sec (Phoenix native
     * units).
     */
    public double getFFCharacterizationVelocity() {
        double output = 0.0;
        for (int i = 0; i < 4; i++) {
            output += modules[i].getFFCharacterizationVelocity() / 4.0;
        }
        return output;
    }

    /** Returns the current odometry pose. */
    @AutoLogOutput(key = "Odometry/Robot")
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /** Returns the current odometry rotation. */
    public Rotation2d getRotation() {
        return getPose().getRotation();
    }

    /** Resets the current odometry pose. */
    public void setPose(Pose2d pose) {
        poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
    }

    /** Adds a new timestamped vision measurement. */
    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters,
            double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        poseEstimator.addVisionMeasurement(
                visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    }

    /** Returns the maximum linear speed in meters per sec. */
    public double getMaxLinearSpeedMetersPerSec() {
        return TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    }

    /** Returns the maximum angular speed in radians per sec. */
    public double getMaxAngularSpeedRadPerSec() {
        return getMaxLinearSpeedMetersPerSec() / DRIVE_BASE_RADIUS;
    }

    /** Returns an array of module translations. */
    public static Translation2d[] getModuleTranslations() {
        return new Translation2d[] {
                new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
                new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
                new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
                new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
        };
    }
    // private final Alert gyroDisconnect = new Alert("Oh nyo, your gyro has
    // disconnected, i'm falling back to kinematics UwU");
}
