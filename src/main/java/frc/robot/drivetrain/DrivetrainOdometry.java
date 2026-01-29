package frc.robot.drivetrain;

import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;

import frc.robot.vision.VisionSubsystem;

public class DrivetrainOdometry extends CommandSwerveDrivetrain {

    private Pose2d lastPose = new Pose2d();
    private double lastTimestamp = 0.0;
    private double lastYaw = 0.0;
    private double lastSpeed = 0.0;
    
    // base standard deviation
      private static final double BASE_VISION_STD_DEV = 0.5;
    private static final double BASE_ODOMETRY_STD_DEV = 0.1;
    // uncertainty thresh holds
     private static final double YAW_SPIKE_THRESHOLD = 0.5;        // rad/s
    private static final double ACCEL_SPIKE_THRESHOLD = 3.0;      // m/s^2
    private static final double HIGH_SPEED_THRESHOLD = 3.0;       // m/s
    
    public void addVisionWithPoseScaling(Pose2d visionPose, double timestampSeconds, double distanceToTag, double Ambiguity) {
         // scale stdDev based on distance (further = less trust)
        double distanceScale = 1.0 + (distanceToTag * 0.5);
        
        // scale based on ambiguity (higher = less trust)
        double ambiguityScale = 1.0 + (Ambiguity * 2.0);
        
        double scaledStdDev = BASE_VISION_STD_DEV * distanceScale * ambiguityScale;
        
        setVisionMeasurementStdDevs(VecBuilder.fill(scaledStdDev, scaledStdDev, scaledStdDev));
        addVisionMeasurement(visionPose, timestampSeconds);
    }

 
     //detects if robot is in uncertain state (bump, acceleration, yaw spike).
    
    public boolean isInUncertainState() {
        double currentYaw = getEstimatedHeading().getRadians();
        double currentSpeed = Math.hypot(
            getState().Speeds.vxMetersPerSecond,
            getState().Speeds.vyMetersPerSecond
        );
        double dt = getPoseUpdatePeriod();
        
        if (dt <= 0) return false;
        
        // yaw rate
        double yawRate = Math.abs(currentYaw - lastYaw) / dt;
        
        // acceleration
        double acceleration = Math.abs(currentSpeed - lastSpeed) / dt;
        
        // check for spikes
        boolean yawSpike = yawRate > YAW_SPIKE_THRESHOLD;
        boolean accelSpike = acceleration > ACCEL_SPIKE_THRESHOLD;
        boolean highSpeed = currentSpeed > HIGH_SPEED_THRESHOLD;
        
        return yawSpike | accelSpike | highSpeed;
    }


    // GEts wheel slip probability based on speed and acceleration.
     
    public double getWheelSlipProbability() {
        double currentSpeed = Math.hypot(
            getState().Speeds.vxMetersPerSecond,
            getState().Speeds.vyMetersPerSecond
        );
        double dt = getPoseUpdatePeriod();
        
        if (dt <= 0) return 0.0;
        
        double acceleration = Math.abs(currentSpeed - lastSpeed) / dt;
        
        // hogher acceleratio0n = higher slip chance
        // normalize to 0-1 range i think
        return Math.min(1.0, acceleration / 10.0);
    }

  
     //dynamically adjusts odometry trust based on conditions.
      //call this when uncertain - increases odometry stdDev (trusts it less).
    
    public void updateOdometryTrust() {
        double slipProbability = getWheelSlipProbability();
        boolean uncertain = isInUncertainState();
        
        double odometryScale = 1.0;
        
        if (uncertain) {
            odometryScale += 2.0;  // Trust odometry less
        }
        
        odometryScale += slipProbability * 3.0;  // slip = trust even less
        
        // note: CTRE's CommandSwerveDrivetrain may not expose odometry stdDev adjustment aparently
        // this would need to be handled differently depending on your base class
    }
    // under this i think it needs to store values for next comparison and updatecache but idk how to do

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
        super(drivetrainConstants, odometryFrequency, odometryStdDevs, visionStdDevs, modules);
        lastTimestamp = Timer.getFPGATimestamp();
    }

    // Returns the Pose2d of the robot's location
    public Pose2d getEstimatedPose() {
        return getState().Pose;
    }

    private VisionSubsystem vision;

    public void setVisionSubsystem(VisionSubsystem vision) {
        this.vision = vision;
    }

    // Returns a Rotation2d location of where the HEAD of robot is to it's "zeroed"
    // angled
    public Rotation2d getEstimatedHeading() {
        return getState().Pose.getRotation();
    }

    public Pose2d getRobotPose() {
        return getEstimatedPose();
    }

    public void resetOdometry(Pose2d pose) {
        seedFieldCentric(getEstimatedHeading());
        lastPose = pose;
    }

    public void resetHeading(Rotation2d heading) {
        Pose2d current = getEstimatedPose();
        resetOdometry(new Pose2d(current.getTranslation(), heading));
    }

    public void addVisionPose(Pose2d visionPose, double timestampSeconds) {
        addVisionMeasurement(visionPose, timestampSeconds);
    }

    public double getPoseUpdatePeriod() {
        return Timer.getFPGATimestamp() - lastTimestamp;
    }

    public void updateCache() {
        lastPose = getEstimatedPose();
        lastTimestamp = Timer.getFPGATimestamp();
    }

    @Override
    public void periodic() {
        super.periodic();
        updateCache();
    }
}
