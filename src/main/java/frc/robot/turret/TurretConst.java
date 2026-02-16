package frc.robot.turret;

public final class TurretConst {

    // CAN IDs
    public static final int MOTOR_ID = 30;
    public static final int CANCODER_ID = 54;

    // Gear ratios
    public static final double MOTOR_GEAR_RATIO = 102.0;
    public static final double ENCODER_GEAR_RATIO = 8.0;
    public static final double DEGREES_PER_ENCODER_ROTATION = 360.0 / ENCODER_GEAR_RATIO; // 45.0

    // Mechanical limits (0-centered, 630 total travel)
    public static final double MIN_ANGLE_DEG = -315.0;
    public static final double MAX_ANGLE_DEG = 315.0;

    // Zone system for power loss recovery
    public static final int NUM_ZONES = 14; // 630 / 45
    public static final int POWER_ON_ZONE = 0;

    // Deadzone (placeholder - disabled until measured on robot)
    public static final double DEADZONE_MIN_DEG = 0.0;
    public static final double DEADZONE_MAX_DEG = 0.0;
    public static final boolean DEADZONE_ENABLED = false;

    // PID gains
    public static final double kP = 0.035;
    public static final double kI = 0.0;
    public static final double kD = 0.008; // D-term for swerve rotation compensation

    // Output limits
    public static final double MAX_OUTPUT = 0.5;
    public static final double SOFT_LIMIT_MARGIN_DEG = 10.0;

    // On-target tolerance
    public static final double ON_TARGET_TOLERANCE_DEG = 2.0;

    // AprilTag IDs for hub targeting
    public static final int[] RED_HUB_CENTER_TAGS = {9, 10};
    public static final int[] RED_HUB_ALL_TAGS = {9, 10, 8, 5, 11, 2, 4, 3};
    public static final int[] BLUE_HUB_CENTER_TAGS = {26, 25};
    public static final int[] BLUE_HUB_ALL_TAGS = {26, 25, 21, 24, 18, 27, 19, 20};

    // Vision thresholds
    public static final double MAX_YAW_FOR_REFINEMENT_DEG = 30.0;
    public static final double MIN_TARGET_AREA = 0.001;
}
