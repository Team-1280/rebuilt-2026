package frc.robot.trajectory;

/** Configurable or tunable constants for trajectory calculations */
public final class TrajectoryConfig {
    // Model coefficients:
    public static final double MODEL_C1 = 18.18081;
    public static final double MODEL_C2 = 9.66252;
    public static final double MODEL_C3 = 1.21679;
    public static final double MODEL_C4 = 0.00849423;
    public static final double MODEL_C5 = -2.4087;

    // R^2 = 0.9902, RMSE = 0.209543

    /**
     * The amount of max possible error in pitch, from the optimal pitch, that the optimizer
     * algorithm accepts. This determines how precise and how expensive the optimizer is.
     */
    public static final double OPTIMIZER_PITCH_TOLERANCE = Math.toRadians(0.1); // TODO: tune

    /** An auxiliary upper cap on the number iterations in the pitch optimize algorithm. */
    public static final int OPTIMIZER_MAX_ITERATIONS = 12;
}
