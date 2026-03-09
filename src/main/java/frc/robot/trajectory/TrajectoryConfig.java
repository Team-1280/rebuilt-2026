package frc.robot.trajectory;

/** Configurable or tunable constants for trajectory calculations */
public final class TrajectoryConfig {
    // Model coefficients:
    // Via logistic equation:
    // y = L/(1+e^-(rx+a)))
    // x = (ln(L/y-1)+a)/-r
    // TODO: add pitch into equation
    public static final double MODEL_COEFFICIENT_L = 11.64503;
    public static final double MODEL_COEFFICIENT_R = 0.0121957;
    public static final double MODEL_COEFFICIENT_A = -2.06597;

    /** Multiplication factor to compensate for error or bias */
    public static final double SPEED_MULTIPLIER = 1.00; // TODO: tune SPEED_MULTIPLIER

    /** Fraction from 0 to 1 of the maximum speed to use, when a maximal speed is desired */
    public static final double MAXIMAL_SPEED_FRACTION = 0.9; // TODO: tune MAXIMAL_SPEED_FRACTION

    /**
     * The amount of max possible error in pitch, from the optimal pitch, that the optimizer
     * algorithm accepts. This determines how precise and how expensive the optimizer is.
     */
    public static final double OPTIMIZER_PITCH_TOLERANCE = 0.1; // TODO: tune

    /** An auxiliary upper cap on the number iterations in the pitch optimize algorithm. */
    public static final int OPTIMIZER_MAX_ITERATIONS = 12;
}
