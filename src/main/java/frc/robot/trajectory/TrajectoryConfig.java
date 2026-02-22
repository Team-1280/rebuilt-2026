package frc.robot.trajectory;

/** Configurable or tunable constants for trajectory calculations */
public final class TrajectoryConfig {
    // Model coefficients:
    // TODO: model coefficients

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
