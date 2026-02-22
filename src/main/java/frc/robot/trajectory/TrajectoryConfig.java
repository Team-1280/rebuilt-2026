package frc.robot.trajectory;

/** Configurable or tunable constants for trajectory calculations */
public final class TrajectoryConfig {
    // Model coefficients:
    // TODO: model coefficients

    /** Multiplication factor to compensate for error or bias */
    public static final double SPEED_MULTIPLIER = 1.00; // TODO: tune SPEED_MULTIPLIER

    /** Fraction from 0 to 1 of the maximum speed to use, when a maximal speed is desired */
    public static final double MAXIMAL_SPEED_FRACTION = 0.9; // TODO: tune MAXIMAL_SPEED_FRACTION
}
