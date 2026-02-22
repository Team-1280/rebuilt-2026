package frc.robot.trajectory;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

/** Mostly unchanging physical constants for trajectory calculations */
public final class TrajectoryConst {
    /** Magnitude of downwards gravitational acceleration */
    public static final double GRAVITY = 9.80; // m/s^2, California

    /** Projectile radius used to test obstacle collision constraints */
    public static final double PROJECTILE_RADIUS = Inches.of(5.91).in(Meters) / 2;
}
