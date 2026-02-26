package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;

public final class DriveConfig {
    /** Maximum throttle linear drive speed. */
    public static final LinearVelocity MAX_SPEED = MetersPerSecond.of(1.6);

    /** Maximum throttle angular drive speed. */
    public static final AngularVelocity MAX_ANGULAR_SPEED = RotationsPerSecond.of(0.5);

    /** Minimum registerable linear drive speed. */
    public static final LinearVelocity SPEED_DEADBAND = MAX_SPEED.times(0.1);

    /** Minimum registerable angular drive speed. */
    public static final AngularVelocity ANGULAR_SPEED_DEADBAND = MAX_ANGULAR_SPEED.times(0.1);
}
