package frc.robot.launcher.turret;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;

public final class TurretConst {
    public static final int MOTOR_ID = -1; // TODO
    public static final int ENCODER_ID = -1; // TODO

    public static final Angle MIN_ANGLE = Degrees.of(0); // TODO
    public static final Angle MAX_ANGLE = Degrees.of(0); // TODO

    /** Number of encoder rotations for every mechanism rotation */
    public static final double ENCODER_TO_MECHANISM_RATIO = 8.5;
}
