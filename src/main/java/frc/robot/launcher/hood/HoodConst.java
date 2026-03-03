package frc.robot.launcher.hood;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;

public final class HoodConst {
    public static final int MOTOR_ID = 13;

    /** The minimum launcher pitch; this is only a software limit. */
    public static final Angle MIN_PITCH = Degrees.of(30.0); // TODO: tune

    /** The maximum launcher pitch; note that there is a physical hardstop here. */
    public static final Angle MAX_PITCH = Degrees.of(73.606);

    public static final double ROTOR_TO_MECHANISM_RATIO = 18.0;
}
