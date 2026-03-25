package frc.robot.launcher.hood;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.units.measure.Angle;

import frc.robot.launcher.LauncherConst;

public final class HoodConst {
    public static final int MOTOR_ID = 13;

    public static final CANBus CAN_BUS = LauncherConst.CAN_BUS;

    /** The minimum launcher pitch; this is only a software limit. */
    public static final Angle MIN_PITCH = Degrees.of(15.0);

    /** The maximum launcher pitch; note that there is a physical hardstop here. */
    public static final Angle MAX_PITCH = Degrees.of(73.606);

    public static final double ROTOR_TO_MECHANISM_RATIO = 24.0;
}
