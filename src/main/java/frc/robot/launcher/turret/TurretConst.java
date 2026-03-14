package frc.robot.launcher.turret;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.units.measure.Angle;

import frc.robot.launcher.LauncherConst;

public final class TurretConst {
    public static final int MOTOR_ID = 16;
    public static final int ENCODER_ID = 17;

    public static final CANBus CAN_BUS = LauncherConst.CAN_BUS;

    // TODO: these are tentative, conservative estimates
    public static final Angle MIN_ANGLE = Degrees.of(-180); // TODO
    public static final Angle MAX_ANGLE = Degrees.of(225); // TODO

    /** Number of encoder rotations for every mechanism rotation */
    public static final double ENCODER_TO_MECHANISM_RATIO = 8.5;
}
