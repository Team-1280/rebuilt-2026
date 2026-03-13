package frc.robot.launcher.shooter;

import static edu.wpi.first.units.Units.Inches;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.units.measure.Distance;

import frc.robot.launcher.LauncherConst;

public final class ShooterConst {
    public static final int RIGHT_LEADER_MOTOR_ID = 14; // right motor from perspective of launcher
    public static final int LEFT_FOLLOWER_MOTOR_ID = 15; // left motor from perspective of launcher

    public static final CANBus CAN_BUS = LauncherConst.CAN_BUS;

    // flywheel and backrollers - 1:1 pulley ratios currently
    public static final Distance FLYWHEEL_RADIUS = Inches.of(2.0);
    public static final Distance BACKROLLER_RADIUS = Inches.of(1.125);
}
