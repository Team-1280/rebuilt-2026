package frc.robot.launcher.shooter;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Distance;

public final class ShooterConst {
    // TODO: add correct motor ids
    public static final int RIGHT_LEADER_MOTOR_ID = -1;
    public static final int LEFT_FOLLOWER_MOTOR_ID = -1;

    // flywheel and backrollers - 1:1 pulley ratios currently
    public static final Distance FLYWHEEL_RADIUS = Inches.of(2.0);
    public static final Distance BACKROLLER_RADIUS = Inches.of(1.125);
}
