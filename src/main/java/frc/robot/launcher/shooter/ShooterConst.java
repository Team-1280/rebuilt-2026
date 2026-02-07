package frc.robot.launcher.shooter;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Distance;

public class ShooterConst {
    // motor ids TODO: add correct motor ids
    public static final int RIGHT_MOTOR_ID = -1; // right motor is leader, left follower
    public static final int LEFT_MOTOR_ID = -1;
    // flywheel and backrollers - 1:1 pulley ratios currently
    public static final Distance FLYWHEEL_RADIUS = Inches.of(1.5);
    public static final Distance BACKROLLER_RADIUS = Inches.of(1.125);
}
