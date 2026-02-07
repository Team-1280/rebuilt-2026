package frc.robot.shooter;


import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;

public class ShooterConst {
    // min and max angles (degrees) - max angle reinforced by a hard stop
    public static final Angle MIN_ANGLE = Degrees.of(30.0);
    public static final Angle MAX_ANGLE = Degrees.of(65.159);
    // flywheel and backrollers - 1:1 pulley ratios (differences only come from wheel size)
    public static final Distance FLYWHEEL_RADIUS = Inches.of(1.5);
    public static final Distance BACKROLLER_RADIUS = Inches.of(1.125);
    // motor ids TODO: add correct motor ids
    public static final int RIGHT_MOTOR_ID = -1; // right motor is leader, left follower
    public static final int LEFT_MOTOR_ID = -1;
    // TODO: add correct current limits
    public static final Current SHOOTER_MOTORS_CURRENT_LIMIT = Amps.of(80);

    public static final TalonFXConfiguration shooterMotorConfig = new TalonFXConfiguration();

    static {
        shooterMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        shooterMotorConfig.CurrentLimits.StatorCurrentLimit = SHOOTER_MOTORS_CURRENT_LIMIT.in(Amps);
        shooterMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        shooterMotorConfig.MotorOutput.Inverted =
            InvertedValue.CounterClockwise_Positive; // right is leader, positive is out
    }
}
