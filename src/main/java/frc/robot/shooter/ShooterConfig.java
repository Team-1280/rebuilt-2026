package frc.robot.shooter;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Current;

public class ShooterConfig {
    // TODO: add correct current limit
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
