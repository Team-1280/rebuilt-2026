package frc.robot.launcher.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

public final class ShooterConfig {
    public static final AngularVelocity MAX_ANGULAR_VELOCITY = RotationsPerSecond.of(0.0); // TODO

    // NOTE: these configurations are shared for both motors; identical other than orientation
    // TODO: add correct current limit
    public static final Current MOTOR_CURRENT_LIMIT = Amps.of(80);

    public static final TalonFXConfiguration motorConfig = new TalonFXConfiguration();

    static {
        motorConfig.CurrentLimits.StatorCurrentLimit = MOTOR_CURRENT_LIMIT.in(Amps);
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        // right (from launcher's perspective) is leader, positive is out
        motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    }
}
