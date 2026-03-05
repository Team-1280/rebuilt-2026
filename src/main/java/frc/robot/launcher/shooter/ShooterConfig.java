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

    // TODO
    public static final Current STATOR_LIMIT = Amps.of(60);
    public static final Current SUPPLY_LIMIT = Amps.of(30);

    // NOTE: these configurations are shared for both motors; identical other than orientation
    public static final TalonFXConfiguration motorConfig = new TalonFXConfiguration();

    static {
        motorConfig.CurrentLimits.StatorCurrentLimit = STATOR_LIMIT.in(Amps);
        motorConfig.CurrentLimits.SupplyCurrentLimit = SUPPLY_LIMIT.in(Amps);

        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        // right (from launcher's perspective) is leader, positive is out
        motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        // TODO: shooter gains
        motorConfig.Slot0.kP = 0.0;
        motorConfig.Slot0.kS = 0.0;
        motorConfig.Slot0.kV = 0.0;
        motorConfig.Slot0.kA = 0.0;
        motorConfig.MotionMagic.MotionMagicAcceleration = 0.0;
        motorConfig.MotionMagic.MotionMagicJerk = 0.0;
    }
}
