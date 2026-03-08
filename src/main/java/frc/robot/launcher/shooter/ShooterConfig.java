package frc.robot.launcher.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

public final class ShooterConfig {
    public static final AngularVelocity MAX_ANGULAR_VELOCITY = RotationsPerSecond.of(90.0);

    public static final Current STATOR_LIMIT = Amps.of(40);
    public static final Current SUPPLY_LIMIT = STATOR_LIMIT;

    // NOTE: these configurations are shared for both motors; identical other than orientation
    public static final TalonFXConfiguration motorConfig = new TalonFXConfiguration();

    static {
        motorConfig.CurrentLimits.StatorCurrentLimit = STATOR_LIMIT.in(Amps);
        motorConfig.CurrentLimits.SupplyCurrentLimit = SUPPLY_LIMIT.in(Amps);

        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        // right (from launcher's perspective) is leader, positive is out
        motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        // Control unit: rotor rotations per second
        motorConfig.Slot0.kP = 0.2;
        motorConfig.Slot0.kS = 0.31;
        motorConfig.Slot0.kV = 0.116;
        motorConfig.MotionMagic.MotionMagicAcceleration = 100.0;
    }
}
