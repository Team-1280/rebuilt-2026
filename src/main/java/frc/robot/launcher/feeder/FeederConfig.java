package frc.robot.launcher.feeder;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

public final class FeederConfig {
    public static final AngularVelocity ANGULAR_VELOCITY = RotationsPerSecond.of(-1); // TODO
    public static final Current STATOR_LIMIT = Amps.of(80);
    public static final Current SUPPLY_LIMIT = Amps.of(30);
    public static final boolean LIMIT_ENABLE = true;

    public static final TalonFXConfiguration motorConfig = new TalonFXConfiguration()

    static {
        motorConfig.CurrentLimits.StatorCurrentLimit = STATOR_LIMIT.in(Amps);
        motorConfig.CurrentLimits.SupplyCurrentLimit = SUPPLY_LIMIT.in(Amps);
        motorConfig.CurrentLimits.StatorCurrentLimitEnable = LIMIT_ENABLE;
        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = LIMIT_ENABLE;
        // Brakes to stop feeding fuel when desired
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // feeding is positive
    }
}
