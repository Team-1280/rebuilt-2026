package frc.robot.launcher.hood;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Current;

public final class HoodConfig {
    public static final Current STATOR_LIMIT = Amps.of(80);
    public static final Current SUPPLY_LIMIT = Amps.of(30);
    public static final boolean LIMIT_ENABLE = true;

    public static final TalonFXConfiguration motorConfig = new TalonFXConfiguration();

    static {
        motorConfig.CurrentLimits.StatorCurrentLimit = STATOR_LIMIT.in(Amps);
        motorConfig.CurrentLimits.SupplyCurrentLimit = SUPPLY_LIMIT.in(Amps);
        motorConfig.CurrentLimits.StatorCurrentLimitEnable = LIMIT_ENABLE;
        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = LIMIT_ENABLE;

        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorConfig.MotorOutput.Inverted =
                InvertedValue.CounterClockwise_Positive; // positive is launch pitch upwards,
        // hood *downwards*
        motorConfig.Feedback.SensorToMechanismRatio = HoodConst.ROTOR_TO_MECHANISM_RATIO;
        motorConfig.Slot0.GravityArmPositionOffset = 0.0; // TODO: find
    }
}
