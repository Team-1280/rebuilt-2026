package frc.robot.launcher.hood;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Current;

public final class HoodConfig {
    // TODO; note that hood is very powerful and so supply draw is mostly negligible
    public static final Current STATOR_LIMIT = Amps.of(40);
    public static final Current SUPPLY_LIMIT = Amps.of(20);

    public static final TalonFXConfiguration motorConfig = new TalonFXConfiguration();

    static {
        motorConfig.CurrentLimits.StatorCurrentLimit = STATOR_LIMIT.in(Amps);
        motorConfig.CurrentLimits.SupplyCurrentLimit = SUPPLY_LIMIT.in(Amps);

        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        // positive is launch pitch upwards, hood *downwards*
        motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        motorConfig.Feedback.SensorToMechanismRatio = HoodConst.ROTOR_TO_MECHANISM_RATIO;
        motorConfig.Slot0.GravityArmPositionOffset = 0.0; // TODO: find
    }
}
