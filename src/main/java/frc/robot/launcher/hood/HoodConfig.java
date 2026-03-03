package frc.robot.launcher.hood;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Current;

public final class HoodConfig {
    public static final Current MOTOR_CURRENT_LIMIT = Amps.of(80);

    public static final TalonFXConfiguration motorConfig = new TalonFXConfiguration();

    static {
        motorConfig.CurrentLimits.StatorCurrentLimit = MOTOR_CURRENT_LIMIT.in(Amps);
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorConfig.MotorOutput.Inverted =
                InvertedValue.CounterClockwise_Positive; // positive is launch pitch upwards, hood *downwards*
        motorConfig.Feedback.SensorToMechanismRatio = HoodConst.ROTOR_TO_MECHANISM_RATIO;
        motorConfig.Slot0.GravityArmPositionOffset = 0.0; // TODO: find
    }
}
