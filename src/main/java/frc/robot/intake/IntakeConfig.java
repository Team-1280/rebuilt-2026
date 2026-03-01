package frc.robot.intake;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Current;

public final class IntakeConfig {
    public static final double ROLLER_SPEED = 0.0; // TODO

    public static final TalonFXConfiguration deployMotorConfig = new TalonFXConfiguration(); // TODO
    public static final Current DEPLOY_MOTOR_CURRENT_LIMIT = Amps.of(80); // TODO

    static {
        deployMotorConfig.CurrentLimits.StatorCurrentLimit = DEPLOY_MOTOR_CURRENT_LIMIT.in(Amps);
        deployMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        deployMotorConfig.MotorOutput.Inverted =
                InvertedValue.CounterClockwise_Positive; // TODO up positive
        deployMotorConfig.Feedback.SensorToMechanismRatio =
                IntakeConst.DEPLOY_ROTOR_TO_MECHANISM_RATIO;
    }

    public static final TalonFXConfiguration rollerMotorConfig = new TalonFXConfiguration(); // TODO
    public static final Current ROLLER_MOTOR_CURRENT_LIMIT = Amps.of(80); // TODO

    static {
        rollerMotorConfig.CurrentLimits.StatorCurrentLimit = ROLLER_MOTOR_CURRENT_LIMIT.in(Amps);
        rollerMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        rollerMotorConfig.MotorOutput.Inverted =
                InvertedValue.CounterClockwise_Positive; // TODO intake positive
    }
}
