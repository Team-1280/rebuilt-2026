package frc.robot.launcher.turret;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;

public final class TurretConfig {
    /** Stow yaw and the expected startup yaw */
    public static final Angle STOW_YAW = Degrees.of(0); // TODO

    public static final Angle DEADZONE_MIN_ANGLE = Degrees.of(0); // TODO
    public static final Angle DEADZONE_MAX_ANGLE = Degrees.of(0); // TODO

    public static final Current MOTOR_CURRENT_LIMIT = Amps.of(80); // TODO

    public static final TalonFXConfiguration motorConfig = new TalonFXConfiguration();

    static {
        motorConfig.CurrentLimits.StatorCurrentLimit = MOTOR_CURRENT_LIMIT.in(Amps);
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorConfig.MotorOutput.Inverted =
                InvertedValue.CounterClockwise_Positive; // TODO: ccw is positive

        motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        motorConfig.Feedback.FeedbackRemoteSensorID = TurretConst.ENCODER_ID;
        motorConfig.Feedback.SensorToMechanismRatio = TurretConst.ENCODER_TO_MECHANISM_RATIO;
    }

    public static final CANcoderConfiguration encoderConfig = new CANcoderConfiguration();

    static {
        encoderConfig.MagnetSensor.MagnetOffset = 0.0; // TODO
        encoderConfig.MagnetSensor.SensorDirection =
                SensorDirectionValue.CounterClockwise_Positive; // TODO: ccw is positive
    }
}
