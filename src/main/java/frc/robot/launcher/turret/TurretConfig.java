package frc.robot.launcher.turret;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

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
    public static final Angle STOW_YAW = Degrees.of(0);

    public static final Angle YAW_TOLERANCE = Degrees.of(3.0);

    public static final Current STATOR_LIMIT = Amps.of(30);
    public static final Current SUPPLY_LIMIT = STATOR_LIMIT;

    public static final TalonFXConfiguration motorConfig = new TalonFXConfiguration();

    static {
        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
                TurretConst.MAX_ANGLE.in(Rotations);
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
                TurretConst.MIN_ANGLE.in(Rotations);

        motorConfig.CurrentLimits.StatorCurrentLimit = STATOR_LIMIT.in(Amps);
        motorConfig.CurrentLimits.SupplyCurrentLimit = SUPPLY_LIMIT.in(Amps);

        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // ccw yaw is positive

        motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        motorConfig.Feedback.FeedbackRemoteSensorID = TurretConst.ENCODER_ID;
        motorConfig.Feedback.SensorToMechanismRatio = TurretConst.ENCODER_TO_MECHANISM_RATIO;

        // Control unit: mechanism rotations
        motorConfig.Slot0.kP = 100.0;
        motorConfig.Slot0.kS = 0.22;
        motorConfig.Slot0.kV = 3.1;
        motorConfig.Slot0.kA = 0.15;
        motorConfig.MotionMagic.MotionMagicCruiseVelocity = 3.0;
        motorConfig.MotionMagic.MotionMagicAcceleration = 40.0;
        motorConfig.MotionMagic.MotionMagicJerk = 100.0;
    }

    public static final CANcoderConfiguration encoderConfig = new CANcoderConfiguration();

    static {
        encoderConfig.MagnetSensor.MagnetOffset = -0.444;
        encoderConfig.MagnetSensor.SensorDirection =
                SensorDirectionValue.CounterClockwise_Positive; // ccw yaw is positive
    }
}
