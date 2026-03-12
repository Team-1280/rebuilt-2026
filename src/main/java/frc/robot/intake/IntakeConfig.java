package frc.robot.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public final class IntakeConfig {
    public static final double ROLLER_SPEED = 0.5;

    /** Maximum error magnitude at which the deploy motor brakes to lock the intake in place. */
    public static final Angle ANGLE_LOCK_TOLERANCE = Degrees.of(2.0); // TODO: tune

    /** Minimum error magnitude at which the deploy motor stops braking and applies correction. */
    public static final Angle ANGLE_UNLOCK_TOLERANCE = Degrees.of(4.0); // TODO: tune

    /** Feedforward that is applied with the sign of the angle error to help correct small error. */
    public static final Voltage ANGLE_ERROR_SIGN_FEEDFORWARD = Volts.of(0.2);

    public static final TalonFXConfiguration deployMotorConfig = new TalonFXConfiguration();
    public static final Current DEPLOY_STATOR_LIMIT = Amps.of(20);
    public static final Current DEPLOY_SUPPLY_LIMIT = DEPLOY_STATOR_LIMIT;

    static {
        deployMotorConfig.CurrentLimits.StatorCurrentLimit = DEPLOY_STATOR_LIMIT.in(Amps);
        deployMotorConfig.CurrentLimits.SupplyCurrentLimit = DEPLOY_SUPPLY_LIMIT.in(Amps);
        deployMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        deployMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // up positive
        deployMotorConfig.Feedback.SensorToMechanismRatio =
                IntakeConst.DEPLOY_ROTOR_TO_MECHANISM_RATIO;

        // Control unit: mechanism rotations
        deployMotorConfig.Slot0.kP = 50.0;
        deployMotorConfig.Slot0.kD = 0.0;
        deployMotorConfig.Slot0.kS = 0.05;
        deployMotorConfig.Slot0.kV = 10.0;
        deployMotorConfig.Slot0.kA = 0.0;
        deployMotorConfig.Slot0.kG = 0.4;
        deployMotorConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        deployMotorConfig.Slot0.GravityArmPositionOffset = 0.037;
        deployMotorConfig.Slot0.StaticFeedforwardSign =
                StaticFeedforwardSignValue.UseClosedLoopSign;
        deployMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 1.0;
        deployMotorConfig.MotionMagic.MotionMagicAcceleration = 4.0;
        deployMotorConfig.MotionMagic.MotionMagicJerk = 20.0;
    }

    public static final TalonFXConfiguration rollerMotorConfig = new TalonFXConfiguration();
    public static final Current ROLLER_STATOR_LIMIT = Amps.of(80);
    public static final Current ROLLER_SUPPLY_LIMIT = Amps.of(40);

    static {
        rollerMotorConfig.CurrentLimits.StatorCurrentLimit = ROLLER_STATOR_LIMIT.in(Amps);
        rollerMotorConfig.CurrentLimits.SupplyCurrentLimit = ROLLER_SUPPLY_LIMIT.in(Amps);
        rollerMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        rollerMotorConfig.MotorOutput.Inverted =
                InvertedValue.Clockwise_Positive; // intake positive
    }
}
