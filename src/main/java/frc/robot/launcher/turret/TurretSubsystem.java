package frc.robot.launcher.turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Note: yaw of 0 is facing forward
public class TurretSubsystem extends SubsystemBase {
    private final TalonFX motor = new TalonFX(TurretConst.MOTOR_ID);
    private final CANcoder encoder = new CANcoder(TurretConst.ENCODER_ID);

    private Angle targetYaw;

    public TurretSubsystem() {
        motor.getConfigurator().apply(TurretConfig.motorConfig);
        encoder.getConfigurator().apply(TurretConfig.encoderConfig);

        // Conventional startup yaw
        targetYaw = TurretConfig.STOW_YAW;
    }

    public void moveYaw(Angle yaw) {
        targetYaw =
                Rotations.of(
                        MathUtil.clamp(
                                yaw.in(Rotations),
                                TurretConst.MIN_ANGLE.in(Rotations),
                                TurretConst.MAX_ANGLE.in(Rotations)));
        motor.setControl(new MotionMagicVoltage(targetYaw));
    }

    public Angle getYaw() {
        return encoder.getPosition().getValue().div(TurretConst.ENCODER_TO_MECHANISM_RATIO);
    }

    public void stow() {
        moveYaw(TurretConfig.STOW_YAW);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("yaw (deg)", () -> getYaw().in(Degrees), null);
        builder.addDoubleProperty(
                "target yaw (deg)", () -> targetYaw.in(Degrees), (yaw) -> moveYaw(Degrees.of(yaw)));
        builder.addDoubleProperty(
                "yaw error (deg)", () -> getYaw().minus(targetYaw).in(Degrees), null);
    }
}
