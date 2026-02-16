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

public class TurretSubsystem extends SubsystemBase {
    private final TalonFX motor = new TalonFX(TurretConst.MOTOR_ID);
    private final CANcoder encoder = new CANcoder(TurretConst.ENCODER_ID);

    public TurretSubsystem() {
        motor.getConfigurator().apply(TurretConfig.motorConfig);
        encoder.getConfigurator().apply(TurretConfig.encoderConfig);
    }

    public void moveYaw(Angle yaw) {
        // Note: yaw of 0 is facing forward
        double clampedYaw =
                MathUtil.clamp(
                        yaw.in(Rotations),
                        TurretConst.MIN_ANGLE.in(Rotations),
                        TurretConst.MAX_ANGLE.in(Rotations));
        motor.setControl(new MotionMagicVoltage(clampedYaw));
    }

    public Angle getYaw() {
        return encoder.getPosition().getValue().div(TurretConst.ENCODER_TO_MECHANISM_RATIO);
    }

    public void stow() {
        moveYaw(TurretConfig.STOW_YAW);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty(
                "yaw (deg)", () -> getYaw().in(Degrees), (yaw) -> moveYaw(Degrees.of(yaw)));
    }
}
