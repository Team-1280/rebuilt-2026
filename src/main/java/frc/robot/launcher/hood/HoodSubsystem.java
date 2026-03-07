package frc.robot.launcher.hood;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.launcher.LauncherConst;

// Note that pitch refers to fuel launch pitch relative to robot
public class HoodSubsystem extends SubsystemBase {
    private static final TalonFX motor = new TalonFX(HoodConst.MOTOR_ID, LauncherConst.CAN_BUS);

    private static Angle targetPitch;

    public HoodSubsystem() {
        motor.getConfigurator().apply(HoodConfig.motorConfig);

        // Hardstop startup angle
        motor.setPosition(HoodConst.MAX_PITCH);
        targetPitch = HoodConst.MAX_PITCH;
    }

    public void movePitch(Angle pitch) {
        targetPitch =
                Rotations.of(
                        MathUtil.clamp(
                                pitch.in(Rotations),
                                HoodConst.MIN_PITCH.in(Rotations),
                                HoodConst.MAX_PITCH.in(Rotations)));
        motor.setControl(new MotionMagicVoltage(targetPitch));
    }

    public Angle getPitch() {
        return motor.getPosition().getValue();
    }

    public void stow() {
        movePitch(HoodConst.MAX_PITCH);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("pitch (deg)", () -> getPitch().in(Degrees), null);
        builder.addDoubleProperty(
                "target pitch (deg)",
                () -> targetPitch.in(Degrees),
                (pitch) -> movePitch(Degrees.of(pitch)));
        builder.addDoubleProperty(
                "pitch error (deg)", () -> targetPitch.minus(getPitch()).in(Degrees), null);
    }
}
