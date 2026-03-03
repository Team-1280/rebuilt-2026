package frc.robot.launcher.hood;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Note that pitch refers to fuel launch pitch relative to robot
public class HoodSubsystem extends SubsystemBase {
    private static final TalonFX motor = new TalonFX(HoodConst.MOTOR_ID);

    private static Angle targetPitch;

    public HoodSubsystem() {
        motor.getConfigurator().apply(HoodConfig.motorConfig);

        // Hardstop startup angle
        motor.setPosition(HoodConst.MAX_PITCH);
        targetPitch = HoodConst.MAX_PITCH;
    }

    public void movePitch(Angle pitch) {
        double clampedPitch =
                MathUtil.clamp(
                        pitch.in(Rotations),
                        HoodConst.MIN_PITCH.in(Rotations),
                        HoodConst.MAX_PITCH.in(Rotations));
        motor.setControl(new MotionMagicVoltage(clampedPitch));
    }

    public Angle getPitch() {
        return motor.getPosition().getValue();
    }

    public void stow() {
        movePitch(HoodConst.MAX_PITCH);
    }

    @Override
    public void periodic() {
        if (getPitch().in(Degrees) < HoodConfig.SAFETY_ANGLE_LIMIT.in(Degrees)) {
            motor.setControl(new StaticBrake());
            DriverStation.reportWarning(
                    "hood motor at "
                            + getPitch().in(Degrees)
                            + " degrees disabled by safety angle limit",
                    false);
        }
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
