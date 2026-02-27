package frc.robot.launcher.turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Note: yaw of 0 is facing forward
public class TurretSubsystem extends SubsystemBase {
    private final TalonFX motor = new TalonFX(TurretConst.MOTOR_ID);
    private final CANcoder encoder = new CANcoder(TurretConst.ENCODER_ID);

    /** Continuous desired turret yaw, raw. */
    private Angle targetYaw;

    public TurretSubsystem() {
        motor.getConfigurator().apply(TurretConfig.motorConfig);
        encoder.getConfigurator().apply(TurretConfig.encoderConfig);

        // Conventional or expected startup yaw
        calibrateYaw(TurretConfig.STOW_YAW);
        targetYaw = TurretConfig.STOW_YAW; // Note: this may be inaccurate until the turret is moved
    }

    /**
     * Calibrate the encoder of the turret using a guess of the current yaw.
     *
     * <p>This is necessary because the encoder has many rotations per mechanism rotation, but it
     * only tracks the position within one rotation on boot, so it may be off by a whole number of
     * rotations on startup. Only the phase of the encoder is always correct.
     *
     * <p>To calibrate the encoder, the method finds the coterminal (off by whole rotations, so some
     * phase) encoder position the closest to the guess, and then sets the encoder position to that.
     *
     * <p>For the calibration to be correct, the guess yaw must be within (1 rotation / (2 *
     * ENCODER_TO_MECHANISM_RATIO)) of the actual yaw, on either side. Guessing outside of this
     * range will cause the measured turret yaw to be off by a multiple of (1 rotation /
     * ENCODER_TO_MECHANISM_RATIO).
     *
     * <p>For an ENCODER_TO_MECHANISM_RATIO of 8.5, this maximum error is 1/17 of a rotation, or
     * about 21.18 degrees (giving a total window of about 42.35 degrees).
     *
     * @param guessYaw a guess of the current turret yaw, used to calibrate the encoder
     */
    private void calibrateYaw(Angle guessYaw) {
        // Check if the guessed yaw is obviously outside of the valid turret yaw range and invalid
        if (guessYaw.minus(TurretConst.MIN_ANGLE).in(Rotations) < -0.25
                || guessYaw.minus(TurretConst.MAX_ANGLE).in(Rotations) > 0.25) {
            DriverStation.reportWarning(
                    "Turret calibration failed due to invalid out-of-bounds guess yaw", false);
            return;
        }
        // Variable values are in encoder rotations
        // Get the encoder position in rotations
        double encoderPosition = encoder.getPosition().getValueAsDouble();
        // Convert the guess yaw to encoder rotations
        double guessPosition = guessYaw.in(Rotations) * TurretConst.ENCODER_TO_MECHANISM_RATIO;
        // Find the closest whole number of rotations, relative to the encoder phase
        double roundedGuessOffset = Math.round(guessPosition - encoderPosition);
        // Clamp the guess offset to make sure the calibrated guess position is in the turret bounds
        double minOffset =
                TurretConst.MIN_ANGLE.in(Rotations) * TurretConst.ENCODER_TO_MECHANISM_RATIO
                        - guessPosition;
        double maxOffset =
                TurretConst.MAX_ANGLE.in(Rotations) * TurretConst.ENCODER_TO_MECHANISM_RATIO
                        - guessPosition;
        double clampedGuessOffset =
                MathUtil.clamp(roundedGuessOffset, Math.ceil(minOffset), Math.floor(maxOffset));
        // Convert back to absolute, to get the coterminal encoder position closest to the guess
        double calibratedEncoderPosition = clampedGuessOffset + encoderPosition;
        encoder.setPosition(calibratedEncoderPosition);
    }

    /** Move the turret to the best position pointing in the same direction as yaw. */
    public void moveYaw(Angle yaw) {
        // Get current yaw in rotations
        double currentYaw = getYaw().in(Rotations);
        // Get desired yaw in rotations. We will find the best coterminal angle of it.
        double desiredYaw = yaw.in(Rotations);
        // Find a whole number of rotations to offset the desired yaw by, to be as close to the
        // current yaw as possible, by rounding their difference (current relative to target)
        double yawOffset = Math.round(currentYaw - desiredYaw);
        // Calculate the lower and upper bounds on the offset to prevent the yaw from being invalid
        double minOffset = Math.ceil(TurretConst.MIN_ANGLE.in(Rotations) - desiredYaw);
        double maxOffset = Math.floor(TurretConst.MAX_ANGLE.in(Rotations) - desiredYaw);
        // Clamp the offset between the minimum and maximum offsets
        double clampedYawOffset = MathUtil.clamp(yawOffset, minOffset, maxOffset);
        // Add this offset to the desired yaw to find the best target yaw
        double rawYaw = desiredYaw + clampedYawOffset;
        moveRawYaw(Rotations.of(rawYaw));
    }

    /** Move the turret to the exact position, ignoring coterminal positions. */
    private void moveRawYaw(Angle yaw) {
        targetYaw =
                Rotations.of(
                        MathUtil.clamp(
                                yaw.in(Rotations),
                                TurretConst.MIN_ANGLE.in(Rotations),
                                TurretConst.MAX_ANGLE.in(Rotations)));
        motor.setControl(new MotionMagicVoltage(targetYaw));
    }

    /** Get the continuous yaw. */
    public Angle getYaw() {
        // Note: this call already uses the remote CANcoder and accounts for the mechanism ratio
        return motor.getPosition().getValue();
    }

    public void stow() {
        moveRawYaw(TurretConfig.STOW_YAW);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        // Use this dashboard property setter to calibrate yaw if necessary after startup
        builder.addDoubleProperty(
                "YAW (deg)",
                () -> getYaw().in(Degrees),
                (guessYaw) -> calibrateYaw(Degrees.of(guessYaw)));
        builder.addDoubleProperty(
                "target yaw, raw (deg)",
                () -> targetYaw.in(Degrees),
                (yaw) -> moveRawYaw(Degrees.of(yaw)));
        builder.addDoubleProperty(
                "yaw error (deg)", () -> getYaw().minus(targetYaw).in(Degrees), null);
    }
}
