package frc.robot.spindexer;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SpindexerSubsystem extends SubsystemBase {
    TalonFX motor = new TalonFX(SpindexerConst.MOTOR_ID);

    public SpindexerSubsystem() {}

    public AngularVelocity getSpeed() {
        return motor.getVelocity().getValue();
    }

    public void setSpeed(AngularVelocity angularVelocity) {
        motor.setControl(new MotionMagicVelocityVoltage(angularVelocity));
    }

    public void stop() {
        motor.stopMotor();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty(
                "Spindexer speed(RPS)",
                () -> getSpeed().in(RotationsPerSecond),
                (double speed) -> setSpeed(RotationsPerSecond.of(speed)));
    }
}
