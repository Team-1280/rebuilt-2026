package frc.robot.launcher.feeder;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FeederSubsystem extends SubsystemBase {
    private final TalonFX motor = new TalonFX(FeederConst.MOTOR_ID);

    public FeederSubsystem() {
        motor.getConfigurator().apply(FeederConfig.motorConfig);
    }

    public AngularVelocity getSpeed() {
        return motor.getVelocity().getValue();
    }

    public void setSpeed(AngularVelocity speed) {
        motor.setControl(new MotionMagicVelocityVoltage(speed));
    }

    public void startSpinning() {
        setSpeed(FeederConfig.MOTOR_SPEED);
    }

    public void stopSpinning() {
        motor.stopMotor();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty(
                "Feeder speed (RPS)",
                () -> getSpeed().in(RotationsPerSecond),
                (double speed) -> setSpeed(RotationsPerSecond.of(speed)));
    }
}
