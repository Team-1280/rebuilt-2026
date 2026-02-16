package frc.robot.launcher.feeder;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FeederSubsystem extends SubsystemBase {
    private final TalonFX motor = new TalonFX(FeederConst.MOTOR_ID);

    private AngularVelocity targetAngularVelocity = RotationsPerSecond.of(0.0);

    public FeederSubsystem() {
        motor.getConfigurator().apply(FeederConfig.motorConfig);
    }

    public AngularVelocity getAngularVelocity() {
        return motor.getVelocity().getValue();
    }

    public void moveAngularVelocity(AngularVelocity angularVelocity) {
        targetAngularVelocity = angularVelocity;
        motor.setControl(new MotionMagicVelocityVoltage(targetAngularVelocity));
    }

    public void start() {
        moveAngularVelocity(FeederConfig.ANGULAR_VELOCITY);
    }

    public void stop() {
        motor.stopMotor();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty(
                "angular velocity (RPS)", () -> getAngularVelocity().in(RotationsPerSecond), null);
        builder.addDoubleProperty(
                "target angular velocity (RPS)",
                () -> targetAngularVelocity.in(RotationsPerSecond),
                (double angularVelocity) ->
                        moveAngularVelocity(RotationsPerSecond.of(angularVelocity)));
        builder.addDoubleProperty(
                "angular velocity error (RPS)",
                () -> getAngularVelocity().minus(targetAngularVelocity).in(RotationsPerSecond),
                null);
    }
}
