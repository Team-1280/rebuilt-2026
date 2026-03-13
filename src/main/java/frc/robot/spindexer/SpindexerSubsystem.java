package frc.robot.spindexer;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SpindexerSubsystem extends SubsystemBase {
    private final TalonFX motor = new TalonFX(SpindexerConst.MOTOR_ID);

    public SpindexerSubsystem() {
        motor.getConfigurator().apply(SpindexerConfig.motorConfig);
    }

    private void moveMotorSpeed(double speed) {
        motor.set(speed);
    }

    private double getMotorSpeed() {
        return motor.get();
    }

    public void start() {
        moveMotorSpeed(SpindexerConfig.MOTOR_SPEED);
    }

    public void stop() {
        moveMotorSpeed(0.0);
    }

    public void reverse() {
        moveMotorSpeed(SpindexerConfig.REVERSE_MOTOR_SPEED);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("motor speed (frac)", this::getMotorSpeed, this::moveMotorSpeed);
    }
}
