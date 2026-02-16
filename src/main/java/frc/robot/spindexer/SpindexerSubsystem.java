package frc.robot.spindexer;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SpindexerSubsystem extends SubsystemBase {
    private final TalonFX motor = new TalonFX(SpindexerConst.MOTOR_ID);

    public SpindexerSubsystem() {
        motor.getConfigurator().apply(SpindexerConfig.motorConfig);
    }

    private void moveSpeed(double speed) {
        motor.set(speed);
    }

    private double getSpeed() {
        return motor.get();
    }

    public void start() {
        moveSpeed(SpindexerConfig.MOTOR_SPEED);
    }

    public void stop() {
        moveSpeed(0.0);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("speed (frac)", this::getSpeed, this::moveSpeed);
    }
}
