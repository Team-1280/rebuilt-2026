package frc.robot.launcher.feeder;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FeederSubsystem extends SubsystemBase {
    private final TalonFX motor = new TalonFX(FeederConst.MOTOR_ID, FeederConst.CAN_BUS);

    public FeederSubsystem() {
        motor.getConfigurator().apply(FeederConfig.motorConfig);
    }

    public double getMotorSpeed() {
        return motor.get();
    }

    private void moveMotorSpeed(double speed) {
        motor.set(speed);
    }

    public void start() {
        moveMotorSpeed(FeederConfig.MOTOR_SPEED);
    }

    public void stop() {
        motor.stopMotor();
    }

    public void reverse() {
        moveMotorSpeed(FeederConfig.REVERSE_MOTOR_SPEED);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("motor speed (frac)", this::getMotorSpeed, this::moveMotorSpeed);
    }
}
