package frc.robot.launcher.shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    private final TalonFX rightLeaderMotor = new TalonFX(ShooterConst.RIGHT_LEADER_MOTOR_ID);
    private final TalonFX leftFollowerMotor = new TalonFX(ShooterConst.LEFT_FOLLOWER_MOTOR_ID);

    public ShooterSubsystem() {
        rightLeaderMotor.getConfigurator().apply(ShooterConfig.motorConfig);
        leftFollowerMotor.getConfigurator().apply(ShooterConfig.motorConfig);
        // Set the follower motor to follow the leader motor output but in the opposite direction
        leftFollowerMotor.setControl(
                new Follower(rightLeaderMotor.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    public void setAngularVelocity(AngularVelocity angularVelocity) {
        rightLeaderMotor.setControl(new MotionMagicVelocityVoltage(angularVelocity));
    }

    public AngularVelocity getAngularVelocity() {
        return rightLeaderMotor.getVelocity().getValue();
    }

    public void stop() {
        rightLeaderMotor.stopMotor();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty(
                "speed (RPS)",
                () -> getAngularVelocity().in(RotationsPerSecond),
                (double speed) -> setAngularVelocity(RotationsPerSecond.of(speed)));
    }
}
