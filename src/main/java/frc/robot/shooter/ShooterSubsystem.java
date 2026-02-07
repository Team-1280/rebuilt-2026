package frc.robot.shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    private final TalonFX leaderShooterMotor = new TalonFX(ShooterConst.RIGHT_MOTOR_ID);
    private final TalonFX followerShooterMotor = new TalonFX(ShooterConst.LEFT_MOTOR_ID);

    public ShooterSubsystem() {
        leaderShooterMotor.getConfigurator().apply(ShooterConst.shooterMotorConfig);
        followerShooterMotor.getConfigurator().apply(ShooterConst.shooterMotorConfig);
        followerShooterMotor.setControl(new Follower(leaderShooterMotor.getDeviceID(), MotorAlignmentValue.Opposed)); // left motor is opposite orientation of leader; flip rotation direction
    }

    public void setAngularVelocity(AngularVelocity angularVelocity) {
        leaderShooterMotor.setControl(new MotionMagicVelocityVoltage(angularVelocity));
    }

    public AngularVelocity getAngularVelocity() {
        return leaderShooterMotor.getVelocity().getValue();
    }

    public void stop() {
        leaderShooterMotor.stopMotor();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("speed (RPS)", () -> getAngularVelocity().in(RotationsPerSecond), (double speed) -> setAngularVelocity(RotationsPerSecond.of(speed)));
  }
}