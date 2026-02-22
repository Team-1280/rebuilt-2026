package frc.robot.launcher.shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    private final TalonFX rightLeaderMotor = new TalonFX(ShooterConst.RIGHT_LEADER_MOTOR_ID);
    private final TalonFX leftFollowerMotor = new TalonFX(ShooterConst.LEFT_FOLLOWER_MOTOR_ID);

    private AngularVelocity targetAngularVelocity = RotationsPerSecond.of(0.0);

    public ShooterSubsystem() {
        rightLeaderMotor.getConfigurator().apply(ShooterConfig.motorConfig);
        leftFollowerMotor.getConfigurator().apply(ShooterConfig.motorConfig);
        // Set the follower motor to follow the leader motor output but in the opposite direction
        leftFollowerMotor.setControl(
                new Follower(rightLeaderMotor.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    public void moveAngularVelocity(AngularVelocity angularVelocity) {
        targetAngularVelocity =
                RotationsPerSecond.of(
                        MathUtil.clamp(
                                angularVelocity.in(RotationsPerSecond),
                                -ShooterConfig.MAX_ANGULAR_VELOCITY.in(RotationsPerSecond),
                                ShooterConfig.MAX_ANGULAR_VELOCITY.in(RotationsPerSecond)));
        rightLeaderMotor.setControl(new MotionMagicVelocityVoltage(targetAngularVelocity));
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
