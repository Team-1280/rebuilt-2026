package frc.robot.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private static final TalonFX deployMotor = new TalonFX(IntakeConst.DEPLOY_MOTOR_ID);
    private static final TalonFX rollerMotor = new TalonFX(IntakeConst.ROLLER_MOTOR_ID);

    private Angle targetAngle;

    public IntakeSubsystem() {
        deployMotor.getConfigurator().apply(IntakeConfig.deployMotorConfig);
        rollerMotor.getConfigurator().apply(IntakeConfig.rollerMotorConfig);

        // Hardstop startup angle
        deployMotor.setPosition(IntakeConst.MAX_ANGLE);
        targetAngle = IntakeConst.MAX_ANGLE;
    }

    public Angle getAngle() {
        return deployMotor.getPosition().getValue();
    }

    public void intakeDown() {
        moveAngle(IntakeConst.MIN_ANGLE);
    }

    public void intakeUp() {
        moveAngle(IntakeConst.MAX_ANGLE);
    }

    private void moveAngle(Angle angle) {
        targetAngle =
                Rotations.of(
                        MathUtil.clamp(
                                angle.in(Rotations),
                                IntakeConst.MIN_ANGLE.in(Rotations),
                                IntakeConst.MAX_ANGLE.in(Rotations)));
        deployMotor.setControl(new MotionMagicVoltage(targetAngle));
    }

    private void moveRollerSpeed(double speed) {
        rollerMotor.set(speed);
    }

    private double getRollerSpeed() {
        return rollerMotor.get();
    }

    public void rollersOn() {
        moveRollerSpeed(IntakeConfig.ROLLER_SPEED);
    }

    public void rollersOff() {
        moveRollerSpeed(0.0);
    }

    public void stow() {
        rollersOff();
        moveAngle(IntakeConst.MAX_ANGLE);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("angle (deg)", () -> getAngle().in(Degrees), null);
        builder.addDoubleProperty(
                "target angle (deg)",
                () -> targetAngle.in(Degrees),
                (intakeAngle) -> moveAngle(Degrees.of(intakeAngle)));
        builder.addDoubleProperty(
                "angle error (deg)", () -> getAngle().minus(targetAngle).in(Degrees), null);

        builder.addDoubleProperty(
                "roller speed (frac)", this::getRollerSpeed, this::moveRollerSpeed);
    }
}
