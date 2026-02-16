import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private static final TalonFX deployMotor = new TalonFX(IntakeConst.DEPLOY_MOTOR_ID);
    private static final TalonFX rollerMotor = new TalonFX(IntakeConst.ROLLER_MOTOR_ID);

    public IntakeSubsystem() {
        deployMotor.getConfigurator().apply(IntakeConfig.deployMotorConfig);
        rollerMotor.getConfigurator().apply(IntakeConfig.rollerMotorConfig);
    }

    public void rollersOn() {
        rollerMotor.set(IntakeConfig.ROLLER_SPEED);
    }

    public void rollersOff() {
        rollerMotor.stopMotor();
    }

    public void intakeDown() {
        moveIntake(IntakeConfig.DOWN_ANGLE);
    }

    public void intakeUp() {
        moveIntake(IntakeConfig.UP_ANGLE);
    }

    private void moveIntake(Angle angle) {
        double clampedAngle =
                MathUtil.clamp(
                        angle.in(Rotations),
                        IntakeConst.MIN_ANGLE.in(Rotations),
                        IntakeConst.MAX_ANGLE.in(Rotations));
        deployMotor.setControl(new MotionMagicVoltage(clampedAngle));
    }
}
