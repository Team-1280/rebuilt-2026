import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase{
    private static final TalonFX deployMotor = new TalonFX(IntakeConst.DEPLOY_MOTOR_ID);
    private static final TalonFX intakeMotor = new TalonFX(IntakeConst.INTAKE_MOTOR_ID);

    public IntakeSubsystem(){
        deployMotor.getConfigurator().apply(IntakeConfig.deployConfig);
        intakeMotor.getConfigurator().apply(IntakeConfig.intakeConfig);
    }

    public void intakeOn() {
        intakeMotor.set(IntakeConfig.INTAKE_SPEED);
    }

    public void intakeOff() {
        intakeMotor.set(0);
    }
    
    public void intakeDown() {
        moveIntake(IntakeConfig.DOWN_ANGLE);
    }

    public void intakeUp() {
        moveIntake(IntakeConfig.UP_ANGLE);
    }

    private void moveIntake(double angle) {
        double clampedAngle = MathUtil.clamp(angle, IntakeConst.MIN_ANGLE, IntakeConst.MAX_ANGLE);
        deployMotor.setControl(new MotionMagicVoltage(clampedAngle));
    }

}
