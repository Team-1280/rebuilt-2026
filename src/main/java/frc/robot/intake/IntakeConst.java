package frc.robot.intake;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;

public final class IntakeConst {
    public static final int DEPLOY_MOTOR_ID = -1; // TODO
    public static final int ROLLER_MOTOR_ID = -1; // TODO

    public static final Angle MIN_ANGLE = Degrees.of(0.00);
    public static final Angle MAX_ANGLE = Degrees.of(128.261834);

    public static final double DEPLOY_ROTOR_TO_MECHANISM_RATIO = 45.0;
}
