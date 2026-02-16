package frc.robot.intake;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.units.measure.Angle;

public class IntakeConfig {
    public static final double ROLLER_SPEED = 0; // TODO

    public static final Angle DOWN_ANGLE = Degrees.of(0); // TODO
    public static final Angle UP_ANGLE = Degrees.of(0); // TODO

    public static final TalonFXConfiguration deployMotorConfig = new TalonFXConfiguration(); // TODO
    public static final TalonFXConfiguration rollerMotorConfig = new TalonFXConfiguration(); // TODO
}
