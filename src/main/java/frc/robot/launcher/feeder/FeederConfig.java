package frc.robot.launcher.feeder;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.units.measure.AngularVelocity;

public final class FeederConfig {
    public static final TalonFXConfiguration motorConfig = new TalonFXConfiguration();

    public static final AngularVelocity MOTOR_SPEED = RotationsPerSecond.of(-1); // TODO
}
