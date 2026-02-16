package frc.robot.advkit;

import org.littletonrobotics.junction.AutoLog;

public interface RotationMotorIO {
    @AutoLog
    public static class RotationMotorIOInputs {
        public boolean connected = false;
        public double positionRad = 0.0;
        public double velocityRadPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
        public double tempCelsius = 0.0;
        public boolean forwardLimitReached = false;
        public boolean reverseLimitReached = false;
    }

    public default void updateInputs(RotationMotorIOInputs inputs) {}

    public default void setOpenLoop(double output) {}

    public default void setTargetPosition(double positionRad) {}

    public default void stop() {}
}
