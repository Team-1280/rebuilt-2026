package frc.robot.advkit;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelMotorIO {
    @AutoLog
    public static class FlywheelMotorIOInputs {
        public boolean connected = false;
        public double velocityRPM = 0.0;
        public double targetVelocityRPM = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
        public double tempCelsius = 0.0;
        public boolean atTargetVelocity = false;
    }

    public default void updateInputs(FlywheelMotorIOInputs inputs) {}

    public default void setOpenLoop(double output) {}

    public default void setTargetVelocity(double velocityRPM) {}

    public default void stop() {}
}
