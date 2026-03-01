package frc.robot.advkit;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberMotorIO {
    @AutoLog
    public static class ClimberMotorIOInputs {
        public boolean connected = false;
        public double positionRad = 0.0;
        public double velocityRadPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double supplyVoltage = 0.0;
        public double supplyCurrentAmps = 0.0;
        public double statorCurrentAmps = 0.0;
        public double deviceTempCelsius = 0.0;
        public double processorTempCelsius = 0.0;
        public boolean stalled = false;
    }

    public default void updateInputs(ClimberMotorIOInputs inputs) {}

    public default void setOpenLoop(double output) {}

    public default void setTargetPosition(double positionRad) {}

    public default void stop() {}
}
