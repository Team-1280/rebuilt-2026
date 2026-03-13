package frc.robot.advkit;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.studica.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

import frc.robot.drivetrain.CommandSwerveIO;
import frc.robot.drivetrain.TunerConstants;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import java.util.Queue;

// a way to interface the Gyro
public interface GyroIO {
    @AutoLog
    public static class GyroIOInputs {
        public boolean connected = false;
        public Rotation2d yawPosition = Rotation2d.kZero;
        public double yawVelocityRadPerSec = 0.0;
        public double[] odometryYawTimestamps = new double[] {};
        public Rotation2d[] odometryYawPositions = new Rotation2d[] {};

        // Gyro 3 (NavX2 MXP)
        public boolean gyro3Connected = false;
        public Rotation2d gyro3YawPosition = Rotation2d.kZero;
        public double gyro3YawVelocityRadPerSec = 0.0;
        public double[] gyro3OdometryYawTimestamps = new double[] {};
        public Rotation2d[] gyro3OdometryYawPositions = new Rotation2d[] {};
    }

    public default void updateInputs(GyroIOInputs inputs) {}

    public class GyroIOInputsAutoLogged extends GyroIO.GyroIOInputs
            implements LoggableInputs, Cloneable {
        @Override
        public void toLog(LogTable table) {
            table.put("Connected", connected);
            table.put("YawPosition", yawPosition);
            table.put("YawVelocityRadPerSec", yawVelocityRadPerSec);
            table.put("OdometryYawTimestamps", odometryYawTimestamps);
            table.put("OdometryYawPositions", odometryYawPositions);
            table.put("Gyro3/Connected", gyro3Connected);
            table.put("Gyro3/YawPosition", gyro3YawPosition);
            table.put("Gyro3/YawVelocityRadPerSec", gyro3YawVelocityRadPerSec);
            table.put("Gyro3/OdometryYawTimestamps", gyro3OdometryYawTimestamps);
            table.put("Gyro3/OdometryYawPositions", gyro3OdometryYawPositions);
        }

        @Override
        public void fromLog(LogTable table) {
            connected = table.get("Connected", connected);
            yawPosition = table.get("YawPosition", yawPosition);
            yawVelocityRadPerSec = table.get("YawVelocityRadPerSec", yawVelocityRadPerSec);
            odometryYawTimestamps = table.get("OdometryYawTimestamps", odometryYawTimestamps);
            odometryYawPositions = table.get("OdometryYawPositions", odometryYawPositions);
            gyro3Connected = table.get("Gyro3/Connected", gyro3Connected);
            gyro3YawPosition = table.get("Gyro3/YawPosition", gyro3YawPosition);
            gyro3YawVelocityRadPerSec =
                    table.get("Gyro3/YawVelocityRadPerSec", gyro3YawVelocityRadPerSec);
            gyro3OdometryYawTimestamps =
                    table.get("Gyro3/OdometryYawTimestamps", gyro3OdometryYawTimestamps);
            gyro3OdometryYawPositions =
                    table.get("Gyro3/OdometryYawPositions", gyro3OdometryYawPositions);
        }

        public GyroIOInputsAutoLogged clone() {
            GyroIOInputsAutoLogged copy = new GyroIOInputsAutoLogged();
            copy.connected = this.connected;
            copy.yawPosition = this.yawPosition;
            copy.yawVelocityRadPerSec = this.yawVelocityRadPerSec;
            copy.odometryYawTimestamps = this.odometryYawTimestamps.clone();
            copy.odometryYawPositions = this.odometryYawPositions.clone();
            copy.gyro3Connected = this.gyro3Connected;
            copy.gyro3YawPosition = this.gyro3YawPosition;
            copy.gyro3YawVelocityRadPerSec = this.gyro3YawVelocityRadPerSec;
            copy.gyro3OdometryYawTimestamps = this.gyro3OdometryYawTimestamps.clone();
            copy.gyro3OdometryYawPositions = this.gyro3OdometryYawPositions.clone();
            return copy;
        }
    }

    // implements using our own gyro: ctre pigeon2.0
    public class GyroIOPigeon2 implements GyroIO {
        private final Pigeon2 pigeon =
                new Pigeon2(TunerConstants.DrivetrainConstants.Pigeon2Id, TunerConstants.kCANBus);
        private final StatusSignal<Angle> yaw = pigeon.getYaw();
        private final Queue<Double> yawPositionQueue;
        private final Queue<Double> yawTimestampQueue;
        private final StatusSignal<AngularVelocity> yawVelocity = pigeon.getAngularVelocityZWorld();

        // Gyro 3: NavX2 connected via USB
        private final AHRS navX2 = new AHRS(AHRS.NavXComType.kUSB1);

        public GyroIOPigeon2() {
            if (TunerConstants.DrivetrainConstants.Pigeon2Configs != null) {
                pigeon.getConfigurator().apply(TunerConstants.DrivetrainConstants.Pigeon2Configs);
            } else {
                pigeon.getConfigurator().apply(new Pigeon2Configuration());
            }
            pigeon.getConfigurator().setYaw(0.0);
            yaw.setUpdateFrequency(CommandSwerveIO.ODOMETRY_FREQ);
            yawVelocity.setUpdateFrequency(50.0);
            pigeon.optimizeBusUtilization();
            yawTimestampQueue = PhoenixOdometry.getInstance().makeTimestampQueue();
            yawPositionQueue = PhoenixOdometry.getInstance().registerSignal(yaw.clone());
        }

        @Override
        public void updateInputs(GyroIOInputs inputs) {
            inputs.connected = BaseStatusSignal.refreshAll(yaw, yawVelocity).equals(StatusCode.OK);
            inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
            inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());
            inputs.odometryYawTimestamps =
                    yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
            inputs.odometryYawPositions =
                    yawPositionQueue.stream()
                            .map((Double value) -> Rotation2d.fromDegrees(value))
                            .toArray(Rotation2d[]::new);
            yawTimestampQueue.clear();
            yawPositionQueue.clear();

            // NavX2 does not support high-frequency Phoenix-style signal queuing;
            // yaw timestamps/positions arrays are left empty and not used for odometry.
            inputs.gyro3Connected = navX2.isConnected();
            inputs.gyro3YawPosition = navX2.getRotation2d();
            inputs.gyro3YawVelocityRadPerSec = Units.degreesToRadians(navX2.getRate());
        }
    }
}
