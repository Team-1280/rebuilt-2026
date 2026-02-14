package frc.robot.advkit;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

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
        }

        @Override
        public void fromLog(LogTable table) {
            connected = table.get("Connected", connected);
            yawPosition = table.get("YawPosition", yawPosition);
            yawVelocityRadPerSec = table.get("YawVelocityRadPerSec", yawVelocityRadPerSec);
            odometryYawTimestamps = table.get("OdometryYawTimestamps", odometryYawTimestamps);
            odometryYawPositions = table.get("OdometryYawPositions", odometryYawPositions);
        }

        public GyroIOInputsAutoLogged clone() {
            GyroIOInputsAutoLogged copy = new GyroIOInputsAutoLogged();
            copy.connected = this.connected;
            copy.yawPosition = this.yawPosition;
            copy.yawVelocityRadPerSec = this.yawVelocityRadPerSec;
            copy.odometryYawTimestamps = this.odometryYawTimestamps.clone();
            copy.odometryYawPositions = this.odometryYawPositions.clone();
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
        }
    }
}
