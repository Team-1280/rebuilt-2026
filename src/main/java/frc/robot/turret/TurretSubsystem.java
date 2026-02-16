package frc.robot.turret;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.vision.VisionConst;
import frc.robot.vision.VisionSubsystem;
import frc.robot.vision.VisionSubsystem.HubTarget;

import org.littletonrobotics.junction.Logger;

import java.util.Optional;
import java.util.function.Supplier;

public class TurretSubsystem extends SubsystemBase {

    private final TalonFX motor = new TalonFX(TurretConst.MOTOR_ID);
    private final CANcoder cancoder = new CANcoder(TurretConst.CANCODER_ID);
    private final DutyCycleOut motorOutput = new DutyCycleOut(0.0);

    private final PIDController pid =
            new PIDController(TurretConst.kP, TurretConst.kI, TurretConst.kD);

    private final Supplier<Pose2d> poseSupplier;
    private final VisionSubsystem vision;

    private int zoneIndex = TurretConst.POWER_ON_ZONE;
    private double lastCancoderPos = -1.0;
    private double turretAngleDeg = 0.0;

    private double hubX = Double.NaN;
    private double hubY = Double.NaN;
    private boolean hubCenterComputed = false;

    private boolean autoAimEnabled = true;
    private double targetAngleDeg = 0.0;

    public TurretSubsystem(Supplier<Pose2d> poseSupplier, VisionSubsystem vision) {
        this.poseSupplier = poseSupplier;
        this.vision = vision;

        pid.setTolerance(TurretConst.ON_TARGET_TOLERANCE_DEG);

        setDefaultCommand(autoAimCommand());
    }

    @Override
    public void periodic() {
        updateZoneTracking();
        computeHubCenter();

        if (autoAimEnabled) {
            autoAimPeriodic();
        }

        Logger.recordOutput("Turret/AngleDeg", turretAngleDeg);
        Logger.recordOutput("Turret/TargetAngleDeg", targetAngleDeg);
        Logger.recordOutput("Turret/AutoAimEnabled", autoAimEnabled);
        Logger.recordOutput("Turret/OnTarget", isOnTarget());
        Logger.recordOutput("Turret/Zone", zoneIndex);
        Logger.recordOutput("Turret/MotorOutput", motorOutput.Output);
        Logger.recordOutput("Turret/HubCenterComputed", hubCenterComputed);
    }

    private double readCancoderPosition() {
        double raw = cancoder.getAbsolutePosition().getValueAsDouble();
        return raw - Math.floor(raw);
    }

    private void updateZoneTracking() {
        double currentPos = readCancoderPosition();

        if (lastCancoderPos < 0) {
            lastCancoderPos = currentPos;
        } else {
            double delta = currentPos - lastCancoderPos;
            if (delta > 0.5) {
                zoneIndex--;
            } else if (delta < -0.5) {
                zoneIndex++;
            }
            lastCancoderPos = currentPos;
        }

        turretAngleDeg =
                (zoneIndex * TurretConst.DEGREES_PER_ENCODER_ROTATION)
                        + (lastCancoderPos * TurretConst.DEGREES_PER_ENCODER_ROTATION)
                        + TurretConst.MIN_ANGLE_DEG;
    }

    private void computeHubCenter() {
        if (hubCenterComputed) {
            return;
        }
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) {
            return;
        }

        int[] centerTags =
                alliance.get() == Alliance.Red
                        ? TurretConst.RED_HUB_CENTER_TAGS
                        : TurretConst.BLUE_HUB_CENTER_TAGS;

        AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(VisionConst.APRIL_TAG_FIELD);

        double sumX = 0, sumY = 0;
        int count = 0;
        for (int tagId : centerTags) {
            Optional<Pose3d> tagPose = layout.getTagPose(tagId);
            if (tagPose.isPresent()) {
                sumX += tagPose.get().getX();
                sumY += tagPose.get().getY();
                count++;
            }
        }

        if (count > 0) {
            hubX = sumX / count;
            hubY = sumY / count;
            hubCenterComputed = true;
            Logger.recordOutput("Turret/HubX", hubX);
            Logger.recordOutput("Turret/HubY", hubY);
        }
    }

    private void autoAimPeriodic() {
        if (!hubCenterComputed) {
            motor.setControl(motorOutput.withOutput(0.0));
            return;
        }

        Pose2d robotPose = poseSupplier.get();

        double fieldAngleToHub =
                Math.toDegrees(Math.atan2(hubY - robotPose.getY(), hubX - robotPose.getX()));
        double robotHeadingDeg = robotPose.getRotation().getDegrees();
        double coarseTargetDeg = fieldAngleToHub - robotHeadingDeg;

        HubTarget hubTarget = vision.getHubTarget();
        if (hubTarget.visible
                && Math.abs(hubTarget.yawDeg) < TurretConst.MAX_YAW_FOR_REFINEMENT_DEG) {
            coarseTargetDeg = turretAngleDeg + hubTarget.yawDeg;
        }

        targetAngleDeg = resolveOverlap(coarseTargetDeg);

        if (TurretConst.DEADZONE_ENABLED) {
            targetAngleDeg = avoidDeadzone(targetAngleDeg);
        }

        targetAngleDeg =
                MathUtil.clamp(
                        targetAngleDeg, TurretConst.MIN_ANGLE_DEG, TurretConst.MAX_ANGLE_DEG);

        double output = pid.calculate(turretAngleDeg, targetAngleDeg);
        output = MathUtil.clamp(output, -TurretConst.MAX_OUTPUT, TurretConst.MAX_OUTPUT);

        output = applySoftLimitTaper(output);

        if (turretAngleDeg <= TurretConst.MIN_ANGLE_DEG && output < 0) {
            output = 0.0;
        }
        if (turretAngleDeg >= TurretConst.MAX_ANGLE_DEG && output > 0) {
            output = 0.0;
        }

        motor.setControl(motorOutput.withOutput(output));
    }

    private double resolveOverlap(double rawTargetDeg) {
        double normalized = rawTargetDeg % 360.0;
        if (normalized > 180.0) normalized -= 360.0;
        if (normalized <= -180.0) normalized += 360.0;

        double altTarget = normalized + (normalized < 0 ? 360.0 : -360.0);

        boolean normalizedInRange =
                normalized >= TurretConst.MIN_ANGLE_DEG && normalized <= TurretConst.MAX_ANGLE_DEG;
        boolean altInRange =
                altTarget >= TurretConst.MIN_ANGLE_DEG && altTarget <= TurretConst.MAX_ANGLE_DEG;

        if (normalizedInRange && altInRange) {
            return Math.abs(normalized - turretAngleDeg) <= Math.abs(altTarget - turretAngleDeg)
                    ? normalized
                    : altTarget;
        } else if (normalizedInRange) {
            return normalized;
        } else if (altInRange) {
            return altTarget;
        }
        return MathUtil.clamp(normalized, TurretConst.MIN_ANGLE_DEG, TurretConst.MAX_ANGLE_DEG);
    }

    private double avoidDeadzone(double targetDeg) {
        if (targetDeg >= TurretConst.DEADZONE_MIN_DEG
                && targetDeg <= TurretConst.DEADZONE_MAX_DEG) {
            double distToMin = Math.abs(targetDeg - TurretConst.DEADZONE_MIN_DEG);
            double distToMax = Math.abs(targetDeg - TurretConst.DEADZONE_MAX_DEG);
            return distToMin <= distToMax
                    ? TurretConst.DEADZONE_MIN_DEG
                    : TurretConst.DEADZONE_MAX_DEG;
        }
        return targetDeg;
    }

    private double applySoftLimitTaper(double output) {
        double distToMin = turretAngleDeg - TurretConst.MIN_ANGLE_DEG;
        double distToMax = TurretConst.MAX_ANGLE_DEG - turretAngleDeg;

        if (distToMin < TurretConst.SOFT_LIMIT_MARGIN_DEG && output < 0) {
            double scale = distToMin / TurretConst.SOFT_LIMIT_MARGIN_DEG;
            output *= scale;
        }
        if (distToMax < TurretConst.SOFT_LIMIT_MARGIN_DEG && output > 0) {
            double scale = distToMax / TurretConst.SOFT_LIMIT_MARGIN_DEG;
            output *= scale;
        }
        return output;
    }

    public Command autoAimCommand() {
        return run(() -> autoAimEnabled = true).withName("AutoAim");
    }

    public Command manualAimCommand(double angleDeg) {
        return runEnd(
                        () -> {
                            autoAimEnabled = false;
                            targetAngleDeg =
                                    MathUtil.clamp(
                                            angleDeg,
                                            TurretConst.MIN_ANGLE_DEG,
                                            TurretConst.MAX_ANGLE_DEG);
                            double output = pid.calculate(turretAngleDeg, targetAngleDeg);
                            output =
                                    MathUtil.clamp(
                                            output,
                                            -TurretConst.MAX_OUTPUT,
                                            TurretConst.MAX_OUTPUT);
                            output = applySoftLimitTaper(output);
                            motor.setControl(motorOutput.withOutput(output));
                        },
                        () -> autoAimEnabled = true)
                .withName("ManualAim");
    }

    public Command manualAimCommand(Supplier<Double> angleSupplier) {
        return runEnd(
                        () -> {
                            autoAimEnabled = false;
                            targetAngleDeg =
                                    MathUtil.clamp(
                                            angleSupplier.get(),
                                            TurretConst.MIN_ANGLE_DEG,
                                            TurretConst.MAX_ANGLE_DEG);
                            double output = pid.calculate(turretAngleDeg, targetAngleDeg);
                            output =
                                    MathUtil.clamp(
                                            output,
                                            -TurretConst.MAX_OUTPUT,
                                            TurretConst.MAX_OUTPUT);
                            output = applySoftLimitTaper(output);
                            motor.setControl(motorOutput.withOutput(output));
                        },
                        () -> autoAimEnabled = true)
                .withName("ManualAimDynamic");
    }

    public double getAngleDeg() {
        return turretAngleDeg;
    }

    public boolean isAutoAimEnabled() {
        return autoAimEnabled;
    }

    public boolean isOnTarget() {
        return Math.abs(turretAngleDeg - targetAngleDeg) < TurretConst.ON_TARGET_TOLERANCE_DEG;
    }

    public int getCurrentZone() {
        return zoneIndex;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Angle", this::getAngleDeg, null);
        builder.addBooleanProperty("AutoAim", this::isAutoAimEnabled, null);
        builder.addBooleanProperty("OnTarget", this::isOnTarget, null);
        builder.addIntegerProperty("Zone", this::getCurrentZone, null);
    }
}
