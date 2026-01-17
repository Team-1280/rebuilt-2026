package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.photonvision.EstimatedRobotPose;

import java.util.Optional;
import java.util.function.BiConsumer;

public class VisionSubsystem extends SubsystemBase {
    private final BiConsumer<Pose2d, Double>
            addVisionMeasurement; // Takes (robot pose, timestampSec)

    private final Camera[] cameras = {
        new Camera(
                "front",
                new Transform3d(
                        new Translation3d(
                                Units.inchesToMeters(13.0),
                                Units.inchesToMeters(5.0),
                                Units.inchesToMeters(6.0)),
                        new Rotation3d(0.0, Math.toRadians(20), Math.toRadians(0)))),
        new Camera(
                "back",
                new Transform3d(
                        new Translation3d(
                                Units.inchesToMeters(13.25),
                                Units.inchesToMeters(-2.25),
                                Units.inchesToMeters(4.75)),
                        new Rotation3d(0.0, 0.0, Math.toRadians(160))))
    };

    public VisionSubsystem(BiConsumer<Pose2d, Double> addVisionMeasurement) {
        this.addVisionMeasurement = addVisionMeasurement;
    }

    @Override
    public void periodic() {
        for (Camera camera : cameras) {
            Optional<EstimatedRobotPose> estimatedPose = camera.update();
            if (estimatedPose.isPresent()) {
                Pose2d pose = estimatedPose.get().estimatedPose.toPose2d();
                double timestamp = estimatedPose.get().timestampSeconds;
                addVisionMeasurement.accept(pose, timestamp);
            }
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {}
}
