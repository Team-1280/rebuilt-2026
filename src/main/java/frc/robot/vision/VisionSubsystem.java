package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.photonvision.EstimatedRobotPose;

import java.util.function.BiConsumer;

public class VisionSubsystem extends SubsystemBase {
    private final BiConsumer<Pose2d, Double>
            addVisionMeasurement; // Takes (robot pose, timestampSec)

    /** All vision ameras */
    private final Camera[] cameras = {
        new Camera("front", VisionConst.FRONT_CAMERA_TRANSFORM),
        new Camera("back", VisionConst.BACK_CAMERA_TRANSFORM)
    };

    /**
     * @param addVisionMeasurement Function that the subsystem calls to add a vision measurement,
     *     e.g. drivetrain::addVisionMeasurement
     */
    public VisionSubsystem(BiConsumer<Pose2d, Double> addVisionMeasurement) {
        this.addVisionMeasurement = addVisionMeasurement;
    }

    @Override
    public void periodic() {
        // Update each camera, and add their measurements if available

        // TODO: possible feature: use estimatedPose.targetsUsed properties (ambiguity, area, skew,
        // etc.) to weigh measurements
        for (Camera camera : cameras) {
            for (EstimatedRobotPose estimatedPose : camera.update()) {
                Pose2d pose = estimatedPose.estimatedPose.toPose2d();
                double timestamp = estimatedPose.timestampSeconds;
                addVisionMeasurement.accept(pose, timestamp);
            }
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {}
}
