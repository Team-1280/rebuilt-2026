package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {

    /**
     * Functional interface for injecting vision measurements into the drivetrain.
     *
     * <p>Carries all metadata needed for trust computation (distance, ambiguity, target count),
     * ensuring measurements route through the full trust pipeline rather than the base-class {@code
     * addVisionMeasurement(Pose2d, double)} bypass.
     */
    @FunctionalInterface
    public interface VisionMeasurementConsumer {
        void accept(
                Pose2d pose,
                double timestampSeconds,
                double distanceMeters,
                double ambiguity,
                int numTargets);
    }

    private final VisionMeasurementConsumer addVisionMeasurement;

    /** All vision cameras */
    private final Camera[] cameras = {
        new Camera("Back Left", VisionConst.BACK_LEFT_CAMERA_TRANSFORM),
        new Camera("Back Right", VisionConst.BACK_RIGHT_CAMERA_TRANSFORM)
    };

    /**
     * @param addVisionMeasurement Callback that injects a vision measurement into the drivetrain's
     *     trust-aware estimator, e.g. {@code drivetrain::addVisionMeasurement}
     */
    public VisionSubsystem(VisionMeasurementConsumer addVisionMeasurement) {
        this.addVisionMeasurement = addVisionMeasurement;
    }

    @Override
    public void periodic() {
        for (Camera camera : cameras) {
            for (Camera.VisionMeasurement measurement : camera.update()) {
                addVisionMeasurement.accept(
                        measurement.pose(),
                        measurement.timestampSeconds(),
                        measurement.distanceMeters(),
                        measurement.ambiguity(),
                        measurement.numTargets());
            }
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {}
}
