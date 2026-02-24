package frc.robot.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import java.util.ArrayList;
import java.util.Optional;

public class Camera {
    private final PhotonCamera camera;
    private final PhotonPoseEstimator poseEstimator;

    /** A vision measurement with all metadata needed for trust computation. */
    public record VisionMeasurement(
            Pose2d pose,
            double timestampSeconds,
            double distanceMeters,
            double ambiguity,
            int numTargets) {}

    /**
     * Create a new Camera representing a physical vision camera on the robot.
     *
     * @param cameraName The name of the camera as configured in PhotonVision
     * @param robotToCameraTransform The transform from the robot's origin to the camera
     */
    public Camera(String cameraName, Transform3d robotToCameraTransform) {
        this.camera = new PhotonCamera(cameraName);
        this.poseEstimator =
                new PhotonPoseEstimator(
                        AprilTagFieldLayout.loadField(VisionConst.APRIL_TAG_FIELD),
                        robotToCameraTransform);
    }

    /**
     * Updates the camera and returns vision measurements from the latest unread results. Should be
     * called periodically.
     *
     * @return The latest vision measurements, including distance, ambiguity, and target count
     */
    public ArrayList<VisionMeasurement> update() {
        ArrayList<VisionMeasurement> measurements = new ArrayList<>();
        for (PhotonPipelineResult result : camera.getAllUnreadResults()) {
            if (!shouldUseResult(result)) {
                continue;
            }
            Optional<EstimatedRobotPose> estimate =
                    poseEstimator.estimateAverageBestTargetsPose(result);
            if (estimate.isEmpty()) {
                continue;
            }
            EstimatedRobotPose est = estimate.get();
            double dist =
                    est.targetsUsed.stream()
                            .mapToDouble(t -> t.getBestCameraToTarget().getTranslation().getNorm())
                            .average()
                            .orElse(Double.MAX_VALUE);
            int numTargets = est.targetsUsed.size();
            double ambiguity = result.getBestTarget().getPoseAmbiguity();
            measurements.add(
                    new VisionMeasurement(
                            est.estimatedPose.toPose2d(),
                            est.timestampSeconds,
                            dist,
                            ambiguity,
                            numTargets));
        }
        return measurements;
    }

    /**
     * Decides whether to use the given pipeline result for pose estimation.
     *
     * @param result the pipeline result to evaluate
     * @return true if the result should be used, false otherwise
     */
    public boolean shouldUseResult(PhotonPipelineResult result) {
        return result.hasTargets()
                && result.getBestTarget().getPoseAmbiguity() < VisionConst.MAX_AMBIGUITY;
    }
}
