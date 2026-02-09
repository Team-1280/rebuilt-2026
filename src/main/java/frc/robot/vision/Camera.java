package frc.robot.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.ArrayList;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

public class Camera {
    private final PhotonCamera camera;
    private final PhotonPoseEstimator poseEstimator;

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
     * Updates the camera and returns estimated robot poses from the latest unread results. Should
     * be called periodically.
     *
     * @return The latest estimated robot poses from the results, if available
     */
    public ArrayList<EstimatedRobotPose> update() {
        ArrayList<EstimatedRobotPose> estimatedPoses = new ArrayList<>();
        for (PhotonPipelineResult result : camera.getAllUnreadResults()) {
            if (!shouldUseResult(result)) {
                continue;
            }
            Optional<EstimatedRobotPose> estimate =
                    poseEstimator.estimateAverageBestTargetsPose(result);
            if (estimate.isEmpty()) {
                continue;
            }
            estimatedPoses.add(estimate.get());
        }
        return estimatedPoses;
    }

    /**
     * Decides whether to use the given pipeline result for pose estimation. This method should be
     * extended more to filter out bad results.
     *
     * @param result
     * @return true if the result should be used, false otherwise
     */
    public boolean shouldUseResult(PhotonPipelineResult result) {
        return result.hasTargets()
                && result.getBestTarget().getPoseAmbiguity() < VisionConst.MAX_AMBIGUITY;
    }
}
