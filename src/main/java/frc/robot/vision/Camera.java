package frc.robot.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Transform3d;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import java.util.Optional;

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

    private PhotonPipelineResult latestResult;

/**
 * gets distance to the best AprilTag in meters.
 */
public double getDistanceToTag() {
    if (latestResult == null || !latestResult.hasTargets()) {
        return Double.MAX_VALUE;
    }
    // gfet transform to best target, extract distance
    var bestTarget = latestResult.getBestTarget();
    var transform = bestTarget.getBestCameraToTarget();
    return Math.hypot(
        transform.getX(),
        Math.hypot(transform.getY(), transform.getZ())
    );
}

/**
 * fgets pose ambiguity of best target (0 = good, higher = worse).
 */
    public double getAmbiguity() {
    if (latestResult == null | !latestResult.hasTargets()) {
        return 1.0;
    }
    return latestResult.getBestTarget().getPoseAmbiguity();
}

    /**
     * updates the camera and returns an estimated robot pose if available, shhould be called
     * periodically
     *
     * @return the latest estimated robot pose from the results
     */

public Optional<EstimatedRobotPose> update() {
    Optional<EstimatedRobotPose> estimatedPose = Optional.empty();
    for (PhotonPipelineResult result : camera.getAllUnreadResults()) {
        if (shouldUseResult(result)) {
            latestResult = result;  // Store for later use
            estimatedPose = poseEstimator.estimateAverageBestTargetsPose(result);
        }
    }
    return estimatedPose;
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
