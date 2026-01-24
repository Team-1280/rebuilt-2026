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

    public Camera(String cameraName, Transform3d robotToCameraPose) {
        this.camera = new PhotonCamera(cameraName);
        this.poseEstimator =
                new PhotonPoseEstimator(
                        AprilTagFieldLayout.loadField(VisionConst.APRIL_TAG_FIELD),
                        robotToCameraPose);
    }

    public Optional<EstimatedRobotPose> update() {
        Optional<EstimatedRobotPose> estimatedPose = Optional.empty();
        for (PhotonPipelineResult result : camera.getAllUnreadResults()) {
            if (shouldUseResult(result)) {
                estimatedPose = poseEstimator.estimateAverageBestTargetsPose(result);
            }
        }
        return estimatedPose;
    }

    public boolean shouldUseResult(PhotonPipelineResult result) {
        return result.hasTargets()
                && result.getBestTarget().getPoseAmbiguity() < VisionConst.MAX_AMBIGUITY;
    }
}
