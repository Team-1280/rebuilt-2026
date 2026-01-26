package frc.robot.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.MathUtil;
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
        this.poseEstimator = new PhotonPoseEstimator(
                AprilTagFieldLayout.loadField(VisionConst.APRIL_TAG_FIELD),
                robotToCameraPose);
    }

    public Optional<EstimatedRobotPose> update() {
        Optional<EstimatedRobotPose> estimatedPose = Optional.empty();
        for (PhotonPipelineResult result : camera.getAllUnreadResults()) {
            estimatedPose = poseEstimator.estimateAverageBestTargetsPose(result);
        }
        return estimatedPose;
    }

    // private boolean visionConstraints() {
    // var result = camera.getAllUnreadResults();
    // boolean inSnapRange = null;
    // if
    // (result.getBestTarget().getAlternateCameraToTarget().getTranslation().getNorm()
    // < 3
    // && MathUtil.inputModulus(
    // result.getBestTarget().getAlternateCameraToTarget().getRotation().toRotation2d().getDegrees()
    // + 15,
    // -180, 180) < 30
    // && Arrays.stream(validIds).anyMatch(n -> n == (int)
    // result.getBestTarget().getFiducialId())) {
    // inSnapRange = true;
    // } else {
    // inSnapRange = false;
    // }
    // }
}
