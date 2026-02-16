package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.turret.TurretConst;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.Optional;
import java.util.function.BiConsumer;

public class VisionSubsystem extends SubsystemBase {
    private final BiConsumer<Pose2d, Double>
            addVisionMeasurement; // Takes (robot pose, timestampSec)

    /** All vision cameras for pose estimation */
    private final Camera[] cameras = {
        new Camera("front", VisionConst.FRONT_CAMERA_TRANSFORM),
        new Camera("back", VisionConst.BACK_CAMERA_TRANSFORM)
    };

    /** Aux camera for hub targeting (NOT used for pose estimation) */
    private final Camera auxCamera = new Camera("aux", VisionConst.AUX_CAMERA_TRANSFORM);

    private HubTarget latestHubTarget = HubTarget.NONE;
    private int[] activeHubTags = {};

    /** Snapshot of hub target data from the aux camera. */
    public static class HubTarget {
        public final boolean visible;
        public final double yawDeg;
        public final int tagId;
        public final double area;

        public static final HubTarget NONE = new HubTarget(false, 0.0, -1, 0.0);

        public HubTarget(boolean visible, double yawDeg, int tagId, double area) {
            this.visible = visible;
            this.yawDeg = yawDeg;
            this.tagId = tagId;
            this.area = area;
        }
    }

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

        // Aux camera hub target processing
        updateActiveHubTags();
        processAuxCamera();
    }

    /** Sets the active hub tag filter based on alliance color. */
    private void updateActiveHubTags() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) {
            activeHubTags = new int[0];
            return;
        }
        activeHubTags =
                alliance.get() == Alliance.Red
                        ? TurretConst.RED_HUB_ALL_TAGS
                        : TurretConst.BLUE_HUB_ALL_TAGS;
    }

    /** Processes aux camera results to find the best hub target. */
    private void processAuxCamera() {
        HubTarget best = HubTarget.NONE;

        for (PhotonPipelineResult result : auxCamera.getUnreadResults()) {
            if (!result.hasTargets()) {
                continue;
            }
            for (PhotonTrackedTarget target : result.getTargets()) {
                if (!isHubTag(target.getFiducialId())) {
                    continue;
                }
                if (target.getArea() < TurretConst.MIN_TARGET_AREA) {
                    continue;
                }
                // Prefer center tags over peripheral tags, then largest area
                if (isBetterTarget(target, best)) {
                    best =
                            new HubTarget(
                                    true,
                                    target.getYaw(),
                                    target.getFiducialId(),
                                    target.getArea());
                }
            }
        }

        latestHubTarget = best;

        // Log aux camera state
        Logger.recordOutput("Vision/Aux/HasTarget", latestHubTarget.visible);
        Logger.recordOutput("Vision/Aux/YawDeg", latestHubTarget.yawDeg);
        Logger.recordOutput("Vision/Aux/TagId", latestHubTarget.tagId);
        Logger.recordOutput("Vision/Aux/Area", latestHubTarget.area);
    }

    /** Checks if a tag ID is in the active hub tag set. */
    private boolean isHubTag(int tagId) {
        for (int id : activeHubTags) {
            if (id == tagId) {
                return true;
            }
        }
        return false;
    }

    /** Checks if a tag ID is a center hub tag for the current alliance. */
    private boolean isCenterTag(int tagId) {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) {
            return false;
        }
        int[] centerTags =
                alliance.get() == Alliance.Red
                        ? TurretConst.RED_HUB_CENTER_TAGS
                        : TurretConst.BLUE_HUB_CENTER_TAGS;
        for (int id : centerTags) {
            if (id == tagId) {
                return true;
            }
        }
        return false;
    }

    /** Returns true if candidate is a better hub target than current best. */
    private boolean isBetterTarget(PhotonTrackedTarget candidate, HubTarget currentBest) {
        boolean candidateIsCenter = isCenterTag(candidate.getFiducialId());
        boolean bestIsCenter = currentBest.visible && isCenterTag(currentBest.tagId);
        // Center tags always win over peripheral
        if (candidateIsCenter && !bestIsCenter) {
            return true;
        }
        if (!candidateIsCenter && bestIsCenter) {
            return false;
        }
        // Among same priority, prefer larger area
        return candidate.getArea() > currentBest.area;
    }

    /** Returns the latest hub target snapshot from the aux camera. */
    public HubTarget getHubTarget() {
        return latestHubTarget;
    }

    @Override
    public void initSendable(SendableBuilder builder) {}
}
