package frc.robot.vision;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public final class VisionConst {
    public static final AprilTagFields APRIL_TAG_FIELD = AprilTagFields.k2026RebuiltWelded;

    /** The maximum ambiguity (of "tag flipping") allowable to accept a pipeline result */
    public static final double MAX_AMBIGUITY = 0.2;

    // Prototype mounts
    public static final Transform3d FRONT_CAMERA_TRANSFORM =
            new Transform3d(
                    new Translation3d(
                            Units.inchesToMeters(10.876112),
                            Units.inchesToMeters(9.499361),
                            Units.inchesToMeters(9.310833)),
                    new Rotation3d(0.0, Math.toRadians(28.125), Math.toRadians(90)));
    public static final Transform3d BACK_CAMERA_TRANSFORM =
            new Transform3d(
                    new Translation3d(
                            Units.inchesToMeters(-10.876112),
                            Units.inchesToMeters(9.499361),
                            Units.inchesToMeters(9.310833)),
                    new Rotation3d(0.0, 28.125, Math.toRadians(-90)));
}
