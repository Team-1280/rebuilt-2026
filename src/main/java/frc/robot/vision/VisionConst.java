package frc.robot.vision;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public final class VisionConst {
    public static final AprilTagFields APRIL_TAG_FIELD = AprilTagFields.k2026RebuiltWelded;

    public static final double MAX_AMBIGUITY = 0.2;

    public static final Transform3d FRONT_CAMERA_TRANSFORM =
            new Transform3d(
                    new Translation3d(
                            Units.inchesToMeters(13.0),
                            Units.inchesToMeters(5.0),
                            Units.inchesToMeters(6.0)),
                    new Rotation3d(0.0, Math.toRadians(20), Math.toRadians(0)));
    public static final Transform3d BACK_CAMERA_TRANSFORM =
            new Transform3d(
                    new Translation3d(
                            Units.inchesToMeters(13.25),
                            Units.inchesToMeters(-2.25),
                            Units.inchesToMeters(4.75)),
                    new Rotation3d(0.0, 0.0, Math.toRadians(160)));
}
