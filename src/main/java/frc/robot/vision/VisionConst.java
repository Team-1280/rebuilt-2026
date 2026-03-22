package frc.robot.vision;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public final class VisionConst {

    public static final AprilTagFields APRIL_TAG_FIELD = AprilTagFields.k2026RebuiltWelded;

    /** The maximum ambiguity (of "tag flipping") allowable to accept a pipeline result */
    public static final double MAX_AMBIGUITY = 0.2;

    // public static final Transform3d BACK_LEFT_CAMERA_TRANSFORM =
    //         new Transform3d(
    //                 new Translation3d(
    //                         Units.inchesToMeters(-4.747),
    //                         Units.inchesToMeters(9.003),
    //                         Units.inchesToMeters(9.423)),
    //                 new Rotation3d(0.0, Math.toRadians(-(90 - 61.75)), Math.toRadians(180 +
    // 27.2)));
    // public static final Transform3d BACK_RIGHT_CAMERA_TRANSFORM =
    //         new Transform3d(
    //                 new Translation3d(
    //                         Units.inchesToMeters(-9.696891),
    //                         Units.inchesToMeters(-10.423049),
    //                         Units.inchesToMeters(8.887247)),
    //                 new Rotation3d(0.0, -(90 - 61.75), Math.toRadians(-90)));
    // public static final Transform3d INTAKE_CAMERA_TRANSFROM =
    //         new Transform3d(
    //                 new Translation3d(
    //                         Units.inchesToMeters(-1.5),
    //                         Units.inchesToMeters(-12.5),
    //                         Units.inchesToMeters(20)),
    //                 new Rotation3d(0.0, 0.0, Math.toRadians(45)));
    // public static final Transform3d AUX_CAMERA_TRANSFORM =
    //         new Transform3d(
    //                 new Translation3d(
    //                         Units.inchesToMeters(13.25),
    //                         Units.inchesToMeters(-2.25),
    //                         Units.inchesToMeters(4.75)),
    //                 new Rotation3d(0.0, 0.0, Math.toRadians(160)));

    public static final Translation3d TRIPLE_MOUNT_TRANSLATION =
            new Translation3d(
                    Units.inchesToMeters(-1.5),
                    Units.inchesToMeters(-12.5),
                    Units.inchesToMeters(20.5));

    public static final Transform3d TRIPLE_INTAKE_CAMERA_TRANSFORM =
            new Transform3d(TRIPLE_MOUNT_TRANSLATION, new Rotation3d(new Rotation2d(45)));
    public static final Transform3d TRIPLE_RIGHT_CAMERA_TRANSFORM =
            new Transform3d(TRIPLE_MOUNT_TRANSLATION, new Rotation3d(new Rotation2d(45 - 120)));
    public static final Transform3d TRIPLE_BACK_CAMERA_TRANSFORM =
            new Transform3d(TRIPLE_MOUNT_TRANSLATION, new Rotation3d(new Rotation2d(45 + 120)));
}
