package frc.robot.vision;

import static edu.wpi.first.math.util.Units.inchesToMeters;

import static java.lang.Math.toRadians;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public final class VisionConst {

    public static final AprilTagFields APRIL_TAG_FIELD = AprilTagFields.k2026RebuiltWelded;

    /** The maximum ambiguity (of "tag flipping") allowable to accept a pipeline result */
    public static final double MAX_AMBIGUITY = 0.2;

    public static final double TRIPLE_CAMERAS_HEIGHT = inchesToMeters(20.5);

    public static final Transform3d INTAKE_CAMERA_TRANSFORM =
            new Transform3d(
                    new Translation3d(
                            inchesToMeters(-0.483014),
                            inchesToMeters(-10.733014),
                            TRIPLE_CAMERAS_HEIGHT),
                    new Rotation3d(0.0, 0.0, toRadians(45)));
    public static final Transform3d RIGHT_CAMERA_TRANSFORM =
            new Transform3d(
                    new Translation3d(
                            inchesToMeters(-1.121457),
                            inchesToMeters(-13.115717),
                            TRIPLE_CAMERAS_HEIGHT),
                    new Rotation3d(0.0, 0.0, toRadians(45 - 120)));
    public static final Transform3d BACK_CAMERA_TRANSFORM =
            new Transform3d(
                    new Translation3d(
                            inchesToMeters(-2.865717),
                            inchesToMeters(-11.371457),
                            TRIPLE_CAMERAS_HEIGHT),
                    new Rotation3d(0.0, 0.0, toRadians(45 + 120)));
}
