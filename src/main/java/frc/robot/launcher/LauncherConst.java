package frc.robot.launcher;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public final class LauncherConst {
    /** The transform from the robot origin to the launcher exit point, including rotation. */
    public static final Transform3d ROBOT_TO_LAUNCHER_TRANSFORM =
            new Transform3d(
                    new Translation3d(0.0, 0.0, 0.0), // TODO: launcher translation
                    Rotation3d.kZero);
}
