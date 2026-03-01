package frc.robot.launcher;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public final class LauncherConst {
    /** The transform from the robot origin to the launcher exit point, including rotation. */
    public static final Transform3d ROBOT_TO_LAUNCHER_TRANSFORM =
            new Transform3d(
                    new Translation3d( // bottom left quadrant of robot
                            Inches.of(-4.273), // turret center x
                            Inches.of(4.273), // turret center y
                            Inches.of(-1)), // TODO: launcher translation Z
                    Rotation3d.kZero);
}
