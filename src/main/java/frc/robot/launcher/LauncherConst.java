package frc.robot.launcher;

import static edu.wpi.first.units.Units.Inches;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public final class LauncherConst {
    public static final CANBus CAN_BUS = new CANBus("launcher");

    /** The transform from the robot origin to the launcher exit point, including rotation. */
    public static final Transform3d ROBOT_TO_LAUNCHER_TRANSFORM =
            new Transform3d(
                    new Translation3d( // bottom left quadrant of robot
                            Inches.of(-2.6967), // turret center x
                            Inches.of(4.50), // turret center y
                            Inches.of(0.0)), // TODO: launcher translation Z
                    Rotation3d.kZero);
}
