package frc.robot.target;

import edu.wpi.first.math.geometry.Translation3d;

import frc.robot.trajectory.TrajectoryConstraints;

public record LaunchTarget(
        Translation3d translation, boolean ignoresVertical, TrajectoryConstraints constraints) {}
