package frc.robot.launcher.trajectory;

import static frc.robot.launcher.trajectory.TrajectoryConst.GRAVITY;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;

import frc.robot.launcher.trajectory.TrajectoryConstraints.SoftConstraint;
import frc.robot.util.PolynomialSolver;

import java.util.Arrays;
import java.util.function.BiFunction;
import java.util.stream.DoubleStream;

/**
 * Class with utilities for finding optimal launcher trajectories.
 *
 * <p>To use: Create a TrajectoryParameters object with the necessary variables, and a
 * TrajectoryConstraints with the constraints on the trajectory.
 *
 * <p>Then, pass these two objects into either TrajectorySolver.solve() or
 * TrajectorySolver.solveIgnoringVertical() (if target height does not matter).
 *
 * <p>The returned Trajectory has the calculated unknowns; use methods to get launcher pitch, yaw,
 * and flywheel speed.
 *
 * <p>This trajectory may be invalid, which should be checked. If it is invalid, the given pitch,
 * yaw, and flywheel speed are still usable (for continuity), though they may be outside of the
 * mechanical limits of the launcher so clamping should happen.
 */
public class TrajectorySolver {
    private TrajectorySolver() {} // not an instantiable class

    /**
     * Number of iterations to use in the pitch bisection algorithm optimizer.
     *
     * <p>Each iteration cuts the maximum possible pitch error from the optimal, in half. However,
     * it requires an extra trajectory calculation.
     *
     * <p>The maximum possible error can be logged in the bisection algorithm as the difference
     * between the final low pitch and high pitch.
     */
    private static final int ITERATIONS = 8; // TODO: tune based on CPU usage

    /**
     * Solve for the optimal trajectory, for the given parameters and constraints, that would get a
     * fuel launched at this time to hit the target in 3D space.
     *
     * <p>The optimal trajectory is the one that satisfies all constraints and also optimizes the
     * soft constraint of either minimizing or maximizing pitch.
     *
     * <p>This method always returns a Trajectory. The returned Trajectory *may be invalid* if no
     * valid trajectory was found. This must always be checked.
     *
     * <p>If the trajectory is invalid, its given launcher pitch, yaw, and flywheel speed are still
     * usable for aiming the launcher, without shooting fuel. (These values will often be outside of
     * the mechanical limits of the mechanisms, so they need to be clamped.)
     *
     * <p>This helps the launcher aiming and flywheel speed be continuous and smooth, avoiding jumps
     * when the robot goes out of range due to mechanical limitations.
     *
     * <p>Even if there is no valid trajectory, the returned trajectory still tries to be as optimal
     * and valid as possible.
     *
     * @param parameters the known variables for the current state of the launcher and target
     * @param constraints the physical constraints that the trajectory must satisfy or optimize
     * @return the most optimal trajectory that the method could approximate, which may be invalid
     */
    public static Trajectory solve(
            TrajectoryParameters parameters, TrajectoryConstraints constraints) {
        SoftConstraint softConstraint = constraints.getSoftConstraint();

        // The initial upper bound for pitch is the launcher's maximum achievable pitch
        double highPitch = constraints.calculateMaxPitch(parameters);
        if (softConstraint == SoftConstraint.MAXIMIZE_PITCH && highPitch < Math.PI / 2) {
            // See if this highest trajectory is usable or optimal, when maximizing pitch
            // (At a pitch straight upwards, the trajectory is always invalid, so skip it)
            Trajectory highestTrajectory = calculateFromPitch(parameters, highPitch);
            if (!constraints.checkLower(highestTrajectory)) {
                // a valid trajectory must impossibly be higher than the highest trajectory
                highestTrajectory.invalidate();
                return highestTrajectory; // closest trajectory
            } else if (constraints.checkUpper(highestTrajectory)) {
                // this highest trajectory, valid or invalid, is optimal
                return highestTrajectory;
            }
            // If upper constraints aren't satisfied, continue and search lower
        }

        // The initial lower bound for pitch is typically the elevation angle to the target
        double lowPitch = constraints.calculateMinPitch(parameters);
        if (softConstraint == SoftConstraint.MINIMIZE_PITCH
                && lowPitch > parameters.getElevationAngle()) {
            // See if this lowest trajectory is usable or optimal, when minimizing pitch
            // (At elevation angle pitch, the trajectory is always invalid, so skip it)
            Trajectory lowestTrajectory = calculateFromPitch(parameters, lowPitch);
            if (!constraints.checkUpper(lowestTrajectory)) {
                // a valid trajectory must impossibly be lower than the lowest trajectory
                lowestTrajectory.invalidate();
                return lowestTrajectory; // closest trajectory
            } else if (constraints.checkLower(lowestTrajectory)) {
                // this lowest trajectory, valid or invalid, is optimal
                return lowestTrajectory;
            }
            // If lower constraints aren't satisfied, continue and search higher
        }

        // First, guess with the pitch that minimizes speed
        double guessPitch = parameters.getMinimalSpeedPitch();

        // Use a pitch approximation algorithm to find a trajectory very close to the optimal
        return computeOptimalPitchTrajectory(
                TrajectorySolver::calculateFromPitch,
                parameters,
                constraints,
                lowPitch,
                highPitch,
                guessPitch);
    }

    /**
     * Solve for the optimal trajectory, but ignore the vertical displacement of the target, and
     * therefore ignore all vertical coordinates except in the constraints.
     *
     * <p>This method is useful when the target is instead a horizontal zone or an infinitely tall
     * volume. For example, passing the fuel to an alliance zone.
     *
     * <p>Note that since the vertical displacement is ignored, it should be checked that the
     * trajectory's height is still viable and, for example, doesn't exit the field.
     *
     * @param parameters the parameters of the trajectory
     * @param constraints the constraints of the trajectory
     * @return a trajectory, valid or invalid
     */
    public static Trajectory solveIgnoringVertical(
            TrajectoryParameters parameters, TrajectoryConstraints constraints) {
        double lowPitch = constraints.calculateMinPitch(parameters);
        double highPitch = constraints.calculateMaxPitch(parameters);
        // First, guess either low or high pitch since it may already be optimal
        double guessPitch =
                constraints.getSoftConstraint() == SoftConstraint.MINIMIZE_PITCH
                        ? lowPitch
                        : highPitch;

        return computeOptimalPitchTrajectory(
                (params, pitch) -> {
                    // Use maximum speed
                    double speed = constraints.calculateApproximateMaxSpeed(pitch);
                    return calculateFromPitchIgnoringVertical(params, pitch, speed);
                },
                parameters,
                constraints,
                lowPitch,
                highPitch,
                guessPitch);
    }

    /**
     * Approximate the optimal trajectory by using a bisection algorithm with pitch as the variable.
     *
     * @param calculateTrajectory a function that takes in the parameters and a pitch, and returns a
     *     calculated Trajectory from it, valid or invalid
     * @param parameters the known trajectory parameters
     * @param constraints the constraints placed on the trajectories
     * @param lowPitch the initial lower bound of the pitch (field-relative)
     * @param highPitch the initial upper bound of the pitch (field-relative)
     * @param guessPitch the initial guess for the pitch (field-relative)
     * @return the most optimal trajectory found, which may be invalid if none valid were found
     */
    public static Trajectory computeOptimalPitchTrajectory(
            BiFunction<TrajectoryParameters, Double, Trajectory> calculateTrajectory,
            TrajectoryParameters parameters,
            TrajectoryConstraints constraints,
            double lowPitch,
            double highPitch,
            double guessPitch) {
        // Clamp the guess pitch to avoid problems with erroneous guesses
        guessPitch = MathUtil.clamp(guessPitch, lowPitch, highPitch);
        SoftConstraint softConstraint = constraints.getSoftConstraint();
        Trajectory trajectory = calculateTrajectory.apply(parameters, guessPitch);
        Trajectory bestTrajectory = null; // keep track of the best pitch's trajectory so far

        // bisection algorithm
        for (int i = 0; ; i++) { // Note: the stop condition is in the middle of the loop
            // Iterate to search for better trajectories between lowPitch and highPitch
            if (softConstraint == SoftConstraint.MAXIMIZE_PITCH) {
                // Maximize pitch to the upper bound from upper constraints
                if (constraints.checkUpper(trajectory)) {
                    // Upper constraints satisfied, record trajectory and continue upwards
                    bestTrajectory = trajectory;
                    lowPitch = guessPitch;
                } else {
                    // Upper constraints failed, try a lower pitch
                    highPitch = guessPitch;
                }
            } else {
                // Minimize pitch to the lower bound from lower constraints
                if (constraints.checkLower(trajectory)) {
                    // Lower constraints satisfied, record trajectory and continue downwards
                    bestTrajectory = trajectory;
                    highPitch = guessPitch;
                } else {
                    // Lower constraints failed, try a higher pitch
                    lowPitch = guessPitch;
                }
            }

            if (i >= ITERATIONS || lowPitch >= highPitch) {
                // Stop after enough iterations or if there is nothing left to approximate
                break;
            }
            // Set up next iteration
            guessPitch = (lowPitch + highPitch) / 2; // Guess in the middle of the possible range
            trajectory = calculateTrajectory.apply(parameters, guessPitch);
        }

        if (bestTrajectory == null) {
            // The algorithm did not reach a valid trajectory; use the final, closest one
            bestTrajectory = trajectory;
        }
        if (!constraints.checkAll(bestTrajectory)) {
            // Invalidate the trajectory if it does not satisfy all constraints
            bestTrajectory.invalidate();
        }
        return bestTrajectory;
    }

    /**
     * Given trajectory parameters and a pitch, calculate the other unknowns and give the one
     * trajectory for this pitch, valid or invalid.
     *
     * <p>The trajectory theoretically will only be invalid if the given pitch is below (or equal
     * to) the elevation angle to the target, as then the projectile must accelerate upwards to
     * reach the target.
     *
     * <p>If the trajectory is invalid, it will still give variables where launch speed is infinite
     * and yaw is directly towards the target, as this is the closest that the trajectory can get.
     *
     * <p>This uses a closed-form solution described in the technical trajectory calculations paper.
     *
     * @param parameters the known trajectory parameters
     * @param pitch the pitch to calculate the trajectory for, relative to the field
     * @return the trajectory for this pitch, which may be invalid if the pitch is too low
     */
    public static Trajectory calculateFromPitch(TrajectoryParameters parameters, double pitch) {
        Translation2d horizontalDisplacement = parameters.getHorizontalDisplacement();
        double verticalDisplacement = parameters.getVerticalDisplacement();
        Translation2d launcherVelocity = parameters.getLauncherVelocity();

        // Initialize useful variables
        double time = Double.NaN;
        double launchSlope = Math.tan(pitch);
        double elevationSlope = verticalDisplacement / parameters.getHorizontalDistance();

        // If launch pitch is too low and invalid, skip calculations
        if (launchSlope > elevationSlope) {
            // Since we are aiming higher than the target, there should be a valid solution
            // Use the closed-form solution for the time quartic equation
            double launchSlopeSquared = launchSlope * launchSlope;
            double a = GRAVITY * GRAVITY / 4.0;
            double c =
                    verticalDisplacement * GRAVITY
                            - launchSlopeSquared * launcherVelocity.getSquaredNorm();
            double d = 2.0 * launchSlopeSquared * horizontalDisplacement.dot(launcherVelocity);
            double e =
                    verticalDisplacement * verticalDisplacement
                            - launchSlopeSquared * horizontalDisplacement.getSquaredNorm();
            double[] realRoots = PolynomialSolver.depressedQuarticRealRoots(a, c, d, e);

            // Check whether there may be extraneous time solutions corresponding to inverted pitch
            if (-launchSlope <= elevationSlope) {
                // Common case (always happens when elevationSlope > 0.0):
                // The negative launch slope has an imaginary time, so no extraneous solutions
                for (double realRoot : realRoots) {
                    // Get one real root *efficiently*. There should be exactly one real root.
                    if (realRoot > 0.0) {
                        time = realRoot;
                        break;
                    }
                }
            } else {
                // Edge case:
                // The negative launch slope time (extraneous) is real, so we must disregard it
                DoubleStream positiveRoots = Arrays.stream(realRoots).filter(t -> t > 0.0);
                // Choose the larger time for positive launch slope, concisely
                time =
                        (launchSlope >= 0.0 ? positiveRoots.max() : positiveRoots.min())
                                .orElse(Double.NaN);
            }
        }

        // Check if any valid time solution was found
        if (!Double.isFinite(time)) {
            // No valid trajectory exists for this pitch because it is too low.
            // Give the closest theoretical trajectory for continuity: if pitch is elevation angle,
            // then speed is infinite, yaw is horizontal displacement direction, and time is 0.
            double yaw = horizontalDisplacement.getAngle().getRadians();
            return new Trajectory(parameters, pitch, yaw, Double.POSITIVE_INFINITY, 0.0, false);
        }

        // Calculate the remaining unknowns for the valid trajectory
        Translation2d effectiveDisplacement =
                horizontalDisplacement.minus(launcherVelocity.times(time));
        double yaw = effectiveDisplacement.getAngle().getRadians();
        double speed = effectiveDisplacement.getNorm() / (time * Math.cos(pitch));
        return new Trajectory(parameters, pitch, yaw, speed, time, true);
    }

    /**
     * Given trajectory parameters, a pitch, and a launch speed, calculate the one unknown of yaw
     * ignoring the vertical direction. Give the trajectory, which may be valid or invalid.
     *
     * <p>The trajectory will be invalid if the horizontal launch speed is too low to overcome the
     * launcher's contributing velocity, so the fuel can never reach the target.
     *
     * <p>Invalid trajectories happen when either the launcher velocity is much too orthogonal
     * ("strafing") compared to the displacement, or when the calculated projectile exit velocity is
     * aligned with the displacement but goes backwards.
     *
     * <p>If the trajectory is invalid, this method will still give the closest possible trajectory
     * to being valid. For high orthogonal launcher velocities, it will cancel out as much of the
     * orthogonal component of the exit velocity as possible.
     *
     * <p>This uses a closed-form solution described in the technical trajectory calculations paper.
     *
     * @param parameters the known trajectory parameters
     * @param pitch the pitch to calculate the trajectory for, relative to the field
     * @param speed the initial launch speed of the projectile when exiting the launcher
     * @return the trajectory, ignoring vertical coordinates, for this pitch and speed
     */
    public static Trajectory calculateFromPitchIgnoringVertical(
            TrajectoryParameters parameters, double pitch, double speed) {
        Translation2d launcherVelocity = parameters.getLauncherVelocity();
        double horizontalSpeed = speed * Math.cos(pitch);
        double distance = parameters.getHorizontalDistance();
        Translation2d normalizedDisplacement = parameters.getHorizontalDisplacement().div(distance);
        boolean trajectoryValid = true;

        // Find the value of the input to arcsin for the yaw calculation
        double arcsinArgument = launcherVelocity.cross(normalizedDisplacement) / horizontalSpeed;
        if (Math.abs(arcsinArgument) > 1.0) {
            // No solution exists for aligning the projectile velocity with the displacement
            trajectoryValid = false;
            // Align as best as possible
            arcsinArgument = MathUtil.clamp(arcsinArgument, -1.0, 1.0);
            // For this case, time represents the time of the closest point on the trajectory to the
            // target horizontally
        }

        // Calculate yaw
        double yawOffset = Math.asin(arcsinArgument);
        double yaw = normalizedDisplacement.getAngle().getRadians() + yawOffset;

        // Calculate time, and check if it is negative and therefore invalid
        double alignedVelocity =
                launcherVelocity.dot(normalizedDisplacement)
                        + horizontalSpeed * Math.cos(yawOffset);
        double time = distance / alignedVelocity;
        if (time <= 0.0) {
            // projectile velocity is parallel but still goes backwards
            trajectoryValid = false;
        }

        return new Trajectory(parameters, pitch, yaw, speed, time, trajectoryValid);
    }
}
