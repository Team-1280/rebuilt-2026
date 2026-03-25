package frc.robot.trajectory;

import static frc.robot.trajectory.TrajectoryConst.GRAVITY;

import java.util.ArrayList;

/** Physical and optimization constraints for filtering and optimizing trajectories */
public class TrajectoryConstraints {
    /** Whether the code should try to minimize or maximize the launch pitch, or minimize speed */
    public static enum SoftConstraint {
        /** Minimize the pitch, reducing time. */
        MINIMIZE_PITCH,
        /** Maximize the pitch, increasing time. */
        MAXIMIZE_PITCH,
        /** Minimize the required launch speed, avoiding speed constraints. */
        MINIMIZE_SPEED, // note: use maximize pitch as a fallback for this in some situations
        /** Optimize for a custom target pitch, set with the withTargetPitch() method. */
        TARGET_PITCH, // note: use minimize pitch as a fallback for this in some situations
    }

    /**
     * A physical, vertical, widthless barrier in the path of trajectory, that is distance away from
     * the launcher and is height above the ground. Lower obstacles are from the ground to the
     * height, and upper obstacles are from the height to the sky.
     */
    public static record Obstacle(double distance, double height) {}

    private SoftConstraint softConstraint;
    private double minLauncherPitch;
    private double maxLauncherPitch;
    private double maxFlywheelSpeed;

    private double minSpeed = 0.0;
    private double minTime = 0.0;
    private double maxTime = Double.POSITIVE_INFINITY;
    private final ArrayList<Obstacle> lowerObstacles = new ArrayList<>();
    private final ArrayList<Obstacle> upperObstacles = new ArrayList<>();
    private double maxHeight = Double.POSITIVE_INFINITY;
    private double targetPitch = 0.0;

    /**
     * Create a set of constraints.
     *
     * @param softConstraint where pitch should be optimized towards
     * @param minLauncherPitch the minimum allowed pitch of the launcher relative to the robot
     * @param maxLauncherPitch the maximum allowed pitch of the launcher relative to the robot
     * @param maxFlywheelSpeed the maximum allowed flywheel speed, which launch speed depends on
     */
    public TrajectoryConstraints(
            SoftConstraint softConstraint,
            double minLauncherPitch,
            double maxLauncherPitch,
            double maxFlywheelSpeed) {
        this.softConstraint = softConstraint;
        this.minLauncherPitch = minLauncherPitch;
        this.maxLauncherPitch = maxLauncherPitch;
        this.maxFlywheelSpeed = maxFlywheelSpeed;
    }

    /** Create a set of constraints with no initial mechanical limits. */
    public TrajectoryConstraints(SoftConstraint softConstraint) {
        this(
                softConstraint,
                Double.NEGATIVE_INFINITY,
                Double.POSITIVE_INFINITY,
                Double.POSITIVE_INFINITY);
    }

    /**
     * Check that the trajectory satisfies all constraints.
     *
     * <p>A return value of true means that the trajectory is valid under the constraints.
     */
    public boolean checkAll(Trajectory trajectory) {
        return checkUpper(trajectory) && checkLower(trajectory);
    }

    /**
     * Check that the trajectory satisfies all upper constraints.
     *
     * <p>If the trajectory fails any upper constraint, this returns false.
     *
     * <p>If the trajectory fails these constraints, then it means a valid trajectory must have a
     * pitch lower than this trajectory.
     */
    public boolean checkUpper(Trajectory trajectory) {
        return checkUpperMaxFlywheelSpeed(trajectory)
                && checkUpperMaxLauncherPitch(trajectory)
                && checkUpperObstacles(trajectory)
                && checkUpperMinSpeed(trajectory)
                && checkUpperMaxHeight(trajectory);
    }

    /**
     * Check that the trajectory satisfies all lower constraints.
     *
     * <p>If the trajectory fails any lower constraint, this returns false.
     *
     * <p>If the trajectory fails these constraints, then it means a valid trajectory must have a
     * pitch higher than this trajectory.
     */
    public boolean checkLower(Trajectory trajectory) {
        return checkLowerMaxFlywheelSpeed(trajectory)
                && checkLowerMinLauncherPitch(trajectory)
                && checkLowerObstacles(trajectory)
                && checkLowerMinSpeed(trajectory);
    }

    /**
     * Check that the trajectory requires a low enough flywheel speed, and that this constraint is
     * an upper constraint (pitch is approaching the minimum speed pitch from above).
     */
    private boolean checkUpperMaxFlywheelSpeed(Trajectory trajectory) {
        return trajectory.getPitch() < trajectory.getParameters().getMinimalSpeedPitch()
                || checkMaxFlywheelSpeed(trajectory);
    }

    /**
     * Check that the trajectory requires a low enough flywheel speed, and that this constraint is a
     * lower constraint (pitch is approaching the minimum speed pitch from below).
     */
    private boolean checkLowerMaxFlywheelSpeed(Trajectory trajectory) {
        return trajectory.getPitch() > trajectory.getParameters().getMinimalSpeedPitch()
                || checkMaxFlywheelSpeed(trajectory);
    }

    /** Check that the trajectory requires a low enough flywheel speed in general. */
    private boolean checkMaxFlywheelSpeed(Trajectory trajectory) {
        return trajectory.getFlywheelSpeed() <= maxFlywheelSpeed;
    }

    /**
     * Check that the trajectory has a high enough launch speed, and that this constraint would be
     * an upper constraint on pitch (otherwise check passes).
     */
    private boolean checkUpperMinSpeed(Trajectory trajectory) {
        return trajectory.getPitch() > trajectory.getParameters().getMinimalSpeedPitch()
                || trajectory.getSpeed() >= minSpeed;
    }

    /**
     * Check that the trajectory has a high enough launch speed, and that this constraint would be a
     * lower constraint on pitch (otherwise check passes).
     */
    private boolean checkLowerMinSpeed(Trajectory trajectory) {
        return trajectory.getPitch() < trajectory.getParameters().getMinimalSpeedPitch()
                || trajectory.getSpeed() >= minSpeed;
    }

    /** Check that the trajectory approximately does not collide with any lower obstacles. */
    private boolean checkLowerObstacles(Trajectory trajectory) {
        for (Obstacle obstacle : lowerObstacles) {
            double verticalHeightAtObstacle =
                    calculateTrajectoryHeightAtDistance(trajectory, obstacle.distance());
            if (verticalHeightAtObstacle - TrajectoryConst.PROJECTILE_RADIUS < obstacle.height) {
                return false;
            }
        }
        return true;
    }

    /** Check that the trajectory approximately does not collide with any upper obstacles. */
    private boolean checkUpperObstacles(Trajectory trajectory) {
        for (Obstacle obstacle : upperObstacles) {
            double verticalHeightAtObstacle =
                    calculateTrajectoryHeightAtDistance(trajectory, obstacle.distance());
            if (verticalHeightAtObstacle + TrajectoryConst.PROJECTILE_RADIUS > obstacle.height) {
                return false;
            }
        }
        return true;
    }

    /** Calculate the height of the fuel at the given distance along the trajectory. */
    private double calculateTrajectoryHeightAtDistance(Trajectory trajectory, double distance) {
        double passTime =
                trajectory.getTime()
                        * distance
                        / trajectory.getParameters().getHorizontalDistance();
        double verticalLaunchVelocity = Math.sin(trajectory.getPitch()) * trajectory.getSpeed();
        double passVerticalDisplacement =
                trajectory.getParameters().getLauncherTranslation().getZ()
                        + passTime * (verticalLaunchVelocity - passTime * GRAVITY / 2);
        return passVerticalDisplacement;
    }

    /** Check that the trajectory does not exceed the maximum height, if any. */
    private boolean checkUpperMaxHeight(Trajectory trajectory) {
        if (maxHeight == Double.POSITIVE_INFINITY || trajectory.getPitch() <= 0.0) {
            return true;
        }
        double verticalLaunchVelocity = Math.sin(trajectory.getPitch()) * trajectory.getSpeed();
        double launcherHeight = trajectory.getParameters().getLauncherTranslation().getZ();
        // Apex height by algebraic simplification or max height formula
        double maxVerticalDisplacement =
                verticalLaunchVelocity * verticalLaunchVelocity / (2 * GRAVITY);
        return launcherHeight + maxVerticalDisplacement <= maxHeight;
    }

    /** Check that the launcher pitch is above the minimum allowed pitch. */
    private boolean checkLowerMinLauncherPitch(Trajectory trajectory) {
        return trajectory.getLauncherPitch() >= minLauncherPitch;
    }

    /** Check that the launcher pitch is below the maximum allowed pitch. */
    private boolean checkUpperMaxLauncherPitch(Trajectory trajectory) {
        return trajectory.getLauncherPitch() <= maxLauncherPitch;
    }

    /** Calculate the maximum pitch achievable under these constraints, up to pi / 2. */
    public double calculateMaxPitch(TrajectoryParameters parameters) {
        return Math.min(Math.PI / 2, maxLauncherPitch + parameters.getLauncherTilt());
    }

    /** Calculate the minimum pitch achievable under these constraints, down to elevation angle. */
    public double calculateMinPitch(TrajectoryParameters parameters) {
        return Math.max(
                parameters.getElevationAngle(), minLauncherPitch - parameters.getLauncherTilt());
    }

    /** Print out trajectory information and constraints. */
    public void debugPrint(Trajectory trajectory) {
        System.out.println("Debug print for: " + trajectory);
        System.out.println("all: " + checkAll(trajectory));
        System.out.println("upper: " + checkUpper(trajectory));
        System.out.println("upper max flywheel: " + checkUpperMaxFlywheelSpeed(trajectory));
        System.out.println("upper max launcher pitch: " + checkUpperMaxLauncherPitch(trajectory));
        System.out.println("upper obstacles: " + checkUpperObstacles(trajectory));
        System.out.println("upper min speed: " + checkUpperMinSpeed(trajectory));
        System.out.println("upper max height: " + checkUpperMaxHeight(trajectory));
        System.out.println("lower: " + checkLower(trajectory));
        System.out.println("lower max flywheel: " + checkLowerMaxFlywheelSpeed(trajectory));
        System.out.println("lower min launcher pitch: " + checkLowerMinLauncherPitch(trajectory));
        System.out.println("lower obstacles: " + checkLowerObstacles(trajectory));
        System.out.println("lower min speed: " + checkLowerMinSpeed(trajectory));
    }

    /** Get where the pitch should try to be optimized towards. */
    public SoftConstraint getSoftConstraint() {
        return softConstraint;
    }

    /** Set where the pitch should try to be optimized towards. */
    public TrajectoryConstraints withSoftConstraint(SoftConstraint softConstraint) {
        this.softConstraint = softConstraint;
        return this;
    }

    /** Get the minimum allowable launcher pitch relative to the robot. */
    public double getMinLauncherPitch() {
        return minLauncherPitch;
    }

    /** Set the minimum allowable launcher pitch relative to the robot. */
    public TrajectoryConstraints withMinLauncherPitch(double minLauncherPitch) {
        this.minLauncherPitch = minLauncherPitch;
        return this;
    }

    /** Get the maximum allowable launcher pitch relative to the robot. */
    public double getMaxLauncherPitch() {
        return maxLauncherPitch;
    }

    /** Set the maximum allowable launcher pitch relative to the robot. */
    public TrajectoryConstraints withMaxLauncherPitch(double maxLauncherPitch) {
        this.maxLauncherPitch = maxLauncherPitch;
        return this;
    }

    /** Get the maximum allowable shooter flywheel motor angular velocity. */
    public double getMaxFlywheelSpeed() {
        return maxFlywheelSpeed;
    }

    /** Set the maximum allowable shooter flywheel motor angular velocity. */
    public TrajectoryConstraints withMaxFlywheelSpeed(double maxFlywheelSpeed) {
        this.maxFlywheelSpeed = maxFlywheelSpeed;
        return this;
    }

    /** Get the minimum allowable launch speed. */
    public double getMinSpeed() {
        return minSpeed;
    }

    /** Set the minimum allowable launch speed. */
    public TrajectoryConstraints withMinSpeed(double minSpeed) {
        this.minSpeed = minSpeed;
        return this;
    }

    /** Get the minimum allowable duration of a trajectory. */
    public double getMinTime() {
        return minTime;
    }

    /** Set the minimum allowable duration of a trajectory. */
    public TrajectoryConstraints withMinTime(double minTime) {
        this.minTime = minTime;
        return this;
    }

    /** Get the maximum allowable duration of a trajectory. */
    public double getMaxTime() {
        return maxTime;
    }

    /** Set the maximum allowable duration of a trajectory. */
    public TrajectoryConstraints withMaxTime(double maxTime) {
        this.maxTime = maxTime;
        return this;
    }

    /** Get the list of lower obstacles. */
    public ArrayList<Obstacle> getLowerObstacles() {
        return lowerObstacles;
    }

    /** Add a lower obstacle. */
    public TrajectoryConstraints withLowerObstacle(Obstacle lowerObstacle) {
        lowerObstacles.add(lowerObstacle);
        return this;
    }

    /** Add a lower obstacle using its distance and height. */
    public TrajectoryConstraints withLowerObstacle(double distance, double height) {
        return withLowerObstacle(new Obstacle(distance, height));
    }

    /** Get the list of upper obstacles. */
    public ArrayList<Obstacle> getUpperObstacles() {
        return upperObstacles;
    }

    /** Add an upper obstacle. */
    public TrajectoryConstraints withUpperObstacle(Obstacle upperObstacle) {
        upperObstacles.add(upperObstacle);
        return this;
    }

    /** Add an upper obstacle using its distance and height. */
    public TrajectoryConstraints withUpperObstacle(double distance, double height) {
        return withUpperObstacle(new Obstacle(distance, height));
    }

    /** Get the maximum allowable max height of a trajectory. */
    public double getMaxHeight() {
        return maxHeight;
    }

    /** Set the maximum allowable max height of a trajectory. */
    public TrajectoryConstraints withMaxHeight(double maxHeight) {
        this.maxHeight = maxHeight;
        return this;
    }

    /** Get the target pitch. Only applies for when the soft constraint is TARGET_PITCH. */
    public double getTargetPitch() {
        return targetPitch;
    }

    /** Set the target pitch. Only applies for when the soft constraint is TARGET_PITCH. */
    public TrajectoryConstraints withTargetPitch(double targetPitch) {
        this.targetPitch = targetPitch;
        return this;
    }
}
