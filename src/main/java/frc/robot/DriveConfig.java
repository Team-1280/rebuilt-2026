package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public final class DriveConfig implements Sendable {
    /** Enable or disable driving. */
    public static boolean enableDriving = true;

    /** Maximum throttle linear drive speed. */
    public static LinearVelocity maxSpeed = MetersPerSecond.of(1.6);

    /** Maximum throttle angular drive speed. */
    public static AngularVelocity maxAngularSpeed = RotationsPerSecond.of(0.5);

    /** Minimum registerable linear drive speed. */
    public static LinearVelocity speedDeadband = maxSpeed.times(0.1);

    /** Minimum registerable angular drive speed. */
    public static AngularVelocity angularSpeedDeadband = maxAngularSpeed.times(0.1);

    /** Maximum X distance from trench bar to center of robot to automatically stow launcher. */
    public static Distance trenchLauncherStowDistance = Meters.of(1.5); // TODO: tune

    private DriveConfig() {}

    public static Sendable getSendable() {
        return new DriveConfig();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addBooleanProperty(
                "ENABLE DRIVING",
                () -> enableDriving,
                (enable) -> {
                    enableDriving = enable;
                });
        builder.addDoubleProperty(
                "max speed (m per s)",
                () -> maxSpeed.in(MetersPerSecond),
                (speed) -> {
                    maxSpeed = MetersPerSecond.of(speed);
                });
        builder.addDoubleProperty(
                "max angular speed (rot per s)",
                () -> maxAngularSpeed.in(RotationsPerSecond),
                (angularSpeed) -> {
                    maxAngularSpeed = RotationsPerSecond.of(angularSpeed);
                });
        builder.addDoubleProperty(
                "speed deadband (m per s)",
                () -> speedDeadband.in(MetersPerSecond),
                (deadband) -> {
                    speedDeadband = MetersPerSecond.of(deadband);
                });
        builder.addDoubleProperty(
                "angular speed deadband (rot per s)",
                () -> angularSpeedDeadband.in(RotationsPerSecond),
                (deadband) -> {
                    angularSpeedDeadband = RotationsPerSecond.of(deadband);
                });
        builder.addDoubleProperty(
                "trench launcher stow x distance (m)",
                () -> trenchLauncherStowDistance.in(Meters),
                (x) -> {
                    trenchLauncherStowDistance = Meters.of(x);
                });
    }
}
