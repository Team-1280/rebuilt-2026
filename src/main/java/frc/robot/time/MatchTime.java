package frc.robot.time;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

import frc.robot.time.TimeConst.Period;
import frc.robot.time.TimeConst.Timeframe;

import java.util.Optional;

/**
 * Class with utilities for getting the match time.
 *
 * <p>If there is no match happening (type is NONE), then the match times are useless.
 */
public class MatchTime implements Sendable {
    /** The absolute time at which the match period began. */
    private static double periodStartTime = getTime();

    /** The current match period. */
    private static Period period = Period.NONE;

    /** An override for whether the robot is in a match. */
    private static Optional<Boolean> isMatchOverride = Optional.empty();

    static {
        RobotModeTriggers.autonomous()
                .onTrue(Commands.runOnce(() -> MatchTime.startPeriod(Period.AUTO)));
        RobotModeTriggers.teleop()
                .onTrue(Commands.runOnce(() -> MatchTime.startPeriod(Period.TELEOP)));
        RobotModeTriggers.test().onTrue(Commands.runOnce(() -> MatchTime.startPeriod(Period.NONE)));
    }

    /** Get whether the robot is in a match and should do match periods. */
    public static boolean isMatch() {
        if (isMatchOverride.isPresent()) {
            return isMatchOverride.get();
        }
        return DriverStation.getMatchType() != MatchType.None;
    }

    /** Get the absolute time, used to disambiguously track time. */
    private static double getTime() {
        return Timer.getTimestamp();
    }

    /** Start the current match period. If the driver station is not in a game match, this fails. */
    private static void startPeriod(Period period) {
        if (isMatch()) {
            periodStartTime = getTime();
            MatchTime.period = period;
        } else {
            // Match periods do not apply for non-matches
            MatchTime.period = Period.NONE;
        }
    }

    /** Get the current match period. */
    public static Period getPeriod() {
        return period;
    }

    /** Get the current match time for the current match period. */
    public static double getMatchTime() {
        return getPeriod().matchTimeStart - (getTime() - periodStartTime);
    }

    public static void setMatchTime(double matchTime) {
        periodStartTime = getTime() - (getPeriod().matchTimeStart - matchTime);
    }

    /** Get the current timeframe in the match period. */
    public static Timeframe getTimeframe() {
        double matchTime = MatchTime.getMatchTime();
        for (Timeframe timeframe : getPeriod().timeframes) {
            if (matchTime > timeframe.endTime) {
                return timeframe;
            }
        }
        return Timeframe.NONE;
    }

    private MatchTime() {}

    public static Sendable getSendable() {
        return new MatchTime();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("MATCH TIME", MatchTime::getMatchTime, MatchTime::setMatchTime);
        builder.addStringProperty("match period", () -> getPeriod().toString(), null);
        builder.addStringProperty("timeframe", () -> getTimeframe().toString(), null);
        builder.addBooleanProperty(
                "is match",
                MatchTime::isMatch,
                (isMatch) -> {
                    isMatchOverride = Optional.of(isMatch);
                });
    }
}
