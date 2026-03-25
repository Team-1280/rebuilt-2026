package frc.robot.time;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import frc.robot.time.TimeConst.Period;
import frc.robot.time.TimeConst.Timeframe;

import java.util.Optional;

/**
 * Class with utilities for shift (hub activation) times.
 *
 * <p>Assumptions:
 *
 * <p>The hub is always active if not a game match or the team alliance is unknown.
 *
 * <p>Opponent alliance is assumed to activate first when auto winner is unknown.
 *
 * <p>In post-match, both hubs are considered deactivated.
 */
public record HubStatus(boolean activated, double timeToActivation, double timeToDeactivation)
        implements Sendable {
    /** Cached first active alliance. Can be written to as an override. */
    private static Optional<Alliance> firstActiveAlliance = Optional.empty();

    /** Get the default always active hub status. */
    public HubStatus() {
        this(true, 0.0, Double.POSITIVE_INFINITY);
    }

    /** Get the alliance whose hub will first be active for the shifts. (Shifts 1 and 3). */
    private static Alliance getFirstActiveAlliance() {
        if (firstActiveAlliance.isPresent()) {
            // Use cached or dashboard-overridden alliance value
            return firstActiveAlliance.get();
        }
        // Get the alliance using the Game Data if available
        /*
         * https://docs.wpilib.org/en/stable/docs/yearly-overview/2026-game-data.html
         * "The alliance will be provided as a single character representing the color of
         * the alliance whose goal will go inactive first (i.e. ‘R’ = red, ‘B’ = blue).
         * This alliance’s goal will be active in Shifts 2 and 4."
         */
        String message = DriverStation.getGameSpecificMessage();
        switch (message) {
            case "":
            default:
                break;
            case "B":
                firstActiveAlliance = Optional.of(Alliance.Red);
                return Alliance.Red;
            case "R":
                firstActiveAlliance = Optional.of(Alliance.Blue);
                return Alliance.Blue;
        }
        // If unknown, default to assuming the opponent alliance's hub activates first
        return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                ? Alliance.Red
                : Alliance.Blue;
    }

    /** Get the status of the team alliance's hub at the current match time. */
    public static HubStatus getTeamHubStatus() {
        return getTeamHubStatus(MatchTime.getMatchTime());
    }

    /** Get the status of the team alliance's hub at the given match time. */
    public static HubStatus getTeamHubStatus(double matchTime) {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) {
            return new HubStatus();
        }
        return getAllianceHubStatus(alliance.get(), matchTime);
    }

    /** Get the status of the given alliance's hub at the current match time. */
    public static HubStatus getAllianceHubStatus(Alliance alliance) {
        return getAllianceHubStatus(alliance, MatchTime.getMatchTime());
    }

    /** Get the status of the given alliance's hub at the given match time. */
    public static HubStatus getAllianceHubStatus(Alliance alliance, double matchTime) {
        boolean isFirstActive = alliance == getFirstActiveAlliance();
        Timeframe timeframe = MatchTime.getTimeframe();
        boolean isActivated =
                switch (timeframe) {
                    case NONE, AUTO_ENABLED, SWITCH_DELAY, TRANSITION_SHIFT, END_GAME -> true;
                    case SHIFT_1, SHIFT_3 -> isFirstActive;
                    case SHIFT_2, SHIFT_4 -> !isFirstActive;
                    case POST_MATCH -> false;
                };
        double timeToActivation;
        double timeToDeactivation;
        if (isActivated) {
            timeToActivation = 0.0;
            // Last active timeframe until deactivation; the current timeframe is known as activated
            Timeframe lastActiveTimeframe =
                    switch (timeframe) {
                        case NONE -> Timeframe.NONE;
                        case AUTO_ENABLED, SWITCH_DELAY, TRANSITION_SHIFT ->
                                isFirstActive ? Timeframe.SHIFT_1 : Timeframe.TRANSITION_SHIFT;
                        case SHIFT_1, SHIFT_2, SHIFT_3 -> timeframe; // timeframe is activated
                        case SHIFT_4, END_GAME -> Timeframe.END_GAME;
                        case POST_MATCH -> Timeframe.POST_MATCH; // should be unreachable
                    };
            timeToDeactivation = matchTime - lastActiveTimeframe.endTime;
            if (timeframe == Timeframe.AUTO_ENABLED || timeframe == Timeframe.SWITCH_DELAY) {
                // matchTime was for auto period, so convert the matchTime to teleop match time
                timeToDeactivation += Period.TELEOP.matchTimeStart - Timeframe.SWITCH_DELAY.endTime;
            }
        } else {
            // Note: a deactivated timeframe is always the last one deactivated
            timeToActivation = matchTime - timeframe.endTime;
            timeToDeactivation = 0.0;
        }
        return new HubStatus(isActivated, timeToActivation, timeToDeactivation);
    }

    public static Sendable getSendable() {
        return new HubStatus();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty(
                "first active alliance (b,r,_)",
                () ->
                        firstActiveAlliance.isPresent()
                                ? firstActiveAlliance.get().toString()
                                : "unknown",
                (alliance) -> {
                    firstActiveAlliance =
                            switch (alliance.toLowerCase()) {
                                case "b" -> Optional.of(Alliance.Blue);
                                case "r" -> Optional.of(Alliance.Red);
                                default -> Optional.empty();
                            };
                });
        builder.addBooleanProperty("team activated", () -> getTeamHubStatus().activated(), null);
        builder.addDoubleProperty(
                "team activation time", () -> getTeamHubStatus().timeToActivation(), null);
        builder.addDoubleProperty(
                "team deactivation time", () -> getTeamHubStatus().timeToDeactivation(), null);
        builder.addStringProperty("alliance", () -> DriverStation.getAlliance().toString(), null);
    }
}
