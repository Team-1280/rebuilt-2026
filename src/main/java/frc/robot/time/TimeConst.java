package frc.robot.time;

/** Constants and enums for match time, timeframes, and periods. All time is in seconds. */
public final class TimeConst {
    /** After a hub deactivates, there is a 3 second delay where the hub still scores fuel. */
    public static final double HUB_SCORING_DEACTIVATION_DELAY = 3;

    /** The match period of the match. Each period has separate match times. */
    public static enum Period {
        NONE(0, Timeframe.NONE),
        AUTO(20, Timeframe.AUTO_ENABLED, Timeframe.SWITCH_DELAY),
        TELEOP(
                140,
                Timeframe.TRANSITION_SHIFT,
                Timeframe.SHIFT_1,
                Timeframe.SHIFT_2,
                Timeframe.SHIFT_3,
                Timeframe.SHIFT_4,
                Timeframe.END_GAME,
                Timeframe.POST_MATCH);

        /** The match time when the period is started. */
        public final double matchTimeStart;

        /** The ordered list of timeframes that the period goes through. */
        public final Timeframe[] timeframes;

        Period(double matchTimeStart, Timeframe... timeframes) {
            this.matchTimeStart = matchTimeStart;
            this.timeframes = timeframes;
        }
    }

    /** The match timeframe within a match period, such as Shift 1. */
    public static enum Timeframe {
        NONE(Double.NEGATIVE_INFINITY),
        AUTO_ENABLED(0),
        SWITCH_DELAY(-3), // A 3 second transition between auto and teleop
        TRANSITION_SHIFT(130),
        SHIFT_1(105),
        SHIFT_2(80),
        SHIFT_3(55),
        SHIFT_4(30),
        END_GAME(0),
        POST_MATCH(Double.NEGATIVE_INFINITY);

        /** Match time at which the period ends. The instant is part of the next timeframe. */
        public final double endTime;

        Timeframe(double endTime) {
            this.endTime = endTime;
        }
    }
}
