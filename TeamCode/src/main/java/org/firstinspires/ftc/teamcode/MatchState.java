package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.core.Alliance;
import org.firstinspires.ftc.teamcode.core.Cell;
import org.firstinspires.ftc.teamcode.core.Pose2d;

/**
 * Carries what auto learned into teleop. Static fields survive between OpModes, so a timestamp
 * guards against using a pose left over from an earlier match.
 */
public final class MatchState {
    private MatchState() {}

    public static Alliance alliance = Alliance.BLUE;

    private static Pose2d pose;
    private static Cell raisedCell;
    private static long savedAtMillis;

    public static void save(Alliance alliance, Pose2d pose, Cell raisedCell) {
        MatchState.alliance = alliance;
        MatchState.pose = pose;
        MatchState.raisedCell = raisedCell;
        savedAtMillis = System.currentTimeMillis();
    }

    public static boolean hasFreshPose() {
        long ageMillis = System.currentTimeMillis() - savedAtMillis;
        return pose != null && ageMillis < Constants.RESUME_POSE_WINDOW_SECONDS * 1000;
    }

    public static Pose2d pose() {
        return pose;
    }

    public static Cell raisedCell() {
        return raisedCell;
    }
}
