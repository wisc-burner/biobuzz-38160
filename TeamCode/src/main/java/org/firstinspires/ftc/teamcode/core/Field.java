package org.firstinspires.ftc.teamcode.core;

import org.firstinspires.ftc.teamcode.Constants;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * BIOBUZZ field geometry, in the frame described on {@link Pose2d}.
 *
 * Sources: Competition Manual TU07 figures 9-2, 9-8, 9-9, 9-10 and 9-17, plus the SDK's cluster
 * layout. The manual's tolerance is +/- 1 in, and flower and zone positions were read off figures,
 * so check them on a real field.
 *
 * Why this is not an SDK tag map: the Cells physically move when a Hive tips, so the SDK leaves
 * every BIOBUZZ cluster's field position at (0, 0, 0) and detection.robotPose is not a field pose.
 */
public final class Field {
    private Field() {}

    public static final double HALF = 72;

    /** Hives are 25.5 in apart, center to center. */
    public static final double HIVE_Y = 12.75;
    /** Center of a raised Cell's opening, worked out from figures 9-9 and 9-10 at the 30 degree tilt. */
    public static final double OPENING_X = 16.45;
    public static final double OPENING_Z = 58.37;

    private static final double FRAME_LEG_Y = 24.73;
    private static final double FRAME_LEG_HALF_LENGTH = 19.5;
    private static final double FLOWER_FROM_CENTER = 68.5;
    private static final double FLOWER_ALONG_WALL = 24;

    /** Obstacles as {x1, y1, x2, y2} line segments; a point is a zero-length segment. */
    public static List<double[]> obstacles() {
        List<double[]> obstacles = new ArrayList<>();
        obstacles.add(new double[] {-FRAME_LEG_HALF_LENGTH, -FRAME_LEG_Y, FRAME_LEG_HALF_LENGTH, -FRAME_LEG_Y});
        obstacles.add(new double[] {-FRAME_LEG_HALF_LENGTH, FRAME_LEG_Y, FRAME_LEG_HALF_LENGTH, FRAME_LEG_Y});
        for (Pose2d flower : flowers()) {
            obstacles.add(new double[] {flower.x, flower.y, flower.x, flower.y});
        }
        if (!Constants.CAN_DRIVE_UNDER_HIVE) {
            obstacles.add(new double[] {0, -FRAME_LEG_Y, 0, FRAME_LEG_Y});
        }
        return obstacles;
    }

    /** Places a route may pass through to get around the Hive frame. */
    public static List<Pose2d> routeWaypoints() {
        List<Pose2d> waypoints = new ArrayList<>();
        for (double x : new double[] {-32, 32}) {
            for (double y : new double[] {-38, -HIVE_Y, HIVE_Y, 38}) {
                waypoints.add(new Pose2d(x, y, 0));
            }
        }
        waypoints.add(new Pose2d(0, -38, 0));
        waypoints.add(new Pose2d(0, 38, 0));
        return waypoints;
    }

    /** All four Flowers. Each sits against a wall, 24 in from the middle of that wall. */
    public static List<Pose2d> flowers() {
        return Arrays.asList(
                new Pose2d(-FLOWER_FROM_CENTER, -FLOWER_ALONG_WALL, 0),
                new Pose2d(FLOWER_ALONG_WALL, -FLOWER_FROM_CENTER, 0),
                new Pose2d(FLOWER_FROM_CENTER, FLOWER_ALONG_WALL, 0),
                new Pose2d(-FLOWER_ALONG_WALL, FLOWER_FROM_CENTER, 0));
    }

    /** Touching the wall with the launcher aimed down the Hive lane. */
    public static Pose2d startPose(Alliance alliance, boolean nearRaisedCell) {
        double x = Field.HALF - Constants.ROBOT_LENGTH_IN / 2;
        Pose2d red;
        if (nearRaisedCell) {
            red = new Pose2d(x, -HIVE_Y, launcherAimedAlong(Math.PI));
        } else {
            // Rear wall. y = -9.5 keeps the robot on red's half and clear of the Flower at y = -24.
            red = new Pose2d(-x, -9.5, launcherAimedAlong(0));
        }
        return alliance.fromRed(red);
    }

    /** Far enough into the Loading Zone to count: the zone starts 11 in from the wall. */
    public static Pose2d parkPosition(Alliance alliance) {
        return alliance.fromRed(new Pose2d(-36, -(HALF - 11 - Constants.ROBOT_LENGTH_IN / 2 + 3), 0));
    }

    /** Start and end of a run along the audience wall, through red's 4 Garden Pollen. */
    public static Pose2d[] gardenRun(Alliance alliance) {
        double x = HALF - Constants.ROBOT_WIDTH_IN / 2 - 1;
        return new Pose2d[] {
                alliance.fromRed(new Pose2d(x, -36, -Math.PI / 2)),
                alliance.fromRed(new Pose2d(x, -HALF + Constants.ROBOT_LENGTH_IN / 2 + 1, -Math.PI / 2)),
        };
    }

    /** Robot heading that points the launcher in the given field direction. */
    public static double launcherAimedAlong(double direction) {
        return direction + (Constants.LAUNCHER_FACES_REAR ? Math.PI : 0);
    }
}
