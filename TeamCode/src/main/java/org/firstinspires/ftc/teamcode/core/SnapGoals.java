package org.firstinspires.ftc.teamcode.core;

import static org.firstinspires.ftc.teamcode.Constants.FLOWER_STANDOFF_IN;
import static org.firstinspires.ftc.teamcode.Constants.OFF_AXIS_COST_IN_PER_RAD;
import static org.firstinspires.ftc.teamcode.Constants.POKER_FORWARD_IN;
import static org.firstinspires.ftc.teamcode.Constants.POKER_ON_LEFT;
import static org.firstinspires.ftc.teamcode.Constants.ROBOT_LENGTH_IN;
import static org.firstinspires.ftc.teamcode.Constants.SHOT_BAND_IN;
import static org.firstinspires.ftc.teamcode.Constants.SHOT_DISTANCE_IN;
import static org.firstinspires.ftc.teamcode.Constants.SHOT_SECTOR_DEG;
import static org.firstinspires.ftc.teamcode.Constants.TURN_COST_IN_PER_RAD;

/** Works out where the driver most likely wants to be. */
public final class SnapGoals {
    private SnapGoals() {}

    private static final int CANDIDATES_PER_SIDE = 3;

    /**
     * The best place to shoot at {@code cell} from. Candidates sit on an arc in front of the opening
     * and are ranked by how far away they are, how much turning they need and how straight-on the
     * shot is. If the robot is already in range it stays put and only turns to aim.
     *
     * @param ownSideOnly stay on our half of the field, as G402 asks during auto
     */
    public static Pose2d bestShot(Pose2d robot, Cell cell, boolean ownSideOnly) {
        Pose2d target = cell.position();
        double sector = Math.toRadians(SHOT_SECTOR_DEG);
        double robotOffAxis = offAxis(robot, cell);
        if (Math.abs(robotOffAxis) <= sector
                && Math.abs(robot.distanceTo(target) - SHOT_DISTANCE_IN) <= SHOT_BAND_IN
                && inBounds(robot, cell, ownSideOnly)) {
            return aimedFrom(robot.x, robot.y, cell);
        }

        Pose2d best = null;
        double bestCost = Double.POSITIVE_INFINITY;
        for (int i = -CANDIDATES_PER_SIDE; i <= CANDIDATES_PER_SIDE; i++) {
            double angle = sector * i / CANDIDATES_PER_SIDE;
            Pose2d candidate = aimedFrom(
                    target.x + cell.outward * SHOT_DISTANCE_IN * Math.cos(angle),
                    target.y + SHOT_DISTANCE_IN * Math.sin(angle),
                    cell);
            if (!inBounds(candidate, cell, ownSideOnly)) {
                continue;
            }
            double cost = RoutePlanner.length(robot, RoutePlanner.route(robot, candidate))
                    + TURN_COST_IN_PER_RAD * Math.abs(Pose2d.wrap(candidate.heading - robot.heading))
                    + OFF_AXIS_COST_IN_PER_RAD * Math.abs(angle);
            if (cost < bestCost) {
                bestCost = cost;
                best = candidate;
            }
        }
        return best != null ? best : aimedFrom(target.x + cell.outward * SHOT_DISTANCE_IN, target.y, cell);
    }

    /** Beside the nearest Flower, parallel to the wall, with the poker over the Flower. */
    public static Pose2d nearestFlower(Pose2d robot) {
        Pose2d best = null;
        double bestLength = Double.POSITIVE_INFINITY;
        for (Pose2d flower : Field.flowers()) {
            Pose2d candidate = beside(flower);
            double length = RoutePlanner.length(robot, RoutePlanner.route(robot, candidate));
            if (length < bestLength) {
                bestLength = length;
                best = candidate;
            }
        }
        return best;
    }

    /** In the Loading Zone, square to the wall, whichever way round needs less turning. */
    public static Pose2d park(Pose2d robot, Alliance alliance) {
        Pose2d spot = Field.parkPosition(alliance);
        double upError = Math.abs(Pose2d.wrap(Math.PI / 2 - robot.heading));
        return new Pose2d(spot.x, spot.y, upError < Math.PI / 2 ? Math.PI / 2 : -Math.PI / 2);
    }

    /** Straight-line distance from robot center to the opening, ignoring height. */
    public static double shotDistance(Pose2d robot, Cell cell) {
        return robot.distanceTo(cell.position());
    }

    private static Pose2d aimedFrom(double x, double y, Cell cell) {
        double toCell = Math.atan2(cell.y() - y, cell.x() - x);
        return new Pose2d(x, y, Field.launcherAimedAlong(toCell));
    }

    private static double offAxis(Pose2d robot, Cell cell) {
        return Math.atan2(robot.y - cell.y(), cell.outward * (robot.x - cell.x()));
    }

    private static boolean inBounds(Pose2d pose, Cell cell, boolean ownSideOnly) {
        double limit = Field.HALF - ROBOT_LENGTH_IN / 2;
        boolean onField = Math.abs(pose.x) <= limit && Math.abs(pose.y) <= limit;
        // The center line belongs to neither half.
        boolean ownHalf = cell.alliance == Alliance.RED ? pose.y < 0 : pose.y > 0;
        return onField && (!ownSideOnly || ownHalf);
    }

    private static Pose2d beside(Pose2d flower) {
        boolean onXWall = Math.abs(flower.x) > Math.abs(flower.y);
        double wallDirection = onXWall
                ? (flower.x > 0 ? 0 : Math.PI)
                : (flower.y > 0 ? Math.PI / 2 : -Math.PI / 2);
        // Heading parallel to the wall that puts the poker side toward it.
        double heading = wallDirection + (POKER_ON_LEFT ? -Math.PI / 2 : Math.PI / 2);
        double x = flower.x - FLOWER_STANDOFF_IN * Math.cos(wallDirection) - POKER_FORWARD_IN * Math.cos(heading);
        double y = flower.y - FLOWER_STANDOFF_IN * Math.sin(wallDirection) - POKER_FORWARD_IN * Math.sin(heading);
        return new Pose2d(x, y, heading);
    }
}
