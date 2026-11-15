package org.firstinspires.ftc.teamcode.core;

import org.firstinspires.ftc.teamcode.Constants;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

/**
 * Finds the shortest obstacle-free route using a handful of fixed waypoints (Dijkstra's algorithm).
 *
 * This matters because each Tip moves the target to the far side of the Hive. A straight line there
 * can clip a frame leg.
 */
public final class RoutePlanner {
    private RoutePlanner() {}

    /** Returns the points to drive through, ending at {@code goal}. */
    public static List<Pose2d> route(Pose2d start, Pose2d goal) {
        List<double[]> obstacles = Field.obstacles();
        List<Pose2d> nodes = new ArrayList<>();
        nodes.add(start);
        nodes.add(goal);
        nodes.addAll(Field.routeWaypoints());

        int count = nodes.size();
        double[] distance = new double[count];
        int[] previous = new int[count];
        boolean[] done = new boolean[count];
        Arrays.fill(distance, Double.POSITIVE_INFINITY);
        Arrays.fill(previous, -1);
        distance[0] = 0;

        // A robot already hugging an obstacle may still leave, and a goal beside one may be reached.
        double startClearance = allowedClearance(start, obstacles);
        double goalClearance = allowedClearance(goal, obstacles);

        for (int step = 0; step < count; step++) {
            int current = closestUnvisited(distance, done);
            if (current < 0) {
                break;
            }
            done[current] = true;
            for (int next = 0; next < count; next++) {
                if (done[next]) {
                    continue;
                }
                double clearance = Constants.ROUTE_CLEARANCE_IN;
                if (current == 0) {
                    clearance = Math.min(clearance, startClearance);
                }
                if (next == 1) {
                    clearance = Math.min(clearance, goalClearance);
                }
                if (!isClear(nodes.get(current), nodes.get(next), obstacles, clearance)) {
                    continue;
                }
                double candidate = distance[current] + nodes.get(current).distanceTo(nodes.get(next));
                if (candidate < distance[next]) {
                    distance[next] = candidate;
                    previous[next] = current;
                }
            }
        }

        if (previous[1] < 0) {
            return Collections.singletonList(goal); // Boxed in: try directly, the driver is watching.
        }
        List<Pose2d> route = new ArrayList<>();
        for (int node = 1; node != 0; node = previous[node]) {
            route.add(0, nodes.get(node));
        }
        return route;
    }

    public static double length(Pose2d start, List<Pose2d> route) {
        double total = 0;
        Pose2d from = start;
        for (Pose2d point : route) {
            total += from.distanceTo(point);
            from = point;
        }
        return total;
    }

    public static boolean isClear(Pose2d a, Pose2d b, List<double[]> obstacles, double clearance) {
        for (double[] o : obstacles) {
            if (segmentDistance(a.x, a.y, b.x, b.y, o[0], o[1], o[2], o[3]) < clearance) {
                return false;
            }
        }
        return true;
    }

    private static int closestUnvisited(double[] distance, boolean[] done) {
        int best = -1;
        for (int i = 0; i < distance.length; i++) {
            if (!done[i] && distance[i] < Double.POSITIVE_INFINITY && (best < 0 || distance[i] < distance[best])) {
                best = i;
            }
        }
        return best;
    }

    private static double allowedClearance(Pose2d point, List<double[]> obstacles) {
        double nearest = Double.POSITIVE_INFINITY;
        for (double[] o : obstacles) {
            nearest = Math.min(nearest, pointToSegment(point.x, point.y, o[0], o[1], o[2], o[3]));
        }
        return Math.max(0, nearest - 0.5);
    }

    static double segmentDistance(double ax, double ay, double bx, double by,
                                  double cx, double cy, double dx, double dy) {
        if (segmentsCross(ax, ay, bx, by, cx, cy, dx, dy)) {
            return 0;
        }
        return Math.min(
                Math.min(pointToSegment(ax, ay, cx, cy, dx, dy), pointToSegment(bx, by, cx, cy, dx, dy)),
                Math.min(pointToSegment(cx, cy, ax, ay, bx, by), pointToSegment(dx, dy, ax, ay, bx, by)));
    }

    private static boolean segmentsCross(double ax, double ay, double bx, double by,
                                         double cx, double cy, double dx, double dy) {
        double d1 = cross(ax, ay, bx, by, cx, cy);
        double d2 = cross(ax, ay, bx, by, dx, dy);
        double d3 = cross(cx, cy, dx, dy, ax, ay);
        double d4 = cross(cx, cy, dx, dy, bx, by);
        return d1 * d2 < 0 && d3 * d4 < 0;
    }

    private static double cross(double ax, double ay, double bx, double by, double px, double py) {
        return (bx - ax) * (py - ay) - (by - ay) * (px - ax);
    }

    private static double pointToSegment(double px, double py, double ax, double ay, double bx, double by) {
        double dx = bx - ax;
        double dy = by - ay;
        double lengthSquared = dx * dx + dy * dy;
        double t = lengthSquared == 0 ? 0 : ((px - ax) * dx + (py - ay) * dy) / lengthSquared;
        t = Math.max(0, Math.min(1, t));
        return Math.hypot(px - (ax + t * dx), py - (ay + t * dy));
    }
}
