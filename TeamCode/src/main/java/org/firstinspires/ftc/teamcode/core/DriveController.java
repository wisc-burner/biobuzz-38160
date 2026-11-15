package org.firstinspires.ftc.teamcode.core;

import static org.firstinspires.ftc.teamcode.Constants.AUTO_DRIVE_KP;
import static org.firstinspires.ftc.teamcode.Constants.AUTO_MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.Constants.AUTO_MAX_DRIVE_POWER;
import static org.firstinspires.ftc.teamcode.Constants.AUTO_MAX_TURN_POWER;
import static org.firstinspires.ftc.teamcode.Constants.AUTO_MIN_DRIVE_POWER;
import static org.firstinspires.ftc.teamcode.Constants.AUTO_MIN_TURN_POWER;
import static org.firstinspires.ftc.teamcode.Constants.AUTO_TURN_KP;
import static org.firstinspires.ftc.teamcode.Constants.HEADING_TOLERANCE_DEG;
import static org.firstinspires.ftc.teamcode.Constants.MAX_DRIVE_HEADING_ERROR_DEG;
import static org.firstinspires.ftc.teamcode.Constants.POSITION_TOLERANCE_IN;
import static org.firstinspires.ftc.teamcode.Constants.WAYPOINT_TOLERANCE_IN;

import java.util.Collections;
import java.util.List;

/**
 * Drives a tank robot along a route, then turns to the final heading.
 *
 * A tank robot cannot strafe, so the plan is always: point at the next waypoint, drive, and rotate
 * at the end. It drives whichever way round (forwards or backwards) needs the least turning.
 */
public final class DriveController {
    /** Powers for {@code Drive.arcade}. Positive turn is counter-clockwise. */
    public static final class Command {
        public final double forward;
        public final double turn;

        Command(double forward, double turn) {
            this.forward = forward;
            this.turn = turn;
        }
    }

    private static final double HEADING_TOLERANCE = Math.toRadians(HEADING_TOLERANCE_DEG);
    private static final double MAX_DRIVE_HEADING_ERROR = Math.toRadians(MAX_DRIVE_HEADING_ERROR_DEG);
    private static final double FLIP_DIRECTION_AT = Math.toRadians(110);
    /** Pushed further than tolerance * this while settling: go back to driving. */
    private static final double RESUME_FACTOR = 2;

    private List<Pose2d> route = Collections.emptyList();
    private int leg;
    private boolean reversing;
    private boolean settling;
    private boolean atGoal;
    private double lastForward;

    public void follow(Pose2d robot, List<Pose2d> newRoute) {
        route = newRoute;
        leg = 0;
        settling = false;
        atGoal = false;
        chooseDirection(robot);
    }

    /** True once the robot is on the final pose. Stays live: a shove clears it until recovered. */
    public boolean atGoal() {
        return atGoal;
    }

    public Command update(Pose2d robot, double dt) {
        if (route.isEmpty()) {
            return new Command(0, 0);
        }
        Pose2d goal = route.get(route.size() - 1);
        if (leg < route.size() - 1 && robot.distanceTo(route.get(leg)) < WAYPOINT_TOLERANCE_IN) {
            leg++;
            chooseDirection(robot);
        }

        boolean lastLeg = leg == route.size() - 1;
        double distanceToGoal = robot.distanceTo(goal);
        if (lastLeg && distanceToGoal < POSITION_TOLERANCE_IN) {
            settling = true;
        } else if (settling && distanceToGoal > POSITION_TOLERANCE_IN * RESUME_FACTOR) {
            settling = false;
            chooseDirection(robot);
        }

        if (settling) {
            double headingError = Pose2d.wrap(goal.heading - robot.heading);
            atGoal = Math.abs(headingError) < HEADING_TOLERANCE;
            lastForward = 0;
            return new Command(0, atGoal ? 0 : turnPower(headingError));
        }
        atGoal = false;

        Pose2d target = route.get(leg);
        double headingError = travelError(robot, target);
        if (Math.abs(headingError) > FLIP_DIRECTION_AT) {
            reversing = !reversing; // Overshot the target: back up rather than turn all the way round.
            headingError = travelError(robot, target);
        }

        double forward = 0;
        if (Math.abs(headingError) < MAX_DRIVE_HEADING_ERROR) {
            double remaining = RoutePlanner.length(robot, route.subList(leg, route.size()));
            forward = drivePower(remaining) * Math.cos(headingError) * (reversing ? -1 : 1);
        }
        return new Command(limitAcceleration(forward, dt), turnPower(headingError));
    }

    private double travelError(Pose2d robot, Pose2d target) {
        double travelDirection = robot.bearingTo(target) + (reversing ? Math.PI : 0);
        return Pose2d.wrap(travelDirection - robot.heading);
    }

    private void chooseDirection(Pose2d robot) {
        Pose2d target = route.get(leg);
        double bearing = robot.bearingTo(target);
        double forwardsTurn = Math.abs(Pose2d.wrap(bearing - robot.heading));
        double backwardsTurn = Math.abs(Pose2d.wrap(bearing + Math.PI - robot.heading));
        if (leg == route.size() - 1) {
            forwardsTurn += Math.abs(Pose2d.wrap(target.heading - bearing));
            backwardsTurn += Math.abs(Pose2d.wrap(target.heading - bearing - Math.PI));
        }
        reversing = backwardsTurn < forwardsTurn;
    }

    private double limitAcceleration(double forward, double dt) {
        if (Math.abs(forward) > Math.abs(lastForward)) {
            double step = AUTO_MAX_ACCEL * dt;
            forward = lastForward + Math.max(-step, Math.min(step, forward - lastForward));
        }
        lastForward = forward;
        return forward;
    }

    private static double drivePower(double inches) {
        return Math.min(AUTO_MAX_DRIVE_POWER, AUTO_MIN_DRIVE_POWER + AUTO_DRIVE_KP * inches);
    }

    private static double turnPower(double radians) {
        double power = Math.min(AUTO_MAX_TURN_POWER, AUTO_MIN_TURN_POWER + AUTO_TURN_KP * Math.abs(radians));
        return Math.copySign(power, radians);
    }
}
