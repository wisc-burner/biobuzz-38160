package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.core.Cell;
import org.firstinspires.ftc.teamcode.core.DriveController;
import org.firstinspires.ftc.teamcode.core.Pose2d;
import org.firstinspires.ftc.teamcode.core.RoutePlanner;
import org.firstinspires.ftc.teamcode.core.SnapGoals;

/**
 * Self-driving to a useful spot. Teleop calls {@link #hold} every loop while a button is down and
 * {@link #release} when it is not, so letting go always hands control straight back to the driver.
 * Auto uses the same methods.
 */
public final class Snap {
    public enum Goal { NONE, SCORE, FLOWER, PARK }

    private final Robot robot;
    private final DriveController controller = new DriveController();
    private final ElapsedTime loopTimer = new ElapsedTime();

    private Goal goal = Goal.NONE;
    private Cell lockedCell;
    private boolean ownSideOnly;
    private boolean blocked;

    Snap(Robot robot) {
        this.robot = robot;
    }

    /** Stay on our half of the field when picking a shot spot. Used in auto for G402. */
    public void setOwnSideOnly(boolean ownSideOnly) {
        this.ownSideOnly = ownSideOnly;
    }

    /**
     * Drives one loop toward {@code wanted}. The destination is chosen once when the hold begins,
     * so it cannot flicker between spots, and is re-chosen if the Hive tips.
     *
     * @return false if the pose cannot be trusted, in which case the driver keeps the wheels
     */
    public boolean hold(Goal wanted) {
        blocked = !robot.estimator.isTrusted();
        if (blocked) {
            goal = Goal.NONE;
            return false;
        }
        boolean targetMoved = wanted == Goal.SCORE && lockedCell != robot.targetCell();
        if (wanted != goal || targetMoved) {
            begin(wanted);
        }
        double dt = Math.min(loopTimer.seconds(), 0.1);
        loopTimer.reset();
        DriveController.Command command = controller.update(robot.pose(), dt);
        robot.drive.arcade(command.forward, command.turn);
        return true;
    }

    public void release() {
        goal = Goal.NONE;
        blocked = false;
    }

    public boolean isDriving() {
        return goal != Goal.NONE;
    }

    public boolean atGoal() {
        return goal != Goal.NONE && controller.atGoal();
    }

    public void driveTo(Pose2d destination) {
        goal = Goal.NONE;
        startRoute(destination);
    }

    /** One loop of a {@link #driveTo} route. */
    public boolean updateRoute() {
        double dt = Math.min(loopTimer.seconds(), 0.1);
        loopTimer.reset();
        DriveController.Command command = controller.update(robot.pose(), dt);
        robot.drive.arcade(command.forward, command.turn);
        return controller.atGoal();
    }

    public String status() {
        if (blocked) {
            return "NO POSE: show the camera a Cell";
        }
        if (goal == Goal.NONE) {
            return "off";
        }
        return goal + (controller.atGoal() ? " ready" : " driving");
    }

    private void begin(Goal wanted) {
        goal = wanted;
        lockedCell = robot.targetCell();
        startRoute(destinationFor(wanted));
    }

    private Pose2d destinationFor(Goal wanted) {
        Pose2d pose = robot.pose();
        switch (wanted) {
            case FLOWER: return SnapGoals.nearestFlower(pose);
            case PARK: return SnapGoals.park(pose, robot.alliance);
            default: return SnapGoals.bestShot(pose, lockedCell, ownSideOnly);
        }
    }

    private void startRoute(Pose2d destination) {
        loopTimer.reset();
        controller.follow(robot.pose(), RoutePlanner.route(robot.pose(), destination));
    }
}
