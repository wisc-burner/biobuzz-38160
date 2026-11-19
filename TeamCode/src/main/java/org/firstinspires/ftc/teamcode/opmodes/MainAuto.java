package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Snap;
import org.firstinspires.ftc.teamcode.core.Alliance;
import org.firstinspires.ftc.teamcode.core.Field;
import org.firstinspires.ftc.teamcode.core.Pose2d;
import org.firstinspires.ftc.teamcode.core.ShotTable;
import org.firstinspires.ftc.teamcode.core.SnapGoals;

/**
 * Shoots the 4 pre-loaded Pollen, optionally runs the Garden for 4 more, then parks.
 * LEAVE is 3 points, an auto PARK is 5, and a Tip is 20.
 *
 * Choose during INIT:
 *   B / X           red / blue
 *   Dpad up / down  start in line with the raised Cell (near) / against the opposite wall (far)
 *   Bumpers         start delay, down / up, in seconds
 *   Dpad left/right Garden run off / on
 */
@Autonomous(name = "Main Auto", group = "Match", preselectTeleOp = "Main TeleOp")
public class MainAuto extends LinearOpMode {
    private static final double DRIVE_TIMEOUT_SECONDS = 8;

    private Robot robot;
    private final ElapsedTime matchTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        Alliance alliance = Alliance.RED;
        boolean near = true;
        int delaySeconds = 0;
        boolean gardenRun = false;

        robot = new Robot(hardwareMap, alliance);
        while (opModeInInit()) {
            if (gamepad1.bWasPressed()) alliance = Alliance.RED;
            if (gamepad1.xWasPressed()) alliance = Alliance.BLUE;
            if (gamepad1.dpadUpWasPressed()) near = true;
            if (gamepad1.dpadDownWasPressed()) near = false;
            if (gamepad1.rightBumperWasPressed()) delaySeconds = Math.min(delaySeconds + 1, 15);
            if (gamepad1.leftBumperWasPressed()) delaySeconds = Math.max(delaySeconds - 1, 0);
            if (gamepad1.dpadLeftWasPressed()) gardenRun = false;
            if (gamepad1.dpadRightWasPressed()) gardenRun = true;

            robot.alliance = alliance;
            robot.estimator.setPose(Field.startPose(alliance, near), true);
            robot.update();

            telemetry.addData("Alliance", alliance);
            telemetry.addData("Start", near ? "near raised Cell" : "far wall");
            telemetry.addData("Delay", "%d s", delaySeconds);
            telemetry.addData("Garden run", gardenRun);
            telemetry.addData("Tags in view", robot.sightings.size());
            telemetry.update();
        }
        if (isStopRequested()) {
            return;
        }

        matchTimer.reset();
        robot.poker.up();
        robot.snap.setOwnSideOnly(true);
        pause(delaySeconds);

        scoreFromBestSpot(4);
        if (gardenRun) {
            collectGarden();
            scoreFromBestSpot(4);
        }
        park();
    }

    private void scoreFromBestSpot(int pollen) {
        if (mustPark()) {
            return;
        }
        robot.shooter.spin(ShotTable.atShotSpot()); // Arrive already up to speed.
        ElapsedTime timer = new ElapsedTime();
        while (tick() && !mustPark() && timer.seconds() < DRIVE_TIMEOUT_SECONDS) {
            robot.snap.hold(Snap.Goal.SCORE);
            robot.shooter.spin(robot.shotVelocity());
            if (robot.snap.atGoal()) {
                break;
            }
        }
        robot.drive.stop();

        double feedSecondsLeft = pollen * Constants.FEED_SECONDS_PER_POLLEN;
        ElapsedTime loop = new ElapsedTime();
        while (tick() && !mustPark() && feedSecondsLeft > 0) {
            boolean feeding = robot.shooter.readyToFeed(Constants.AUTO_VELOCITY_TOLERANCE);
            robot.shooter.feed(feeding ? 1 : 0);
            if (feeding) {
                robot.intake.in();
                feedSecondsLeft -= loop.seconds();
            } else {
                robot.intake.stop();
            }
            loop.reset();
        }
        robot.snap.release();
        robot.shooter.feed(0);
        robot.intake.stop();
        robot.shooter.stopFlywheel();
    }

    private void collectGarden() {
        Pose2d[] run = Field.gardenRun(robot.alliance);
        driveTo(run[0]);
        robot.intake.in();
        driveTo(run[1]);
        robot.intake.stop();
    }

    private void park() {
        robot.snap.release();
        robot.shooter.stopFlywheel();
        driveTo(SnapGoals.park(robot.pose(), robot.alliance));
        robot.drive.stop();
        while (tick()) {
            idle();
        }
    }

    private void driveTo(Pose2d destination) {
        robot.snap.driveTo(destination);
        ElapsedTime timer = new ElapsedTime();
        while (tick() && timer.seconds() < DRIVE_TIMEOUT_SECONDS) {
            if (robot.snap.updateRoute()) {
                break;
            }
        }
        robot.drive.stop();
    }

    private void pause(int seconds) {
        ElapsedTime timer = new ElapsedTime();
        while (tick() && timer.seconds() < seconds) {
            idle();
        }
    }

    private boolean mustPark() {
        return matchTimer.seconds() > Constants.AUTO_PARK_BY_SECONDS;
    }

    /** Runs at the top of every auto loop, and keeps teleop's starting pose up to date. */
    private boolean tick() {
        if (!opModeIsActive()) {
            return false;
        }
        robot.update();
        robot.saveMatchState();
        telemetry.addData("Time", "%.1f", matchTimer.seconds());
        robot.addTelemetry(telemetry);
        telemetry.update();
        return true;
    }
}
