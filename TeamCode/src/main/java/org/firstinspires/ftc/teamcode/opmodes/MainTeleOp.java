package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.MatchState;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Snap;
import org.firstinspires.ftc.teamcode.core.Alliance;
import org.firstinspires.ftc.teamcode.core.Field;
import org.firstinspires.ftc.teamcode.core.Pose2d;

/**
 * One-driver teleop. Buttons match REV's starter code, and the additions are marked.
 *
 *   Left stick      arcade drive
 *   Left trigger    NEW hold: self-drive to the best shot spot, aim, spin up
 *   Dpad left       NEW hold: self-drive beside the nearest Flower
 *   Dpad right      NEW hold: self-drive into our Loading Zone
 *   Left bumper     launch (waits for the flywheel, and for the robot if self-driving)
 *   Right bumper    intake
 *   Right trigger   outtake
 *   Y / X           flywheel only / reverse flywheel
 *   Dpad up / down  feeder forward / reverse
 *   A / B           poker down / up
 *   Back            NEW Hive tipped but the camera missed it: swap target Cell
 *   Right stick btn NEW robot square to the audience/rear walls, either way round: reset heading
 *
 * While in INIT, press B for red or X for blue. Auto's choice carries over if it ran.
 */
@TeleOp(name = "Main TeleOp", group = "Match")
public class MainTeleOp extends LinearOpMode {
    private Robot robot;
    private boolean wasReady;
    private boolean warned60;
    private boolean warned20;

    @Override
    public void runOpMode() {
        boolean resumed = MatchState.hasFreshPose();
        robot = new Robot(hardwareMap, MatchState.alliance);
        if (resumed) {
            robot.estimator.setPose(MatchState.pose(), true);
            robot.hive.setRaised(MatchState.raisedCell());
        } else {
            assumeStartingPose();
        }

        // Nothing may move here: G403 covers the auto-to-teleop gap.
        while (opModeInInit()) {
            robot.update();
            if (!resumed && gamepad1.bWasPressed()) {
                robot.alliance = Alliance.RED;
                assumeStartingPose();
            }
            if (!resumed && gamepad1.xWasPressed()) {
                robot.alliance = Alliance.BLUE;
                assumeStartingPose();
            }
            telemetry.addData("Alliance", "%s  (B = red, X = blue)", robot.alliance);
            telemetry.addData("Pose from auto", resumed);
            robot.addTelemetry(telemetry);
            telemetry.update();
        }
        if (isStopRequested()) {
            return;
        }

        resetRuntime();
        robot.poker.up();
        while (opModeIsActive()) {
            robot.update();
            drive();
            launcherAndIntake();
            poker();
            matchCues();
            robot.addTelemetry(telemetry);
            telemetry.update();
        }
    }

    /** Without auto we only know roughly where the robot is, until the camera sees a Cell. */
    private void assumeStartingPose() {
        robot.estimator.setPose(Field.startPose(robot.alliance, true), false);
    }

    private void drive() {
        Snap.Goal wanted = Snap.Goal.NONE;
        if (gamepad1.left_trigger_pressed) {
            wanted = Snap.Goal.SCORE;
        } else if (gamepad1.dpad_left) {
            wanted = Snap.Goal.FLOWER;
        } else if (gamepad1.dpad_right) {
            wanted = Snap.Goal.PARK;
        }

        if (wanted != Snap.Goal.NONE && robot.snap.hold(wanted)) {
            return;
        }
        if (wanted == Snap.Goal.NONE) {
            robot.snap.release();
        }
        double forward = shape(-gamepad1.left_stick_y);
        double turn = -shape(gamepad1.left_stick_x) * Constants.DRIVER_TURN_SCALE;
        robot.drive.arcade(forward, turn);
    }

    private void launcherAndIntake() {
        boolean snappingToScore = gamepad1.left_trigger_pressed && robot.snap.isDriving();
        boolean launch = gamepad1.left_bumper;
        double tolerance = Constants.TELEOP_VELOCITY_TOLERANCE;

        if (gamepad1.x) {
            robot.shooter.reverse();
        } else if (launch || gamepad1.y || snappingToScore) {
            robot.shooter.spin(robot.shotVelocity());
        } else {
            robot.shooter.stopFlywheel();
        }

        // While self-driving, holding launch waits until the robot has arrived and is aimed.
        boolean aimed = !snappingToScore || robot.snap.atGoal();
        boolean feeding = launch && aimed && robot.shooter.readyToFeed(tolerance);

        if (feeding || gamepad1.dpad_up) {
            robot.shooter.feed(1);
        } else if (gamepad1.dpad_down) {
            robot.shooter.feed(-1);
        } else {
            robot.shooter.feed(0);
        }

        if (feeding || gamepad1.right_bumper) {
            robot.intake.in();
        } else if (gamepad1.right_trigger_pressed) {
            robot.intake.out();
        } else {
            robot.intake.stop();
        }

        boolean ready = snappingToScore && robot.snap.atGoal() && robot.shooter.atSpeed(tolerance);
        if (ready && !wasReady) {
            gamepad1.rumble(200); // Tells the driver to fire without looking at the screen.
        }
        wasReady = ready;
    }

    private void poker() {
        if (gamepad1.aWasPressed()) {
            robot.poker.down();
        } else if (gamepad1.bWasPressed()) {
            robot.poker.up();
        }
        if (gamepad1.backWasPressed()) {
            robot.hive.flip(robot.alliance);
        }
        if (gamepad1.rightStickButtonWasPressed()) {
            // Square along the long axis, either way round. The next raised Cell the camera sees
            // tells us which way, and where we are.
            Pose2d pose = robot.pose();
            double squared = Math.abs(Pose2d.wrap(pose.heading)) < Math.PI / 2 ? 0 : Math.PI;
            robot.estimator.setPose(new Pose2d(pose.x, pose.y, squared), false);
        }
    }

    private void matchCues() {
        double left = Constants.TELEOP_SECONDS - getRuntime();
        if (left <= 60 && !warned60) {
            warned60 = true;
            gamepad1.rumbleBlips(2); // Nectar may now go into Flowers (G410).
        }
        if (left <= 20 && !warned20) {
            warned20 = true;
            gamepad1.rumbleBlips(3); // Time to head for the Loading Zone.
        }
    }

    /** Squares the stick for finer control at low speed. */
    private static double shape(double stick) {
        return Math.abs(stick) < Constants.STICK_DEADBAND ? 0 : Math.copySign(stick * stick, stick);
    }
}
