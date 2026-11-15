package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.core.Alliance;
import org.firstinspires.ftc.teamcode.core.Field;
import org.firstinspires.ftc.teamcode.core.Pose2d;
import org.firstinspires.ftc.teamcode.core.Sighting;
import org.firstinspires.ftc.teamcode.core.TagLocalizer;

/**
 * Bench and tuning tool. Follow the checklist in TeamCode/README.md.
 *
 *   Left stick      drive
 *   Left bumper     spin flywheel and feed when up to speed
 *   Right bumper    intake
 *   Dpad up / down  flywheel target +/- 10 ticks/s
 *   A / B           poker down / up
 *   Y               zero the inches counter
 */
@TeleOp(name = "Diagnostics", group = "Tuning")
public class Diagnostics extends LinearOpMode {
    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap, Alliance.BLUE);
        robot.estimator.setPose(Field.startPose(Alliance.BLUE, true), false);
        double flywheelTarget = Constants.DEFAULT_FLYWHEEL_VELOCITY;
        double inchesZero = 0;

        while (opModeInInit()) {
            robot.update();
            report(robot, flywheelTarget, inchesZero);
        }
        while (opModeIsActive()) {
            robot.update();
            robot.drive.arcade(-gamepad1.left_stick_y, -gamepad1.left_stick_x);

            if (gamepad1.dpadUpWasPressed()) flywheelTarget += 10;
            if (gamepad1.dpadDownWasPressed()) flywheelTarget -= 10;
            if (gamepad1.yWasPressed()) inchesZero = robot.drive.forwardInches();
            if (gamepad1.aWasPressed()) robot.poker.down();
            if (gamepad1.bWasPressed()) robot.poker.up();

            boolean feeding = false;
            if (gamepad1.left_bumper) {
                robot.shooter.spin(flywheelTarget);
                feeding = robot.shooter.readyToFeed(Constants.TELEOP_VELOCITY_TOLERANCE);
            } else {
                robot.shooter.stopFlywheel();
            }
            robot.shooter.feed(feeding ? 1 : 0);
            if (feeding || gamepad1.right_bumper) {
                robot.intake.in();
            } else {
                robot.intake.stop();
            }
            report(robot, flywheelTarget, inchesZero);
        }
    }

    private void report(Robot robot, double flywheelTarget, double inchesZero) {
        telemetry.addData("Inches driven", "%.1f", robot.drive.forwardInches() - inchesZero);
        telemetry.addData("Raw yaw (CCW +)", "%.1f deg", Math.toDegrees(robot.drive.yawRadians()));
        telemetry.addData("Flywheel target", "%.0f", flywheelTarget);
        robot.addTelemetry(telemetry);
        for (Sighting sighting : robot.sightings) {
            TagLocalizer.Target target = TagLocalizer.toRobotFrame(sighting);
            Pose2d fix = TagLocalizer.robotPosition(target, robot.pose().heading, sighting.cell);
            telemetry.addLine(String.format("%s %s roll %.0f, %d%% visible",
                    sighting.cell, sighting.isRaised() ? "RAISED" : "lowered",
                    sighting.rollDegrees, sighting.percentVisible));
            telemetry.addLine(String.format("  from robot: fwd %.1f left %.1f height %.1f (want %.1f)",
                    target.forward, target.left, target.height, Field.OPENING_Z));
            telemetry.addLine(String.format("  robot would be at %s", fix));
        }
        telemetry.update();
    }
}
