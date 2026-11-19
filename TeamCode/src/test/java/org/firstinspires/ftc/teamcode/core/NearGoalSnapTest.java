package org.firstinspires.ftc.teamcode.core;

import static org.junit.Assert.assertTrue;

import org.junit.Test;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Random;

/**
 * Snapping from just beside the shot spot, with late camera frames and a robot that turns slowly.
 * The robot used to turn in place forever here: each frame moved its idea of where it was by an
 * inch or two, which swung the direction to a goal a few inches away, so it never finished turning.
 */
public class NearGoalSnapTest {
    private static final double DT = 0.02;
    private static final double CAMERA_LATE_SEC = 0.2;
    private static final double TOP_SPEED_IN_PER_SEC = 59;
    private static final double TOP_TURN_RAD_PER_SEC = 3;
    private static final double MOTOR_LAG_SEC = 0.12;
    private static final double FRICTION_POWER = 0.05;

    @Test
    public void neverSpinsInPlaceNearTheShotSpot() {
        Cell cell = Cell.BLUE_AUDIENCE;
        Pose2d spot = SnapGoals.bestShot(new Pose2d(60, 40, 0), cell, false);
        for (double dx = -8; dx <= 8; dx += 2) {
            for (double dy = -8; dy <= 8; dy += 2) {
                for (double degrees = 0; degrees < 360; degrees += 45) {
                    Pose2d start = new Pose2d(spot.x + dx, spot.y + dy, Math.toRadians(degrees));
                    snap(start, cell);
                }
            }
        }
    }

    private static void snap(Pose2d start, Cell cell) {
        Random noise = new Random(7);
        double x = start.x, y = start.y, heading = start.heading;
        double speed = 0, turnRate = 0, distance = 0, turned = 0;
        PoseEstimator estimator = new PoseEstimator();
        HiveTracker hive = new HiveTracker();
        hive.setRaised(cell);
        estimator.update(0, 0, 0);
        estimator.setPose(start, true);
        DriveController controller = new DriveController();
        Pose2d goal = SnapGoals.bestShot(start, cell, false);
        controller.follow(start, RoutePlanner.route(start, goal));

        List<Pose2d> truthLog = new ArrayList<>();
        int framesLate = (int) Math.round(CAMERA_LATE_SEC / DT);
        for (int step = 1; step <= 8 / DT; step++) {
            double time = step * DT;
            estimator.update(time, distance, heading - start.heading);
            truthLog.add(new Pose2d(x, y, heading));
            if (step % 3 == 0 && truthLog.size() > framesLate) {
                Pose2d seen = truthLog.get(truthLog.size() - 1 - framesLate);
                Sighting clean = TagLocalizerTest.sightingFrom(seen, cell, time - CAMERA_LATE_SEC);
                if (clean.cameraY > 5) { // In front of the lens.
                    Sighting noisy = new Sighting(cell,
                            clean.cameraX + noise.nextGaussian() * 0.4,
                            clean.cameraY + noise.nextGaussian() * 0.4,
                            clean.cameraZ + noise.nextGaussian() * 0.4,
                            0, 100, clean.frameTime);
                    TagLocalizer.apply(Collections.singletonList(noisy), estimator, hive);
                }
            }

            DriveController.Command command = controller.update(estimator.pose(), DT);
            speed += (rate(command.forward, TOP_SPEED_IN_PER_SEC) - speed) * DT / MOTOR_LAG_SEC;
            turnRate += (rate(command.turn, TOP_TURN_RAD_PER_SEC) - turnRate) * DT / MOTOR_LAG_SEC;
            x += speed * Math.cos(heading) * DT;
            y += speed * Math.sin(heading) * DT;
            heading = Pose2d.wrap(heading + turnRate * DT);
            turned += Math.abs(turnRate * DT);
            distance += speed * DT;
        }
        assertTrue("never arrived from " + start, controller.atGoal());
        assertTrue("turned " + Math.round(Math.toDegrees(turned)) + " degrees from " + start,
                turned < 2 * Math.PI);
    }

    private static double rate(double power, double top) {
        return Math.abs(power) < FRICTION_POWER ? 0 : power * top;
    }
}
