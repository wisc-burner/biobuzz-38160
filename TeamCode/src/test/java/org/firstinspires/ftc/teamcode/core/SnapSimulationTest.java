package org.firstinspires.ftc.teamcode.core;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.Constants;
import org.junit.Test;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Random;

/**
 * Drives a simulated tank robot with motor lag and static friction. This proves the logic, not the
 * tuning: the model's numbers are rough estimates of the REV robot.
 */
public class SnapSimulationTest {
    private static final double DT = 0.02;
    private static final double TOP_SPEED_IN_PER_SEC = 59;
    private static final double TOP_TURN_RAD_PER_SEC = 6;
    private static final double MOTOR_LAG_SEC = 0.12;
    private static final double FRICTION_POWER = 0.05;

    private static final class Sim {
        double x, y, heading, speed, turnRate, distance;

        Sim(Pose2d start) {
            x = start.x;
            y = start.y;
            heading = start.heading;
        }

        void step(DriveController.Command command) {
            speed += (powerToRate(command.forward, TOP_SPEED_IN_PER_SEC) - speed) * DT / MOTOR_LAG_SEC;
            turnRate += (powerToRate(command.turn, TOP_TURN_RAD_PER_SEC) - turnRate) * DT / MOTOR_LAG_SEC;
            x += speed * Math.cos(heading) * DT;
            y += speed * Math.sin(heading) * DT;
            heading = Pose2d.wrap(heading + turnRate * DT);
            distance += speed * DT;
        }

        Pose2d pose() {
            return new Pose2d(x, y, heading);
        }

        private static double powerToRate(double power, double top) {
            return Math.abs(power) < FRICTION_POWER ? 0 : power * top;
        }
    }

    /** Drives until arrival and returns the seconds taken. Fails if it hits the frame or times out. */
    private static double driveTo(Sim sim, Pose2d goal) {
        return driveTo(sim, goal, RoutePlanner.route(sim.pose(), goal), 12);
    }

    private static double driveTo(Sim sim, Pose2d goal, List<Pose2d> route, double timeLimit) {
        DriveController controller = new DriveController();
        controller.follow(sim.pose(), route);
        double time = 0;
        while (time < timeLimit) {
            sim.step(controller.update(sim.pose(), DT));
            time += DT;
            assertClearOfFrame(sim.pose());
            if (controller.atGoal() && Math.abs(sim.speed) < 2) {
                return time;
            }
        }
        throw new AssertionError("never arrived: stuck at " + sim.pose() + " heading for " + goal);
    }

    private static void assertClearOfFrame(Pose2d pose) {
        // Half the robot's width: anything closer means the side of the robot hit a frame leg.
        double margin = Constants.ROBOT_WIDTH_IN / 2;
        boolean besideLeg = Math.abs(pose.x) < 19.5 + margin;
        boolean atLeg = Math.abs(Math.abs(pose.y) - 24.73) < margin;
        assertTrue("hit Hive frame at " + pose, !(besideLeg && atLeg));
    }

    private static List<Pose2d> startGrid() {
        List<Pose2d> starts = new ArrayList<>();
        for (double x = -60; x <= 60; x += 15) {
            for (double y = -60; y <= 60; y += 15) {
                Pose2d point = new Pose2d(x, y, 0);
                boolean nearLeg = Math.abs(x) < 26 && Math.abs(Math.abs(y) - 24.73) < 11;
                if (!nearLeg) {
                    for (double degrees = 0; degrees < 360; degrees += 90) {
                        starts.add(new Pose2d(point.x, point.y, Math.toRadians(degrees)));
                    }
                }
            }
        }
        return starts;
    }

    @Test
    public void snapsToAShotFromAnywhereOnTheField() {
        for (Cell cell : Cell.values()) {
            for (Pose2d start : startGrid()) {
                Sim sim = new Sim(start);
                Pose2d goal = SnapGoals.bestShot(start, cell, false);

                driveTo(sim, goal);

                assertEquals(goal.x, sim.x, Constants.POSITION_TOLERANCE_IN * 2);
                assertEquals(goal.y, sim.y, Constants.POSITION_TOLERANCE_IN * 2);
                double aimError = Pose2d.wrap(sim.heading - goal.heading);
                assertEquals(0, aimError, Math.toRadians(Constants.HEADING_TOLERANCE_DEG * 2));
            }
        }
    }

    @Test
    public void shotSpotAimsTheLauncherAtTheCell() {
        for (Cell cell : Cell.values()) {
            Pose2d goal = SnapGoals.bestShot(new Pose2d(0, 60, 0), cell, false);
            double toCell = goal.bearingTo(cell.position());
            double launcher = Field.launcherAimedAlong(goal.heading);
            assertEquals(0, Pose2d.wrap(launcher - toCell), 1e-9);
            assertEquals(Constants.SHOT_DISTANCE_IN, SnapGoals.shotDistance(goal, cell), 1e-9);
            assertEquals("must be in front of the opening", cell.outward, (int) Math.signum(goal.x - cell.x()));
        }
    }

    @Test
    public void robotAlreadyInRangeJustTurns() {
        Pose2d robot = new Pose2d(16.45 + Constants.SHOT_DISTANCE_IN - 2, -12.75 + 5, 1.0);
        Pose2d goal = SnapGoals.bestShot(robot, Cell.RED_AUDIENCE, false);
        assertEquals(robot.x, goal.x, 1e-9);
        assertEquals(robot.y, goal.y, 1e-9);
    }

    @Test
    public void autoNeverPicksASpotOnTheOpponentsHalf() {
        for (Pose2d start : startGrid()) {
            assertTrue(SnapGoals.bestShot(start, Cell.RED_AUDIENCE, true).y < 0);
            assertTrue(SnapGoals.bestShot(start, Cell.BLUE_SCORING, true).y > 0);
        }
    }

    @Test
    public void aTipSendsTheRobotToTheOtherSide() {
        Sim sim = new Sim(new Pose2d(54, -12.75, 0));
        Pose2d goal = SnapGoals.bestShot(sim.pose(), Cell.RED_SCORING, false);
        double seconds = driveTo(sim, goal);
        assertTrue("crossing took " + seconds + " s", seconds < 6);
        assertTrue(sim.x < 0);
    }

    @Test
    public void routesAroundTheFrameWhenTheHiveLaneIsNotDirect() {
        Pose2d start = new Pose2d(40, -60, 0);
        Pose2d goal = new Pose2d(-40, 5, 0);
        List<Pose2d> route = RoutePlanner.route(start, goal);
        assertTrue("expected a detour", route.size() > 1);
        Pose2d from = start;
        for (Pose2d point : route) {
            assertTrue(RoutePlanner.isClear(from, point, Field.obstacles(), Constants.ROUTE_CLEARANCE_IN));
            from = point;
        }
    }

    @Test
    public void straightShotNeedsNoDetour() {
        Pose2d goal = new Pose2d(-50, -12.75, 0);
        assertEquals(Collections.singletonList(goal), RoutePlanner.route(new Pose2d(50, -12.75, 0), goal));
    }

    @Test
    public void reachesEveryFlowerWithThePokerFacingTheWall() {
        for (Pose2d flower : Field.flowers()) {
            Pose2d start = new Pose2d(flower.x * 0.6, flower.y * 0.6, 0);
            Pose2d goal = SnapGoals.nearestFlower(new Pose2d(flower.x * 0.8, flower.y * 0.8, 0));
            assertTrue("too far from Flower: " + goal, goal.distanceTo(flower) < 20);

            double pokerDirection = goal.heading + (Constants.POKER_ON_LEFT ? Math.PI / 2 : -Math.PI / 2);
            double pokerX = goal.x + Constants.FLOWER_STANDOFF_IN * Math.cos(pokerDirection)
                    + Constants.POKER_FORWARD_IN * Math.cos(goal.heading);
            double pokerY = goal.y + Constants.FLOWER_STANDOFF_IN * Math.sin(pokerDirection)
                    + Constants.POKER_FORWARD_IN * Math.sin(goal.heading);
            assertEquals(flower.x, pokerX, 1e-6);
            assertEquals(flower.y, pokerY, 1e-6);

            driveTo(new Sim(start), goal);
        }
    }

    @Test
    public void parksFromAnywhere() {
        for (Alliance alliance : Alliance.values()) {
            for (Pose2d start : startGrid()) {
                driveTo(new Sim(start), SnapGoals.park(start, alliance));
            }
        }
    }

    /** The whole loop: bad encoders, a bad starting guess, noisy late camera frames. */
    @Test
    public void cameraPullsADriftingRobotOntoTheShotSpot() {
        Random noise = new Random(1234);
        Pose2d truthStart = new Pose2d(58, -30, Math.PI);
        Sim sim = new Sim(truthStart);
        PoseEstimator estimator = new PoseEstimator();
        HiveTracker hive = new HiveTracker();
        DriveController controller = new DriveController();

        estimator.update(0, 0, 0);
        estimator.setPose(new Pose2d(50, -18, truthStart.heading), false);

        // The robot faces away from the Hive to begin with, so turn until the camera finds it.
        double encoderDrift = 1.03;
        double time = 0;
        boolean started = false;
        Pose2d goal = null;
        double lateSeconds = 0.08;
        List<Pose2d> truthLog = new ArrayList<>();

        while (time < 15) {
            time += DT;
            estimator.update(time, sim.distance * encoderDrift, sim.heading - truthStart.heading);
            truthLog.add(sim.pose());

            int framesBack = (int) Math.round(lateSeconds / DT);
            if (((int) (time / DT)) % 3 == 0 && truthLog.size() > framesBack) {
                Pose2d seen = truthLog.get(truthLog.size() - 1 - framesBack);
                Sighting clean = TagLocalizerTest.sightingFrom(seen, Cell.RED_AUDIENCE, time - lateSeconds);
                Sighting noisy = new Sighting(clean.cell,
                        clean.cameraX + noise.nextGaussian() * 0.4,
                        clean.cameraY + noise.nextGaussian() * 0.4,
                        clean.cameraZ + noise.nextGaussian() * 0.4,
                        0, 100, clean.frameTime);
                TagLocalizer.apply(Collections.singletonList(noisy), estimator, hive);
            }

            if (!started && estimator.isTrusted()) {
                started = true;
                goal = SnapGoals.bestShot(estimator.pose(), Cell.RED_AUDIENCE, false);
                controller.follow(estimator.pose(), RoutePlanner.route(estimator.pose(), goal));
            }
            if (started) {
                sim.step(controller.update(estimator.pose(), DT));
                if (controller.atGoal() && Math.abs(sim.speed) < 1 && time > 4) {
                    break;
                }
            }
        }

        assertTrue("never arrived", started && controller.atGoal());
        double range = sim.pose().distanceTo(Cell.RED_AUDIENCE.position());
        assertEquals("true shot distance", Constants.SHOT_DISTANCE_IN, range, Constants.SHOT_BAND_IN);
    }
}
