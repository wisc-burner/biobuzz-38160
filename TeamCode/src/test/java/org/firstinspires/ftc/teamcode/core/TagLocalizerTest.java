package org.firstinspires.ftc.teamcode.core;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.Constants;
import org.junit.Test;

import java.util.Collections;

public class TagLocalizerTest {
    /** Builds the ftcPose a real camera would report, using the camera's axes in the field frame. */
    static Sighting sightingFrom(Pose2d robot, Cell cell, double time) {
        double pitch = Math.toRadians(Constants.CAMERA_PITCH_UP_DEG);
        double facing = robot.heading + Math.toRadians(Constants.CAMERA_YAW_DEG);

        double[] lens = {Math.cos(facing) * Math.cos(pitch), Math.sin(facing) * Math.cos(pitch), Math.sin(pitch)};
        double[] right = {Math.sin(facing), -Math.cos(facing), 0};
        double[] up = {-Math.cos(facing) * Math.sin(pitch), -Math.sin(facing) * Math.sin(pitch), Math.cos(pitch)};

        double cos = Math.cos(robot.heading);
        double sin = Math.sin(robot.heading);
        double cameraX = robot.x + Constants.CAMERA_FORWARD_IN * cos - Constants.CAMERA_LEFT_IN * sin;
        double cameraY = robot.y + Constants.CAMERA_FORWARD_IN * sin + Constants.CAMERA_LEFT_IN * cos;
        double[] toCell = {cell.x() - cameraX, cell.y() - cameraY, Field.OPENING_Z - Constants.CAMERA_HEIGHT_IN};

        return new Sighting(cell, dot(toCell, right), dot(toCell, lens), dot(toCell, up), 0, 100, time);
    }

    private static double dot(double[] a, double[] b) {
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    }

    @Test
    public void recoversRobotPositionFromManyPoses() {
        for (Cell cell : Cell.values()) {
            for (double x = -60; x <= 60; x += 30) {
                for (double y = -60; y <= 60; y += 30) {
                    for (double degrees = -150; degrees <= 180; degrees += 30) {
                        Pose2d truth = new Pose2d(x, y, Math.toRadians(degrees));
                        TagLocalizer.Target target = TagLocalizer.toRobotFrame(sightingFrom(truth, cell, 0));
                        Pose2d found = TagLocalizer.robotPosition(target, truth.heading, cell);
                        assertEquals(truth.x, found.x, 1e-6);
                        assertEquals(truth.y, found.y, 1e-6);
                        assertEquals(Field.OPENING_Z, target.height, 1e-6);
                    }
                }
            }
        }
    }

    @Test
    public void targetBehindLauncherSideIsBehindTheRobot() {
        // Launcher faces the rear, so a robot aimed at the Cell has it behind robot center.
        Pose2d start = Field.startPose(Alliance.RED, true);
        TagLocalizer.Target target = TagLocalizer.toRobotFrame(sightingFrom(start, Cell.RED_AUDIENCE, 0));
        assertEquals(Constants.LAUNCHER_FACES_REAR, target.forward < 0);
        assertEquals(0, target.left, 1e-6);
    }

    @Test
    public void raisedClusterCorrectsThePose() {
        Pose2d truth = new Pose2d(55, -12.75, 0);
        PoseEstimator estimator = new PoseEstimator();
        estimator.update(0, 0, 0);
        estimator.setPose(new Pose2d(40, -30, 0), false);
        estimator.update(0.02, 0, 0);

        TagLocalizer.apply(Collections.singletonList(sightingFrom(truth, Cell.RED_AUDIENCE, 0.02)),
                estimator, new HiveTracker());

        assertTrue(estimator.isTrusted());
        assertEquals(truth.x, estimator.pose().x, 1e-6);
        assertEquals(truth.y, estimator.pose().y, 1e-6);
    }

    @Test
    public void guessedHeadingTheWrongWayRoundIsFlipped() {
        // Blue teleop without auto guesses the robot faces the audience wall. Here it was set down
        // on the audience side facing the rear wall, looking at the raised blue audience Cell.
        Pose2d truth = new Pose2d(54, 12.75, Math.PI);
        PoseEstimator estimator = new PoseEstimator();
        estimator.update(0, 0, 0);
        estimator.setPose(Field.startPose(Alliance.BLUE, true), false);
        estimator.update(0.02, 0, 0);

        TagLocalizer.apply(Collections.singletonList(sightingFrom(truth, Cell.BLUE_AUDIENCE, 0.02)),
                estimator, new HiveTracker());

        assertEquals(truth.x, estimator.pose().x, 1e-6);
        assertEquals(truth.y, estimator.pose().y, 1e-6);
        assertEquals(0, Pose2d.wrap(estimator.pose().heading - truth.heading), 1e-6);
    }

    @Test
    public void wrongHeightIsIgnored() {
        Sighting good = sightingFrom(new Pose2d(55, -12.75, 0), Cell.RED_AUDIENCE, 0.02);
        Sighting bad = new Sighting(good.cell, good.cameraX, good.cameraY * 0.4, good.cameraZ * 0.4, 0, 100, 0.02);
        PoseEstimator estimator = new PoseEstimator();
        estimator.update(0, 0, 0);
        estimator.setPose(new Pose2d(0, 0, 0), false);

        TagLocalizer.apply(Collections.singletonList(bad), estimator, new HiveTracker());

        assertFalse(estimator.isTrusted());
    }

    @Test
    public void barelyVisibleClusterIsIgnored() {
        Sighting good = sightingFrom(new Pose2d(55, -12.75, 0), Cell.RED_AUDIENCE, 0.02);
        Sighting partial = new Sighting(good.cell, good.cameraX, good.cameraY, good.cameraZ, 0, 25, 0.02);
        PoseEstimator estimator = new PoseEstimator();
        estimator.update(0, 0, 0);
        estimator.setPose(new Pose2d(0, 0, 0), false);

        TagLocalizer.apply(Collections.singletonList(partial), estimator, new HiveTracker());

        assertFalse(estimator.isTrusted());
    }

    @Test
    public void hiveStateFollowsRollAfterSeveralFrames() {
        HiveTracker hive = new HiveTracker();
        Sighting upsideDown = new Sighting(Cell.RED_AUDIENCE, 0, 60, 0, 159, 100, 0);

        for (int i = 0; i < Constants.HIVE_FLIP_FRAMES - 1; i++) {
            hive.observe(upsideDown);
            assertEquals("one glitchy frame must not flip it", Cell.RED_AUDIENCE, hive.raised(Alliance.RED));
        }
        hive.observe(upsideDown);
        assertEquals(Cell.RED_SCORING, hive.raised(Alliance.RED));
        assertEquals("blue is a separate Hive", Cell.BLUE_SCORING, hive.raised(Alliance.BLUE));
    }

    @Test
    public void aRightWayUpFrameResetsTheFlipCount() {
        HiveTracker hive = new HiveTracker();
        Sighting upsideDown = new Sighting(Cell.RED_AUDIENCE, 0, 60, 0, 159, 100, 0);
        Sighting upright = new Sighting(Cell.RED_AUDIENCE, 0, 60, 0, -19, 100, 0);

        for (int i = 0; i < Constants.HIVE_FLIP_FRAMES * 3; i++) {
            hive.observe(i % 2 == 0 ? upsideDown : upright);
        }
        assertEquals(Cell.RED_AUDIENCE, hive.raised(Alliance.RED));
    }
}
