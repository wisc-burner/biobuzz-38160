package org.firstinspires.ftc.teamcode.core;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.Constants;
import org.junit.Test;

public class PoseEstimatorTest {
    private static PoseEstimator startedAt(Pose2d pose) {
        PoseEstimator estimator = new PoseEstimator();
        estimator.update(0, 0, 0);
        estimator.setPose(pose, true);
        return estimator;
    }

    @Test
    public void integratesDrivingAlongHeading() {
        PoseEstimator estimator = startedAt(new Pose2d(0, 0, Math.PI / 2));
        estimator.update(0.1, 10, 0);
        assertEquals(0, estimator.pose().x, 1e-9);
        assertEquals(10, estimator.pose().y, 1e-9);
    }

    @Test
    public void setPoseOffsetsAnImuThatDoesNotReadZero() {
        PoseEstimator estimator = new PoseEstimator();
        estimator.update(0, 100, 1.0);
        estimator.setPose(new Pose2d(5, 5, 0), true);
        estimator.update(0.1, 110, 1.0);
        assertEquals(0, estimator.pose().heading, 1e-9);
        assertEquals(15, estimator.pose().x, 1e-9);
    }

    @Test
    public void headingFollowsTheImuAcrossTheWrap() {
        PoseEstimator estimator = startedAt(new Pose2d(0, 0, Math.toRadians(170)));
        estimator.update(0.1, 0, Math.toRadians(20));
        assertEquals(Math.toRadians(-170), estimator.pose().heading, 1e-9);
    }

    /** At 40 in/s a 150 ms old frame is 6 in behind, which must not be mistaken for drift. */
    @Test
    public void lateFrameMatchingThePastChangesNothing() {
        PoseEstimator estimator = startedAt(new Pose2d(0, 0, 0));
        double speed = 40;
        double dt = 0.02;
        for (int i = 1; i <= 30; i++) {
            estimator.update(i * dt, speed * i * dt, 0);
        }
        double now = 30 * dt;
        double frameTime = now - 0.15;

        assertTrue(estimator.addFix(frameTime, speed * frameTime, 0, 1));

        assertEquals(speed * now, estimator.pose().x, 1e-6);
    }

    @Test
    public void lateFrameShowingRealDriftShiftsThePresent() {
        PoseEstimator estimator = startedAt(new Pose2d(0, 0, 0));
        for (int i = 1; i <= 30; i++) {
            estimator.update(i * 0.02, i, 0);
        }
        estimator.addFix(0.5, 25 + 4, 3, 0.5);
        assertEquals(30 + 2, estimator.pose().x, 1e-6);
        assertEquals(1.5, estimator.pose().y, 1e-6);
    }

    @Test
    public void repeatedFixesConvergeRatherThanOvershoot() {
        PoseEstimator estimator = startedAt(new Pose2d(0, 0, 0));
        for (int i = 1; i <= 100; i++) {
            estimator.update(i * 0.02, 0, 0);
            estimator.addFix(i * 0.02, 6, 0, 0.25);
        }
        assertEquals(6, estimator.pose().x, 1e-6);
    }

    @Test
    public void oneWildFixIsRejected() {
        PoseEstimator estimator = startedAt(new Pose2d(0, 0, 0));
        estimator.update(0.02, 0, 0);
        assertFalse(estimator.addFix(0.02, 60, 0, 0.25));
        assertEquals(0, estimator.pose().x, 1e-9);
    }

    @Test
    public void aRunOfFixesWinsAfterTheRobotIsShoved() {
        PoseEstimator estimator = startedAt(new Pose2d(0, 0, 0));
        estimator.update(0.02, 0, 0);
        for (int i = 0; i < Constants.FIX_REJECTS_TO_ACCEPT; i++) {
            estimator.addFix(0.02, 60, 0, 0.25);
        }
        assertEquals(60, estimator.pose().x, 1e-9);
    }

    @Test
    public void guessedPoseJumpsToTheFirstFix() {
        PoseEstimator estimator = new PoseEstimator();
        estimator.update(0, 0, 0);
        estimator.setPose(new Pose2d(0, 0, 0), false);
        assertFalse(estimator.isTrusted());

        assertTrue(estimator.addFix(0, 50, -20, 0.1));

        assertTrue(estimator.isTrusted());
        assertEquals(50, estimator.pose().x, 1e-9);
        assertEquals(-20, estimator.pose().y, 1e-9);
    }

    @Test
    public void historyStaysConsistentAfterSetPoseAndWrapAround() {
        PoseEstimator estimator = startedAt(new Pose2d(0, 0, 0));
        for (int i = 1; i <= 200; i++) {
            estimator.update(i * 0.02, i, 0);
        }
        estimator.setPose(new Pose2d(10, 0, 0), true);
        estimator.update(4.02, 200, 0);
        assertTrue(estimator.addFix(4.02, 11, 0, 1));
        assertEquals(11, estimator.pose().x, 1e-6);
    }
}
