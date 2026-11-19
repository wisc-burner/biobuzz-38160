package org.firstinspires.ftc.teamcode.core;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class ShotDetectorTest {
    private static final double TARGET = 1200;

    @Test
    public void spinningUpIsNotAShot() {
        ShotDetector detector = new ShotDetector();
        for (double v = 0; v < TARGET; v += 50) {
            assertFalse(detector.update(TARGET, v));
        }
    }

    @Test
    public void eachDipCountsOnce() {
        ShotDetector detector = new ShotDetector();
        int shots = 0;
        for (int pollen = 0; pollen < 3; pollen++) {
            double[] speeds = {1195, 1205, 1150, 1080, 1060, 1100, 1150, 1190, 1200};
            for (double v : speeds) {
                if (detector.update(TARGET, v)) {
                    shots++;
                    assertEquals(1205, detector.launchVelocity(), 1e-9);
                }
            }
        }
        assertEquals(3, shots);
    }

    @Test
    public void targetCreepingUpIsNotAShot() {
        ShotDetector detector = new ShotDetector();
        detector.update(TARGET, TARGET);
        // Backing away raises the target faster than the flywheel follows.
        for (double target = TARGET; target <= 1300; target += 10) {
            assertFalse(detector.update(target, TARGET + (target - TARGET) / 2));
        }
    }

    @Test
    public void stoppingTheFlywheelIsNotAShot() {
        ShotDetector detector = new ShotDetector();
        detector.update(TARGET, TARGET);
        assertFalse(detector.update(0, 900));
    }
}
