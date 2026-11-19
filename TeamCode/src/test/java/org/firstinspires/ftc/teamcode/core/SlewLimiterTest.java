package org.firstinspires.ftc.teamcode.core;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class SlewLimiterTest {
    @Test
    public void reversingTakesTwiceAsLongAsStarting() {
        SlewLimiter limiter = new SlewLimiter(2.5);
        int loops = 0;
        while (limiter.calculate(1, 0.02) < 1) {
            loops++;
        }
        assertEquals(0.4, (loops + 1) * 0.02, 1e-9);
        loops = 0;
        while (limiter.calculate(-1, 0.02) > -1) {
            loops++;
        }
        assertEquals(0.8, (loops + 1) * 0.02, 1e-9);
    }

    @Test
    public void aStalledLoopDoesNotJump() {
        SlewLimiter limiter = new SlewLimiter(2.5);
        assertEquals(0.25, limiter.calculate(1, 5), 1e-9);
    }

    @Test
    public void carriesOnFromAReset() {
        SlewLimiter limiter = new SlewLimiter(2.5);
        limiter.reset(0.8);
        assertEquals(0.55, limiter.calculate(0, 0.1), 1e-9);
    }
}
