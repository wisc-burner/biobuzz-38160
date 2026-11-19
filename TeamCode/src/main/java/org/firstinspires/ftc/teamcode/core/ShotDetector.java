package org.firstinspires.ftc.teamcode.core;

import org.firstinspires.ftc.teamcode.Constants;

/**
 * Spots each Pollen leaving by the dip it puts in the flywheel's speed.
 *
 * The dip is measured from the last steady speed rather than the target, so the target creeping up
 * as the robot drives back does not look like a shot.
 */
public final class ShotDetector {
    private boolean armed;
    private double launchVelocity;

    /** Call once per loop. True on the loop a shot is seen. */
    public boolean update(double target, double velocity) {
        if (target <= 0) {
            armed = false;
            return false;
        }
        if (Math.abs(velocity - target) <= Constants.TELEOP_VELOCITY_TOLERANCE) {
            armed = true;
            launchVelocity = velocity;
            return false;
        }
        if (armed && velocity < launchVelocity - Constants.SHOT_DIP_TICKS_PER_SEC) {
            armed = false; // Wait for the flywheel to recover before counting another.
            return true;
        }
        return false;
    }

    /** Flywheel speed just before the last shot, in ticks/s. */
    public double launchVelocity() {
        return launchVelocity;
    }
}
