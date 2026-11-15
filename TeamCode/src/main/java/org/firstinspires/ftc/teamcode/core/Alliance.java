package org.firstinspires.ftc.teamcode.core;

/** The BIOBUZZ field is symmetric under a 180 degree spin, so poses are written once, for red. */
public enum Alliance {
    RED,
    BLUE;

    /** Converts a pose written for red into this alliance's pose. */
    public Pose2d fromRed(Pose2d redPose) {
        return this == RED ? redPose : new Pose2d(-redPose.x, -redPose.y, redPose.heading + Math.PI);
    }
}
