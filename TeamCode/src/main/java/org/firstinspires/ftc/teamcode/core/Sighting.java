package org.firstinspires.ftc.teamcode.core;

/** One AprilTag cluster seen by the camera, stripped of SDK types so it can be tested on a laptop. */
public final class Sighting {
    public final Cell cell;
    /** Vector from camera to the center of the Cell opening: right, out of the lens, and up. */
    public final double cameraX;
    public final double cameraY;
    public final double cameraZ;
    public final double rollDegrees;
    public final int percentVisible;
    public final double frameTime;

    public Sighting(Cell cell, double cameraX, double cameraY, double cameraZ,
                    double rollDegrees, int percentVisible, double frameTime) {
        this.cell = cell;
        this.cameraX = cameraX;
        this.cameraY = cameraY;
        this.cameraZ = cameraZ;
        this.rollDegrees = rollDegrees;
        this.percentVisible = percentVisible;
        this.frameTime = frameTime;
    }

    /** FIRST's rule: a cluster that reads right-side up (|roll| < 90) is on the scorable Cell. */
    public boolean isRaised() {
        return Math.abs(rollDegrees) < 90;
    }
}
