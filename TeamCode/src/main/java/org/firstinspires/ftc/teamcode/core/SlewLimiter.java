package org.firstinspires.ftc.teamcode.core;

/**
 * Limits how fast a power may change, both speeding up and slowing down. A tall robot on brake
 * mode tips if full forward goes straight to full reverse.
 */
public final class SlewLimiter {
    /** A stalled loop must not turn into one big jump. */
    private static final double MAX_DT = 0.1;

    private final double maxPerSecond;
    private double value;

    public SlewLimiter(double maxPerSecond) {
        this.maxPerSecond = maxPerSecond;
    }

    /** Moves toward {@code target} by at most what {@code dt} seconds allow. */
    public double calculate(double target, double dt) {
        double step = maxPerSecond * Math.min(dt, MAX_DT);
        value += Math.max(-step, Math.min(step, target - value));
        return value;
    }

    /** Carry on from a power something else was driving at. */
    public void reset(double value) {
        this.value = value;
    }
}
