package org.firstinspires.ftc.teamcode.core;

/**
 * A robot position on the field: inches, and heading in radians (counter-clockwise, 0 along +X).
 *
 * Field frame: origin at the field center, +X toward the audience, +Y toward the blue alliance.
 * The SDK has no BIOBUZZ field map, so this frame is ours (see {@link Field}).
 */
public final class Pose2d {
    public final double x;
    public final double y;
    public final double heading;

    public Pose2d(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = wrap(heading);
    }

    public double distanceTo(Pose2d other) {
        return Math.hypot(other.x - x, other.y - y);
    }

    public double bearingTo(Pose2d other) {
        return Math.atan2(other.y - y, other.x - x);
    }

    /** Wraps an angle to the range -PI to PI. */
    public static double wrap(double radians) {
        return Math.atan2(Math.sin(radians), Math.cos(radians));
    }

    @Override
    public String toString() {
        return String.format("(%.1f, %.1f, %.0f deg)", x, y, Math.toDegrees(heading));
    }
}
