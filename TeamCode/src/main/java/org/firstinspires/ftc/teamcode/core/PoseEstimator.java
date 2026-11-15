package org.firstinspires.ftc.teamcode.core;

import static org.firstinspires.ftc.teamcode.Constants.FIX_GATE_IN;
import static org.firstinspires.ftc.teamcode.Constants.FIX_REJECTS_TO_ACCEPT;

/**
 * Tracks where the robot is.
 *
 * Heading comes straight from the IMU. Position comes from the drive encoders, which drift, and
 * camera fixes pull it back. A camera frame is already old when it arrives, so a fix is compared
 * with where the robot was when the frame was taken, not with where it is now.
 */
public final class PoseEstimator {
    private static final int HISTORY_SIZE = 64;

    private final double[] historyTime = new double[HISTORY_SIZE];
    private final double[] historyX = new double[HISTORY_SIZE];
    private final double[] historyY = new double[HISTORY_SIZE];
    private int historyCount;
    private int historyNext;

    private double x;
    private double y;
    private double heading;
    private double headingOffset;
    private double turnRate;

    private double lastTime;
    private double lastForward;
    private double lastRawYaw;
    private boolean hasReading;

    private boolean trusted;
    private double lastFixTime = Double.NEGATIVE_INFINITY;
    private int rejectsInARow;

    /**
     * Sets the pose. Call {@link #update} at least once first so the IMU offset can be computed.
     *
     * @param trusted false if this is only a guess, which lets the next camera fix replace it
     */
    public void setPose(Pose2d pose, boolean trusted) {
        x = pose.x;
        y = pose.y;
        setHeading(pose.heading);
        this.trusted = trusted;
        rejectsInARow = 0;
        historyCount = 0;
        historyNext = 0;
    }

    public void setHeading(double fieldHeading) {
        heading = Pose2d.wrap(fieldHeading);
        headingOffset = heading - lastRawYaw;
    }

    /**
     * @param forwardInches total distance driven according to the encoders
     * @param rawYaw        IMU yaw in radians, counter-clockwise positive
     */
    public void update(double now, double forwardInches, double rawYaw) {
        double newHeading = Pose2d.wrap(rawYaw + headingOffset);
        if (hasReading) {
            double headingChange = Pose2d.wrap(newHeading - heading);
            double distance = forwardInches - lastForward;
            double middleHeading = heading + headingChange / 2;
            x += distance * Math.cos(middleHeading);
            y += distance * Math.sin(middleHeading);
            double dt = now - lastTime;
            turnRate = dt > 0 ? headingChange / dt : 0;
        }
        heading = newHeading;
        lastTime = now;
        lastForward = forwardInches;
        lastRawYaw = rawYaw;
        hasReading = true;

        historyTime[historyNext] = now;
        historyX[historyNext] = x;
        historyY[historyNext] = y;
        historyNext = (historyNext + 1) % HISTORY_SIZE;
        historyCount = Math.min(historyCount + 1, HISTORY_SIZE);
    }

    /**
     * Applies a camera position fix.
     *
     * @param frameTime when the frame was captured, on the same clock as {@link #update}
     * @param gain      fraction of the correction to apply, 0 to 1
     * @return true if the fix was used
     */
    public boolean addFix(double frameTime, double fixX, double fixY, double gain) {
        double[] then = positionAt(frameTime);
        double errorX = fixX - then[0];
        double errorY = fixY - then[1];

        if (!trusted) {
            gain = 1;
        } else if (Math.hypot(errorX, errorY) > FIX_GATE_IN) {
            rejectsInARow++;
            if (rejectsInARow < FIX_REJECTS_TO_ACCEPT) {
                return false;
            }
            gain = 1; // The camera keeps insisting: the robot probably got shoved.
        }

        double shiftX = gain * errorX;
        double shiftY = gain * errorY;
        x += shiftX;
        y += shiftY;
        for (int i = 0; i < historyCount; i++) {
            historyX[i] += shiftX;
            historyY[i] += shiftY;
        }
        trusted = true;
        rejectsInARow = 0;
        lastFixTime = frameTime;
        return true;
    }

    public Pose2d pose() {
        return new Pose2d(x, y, heading);
    }

    /** False until the pose comes from a known start or a camera fix. Do not self-drive on a guess. */
    public boolean isTrusted() {
        return trusted;
    }

    public double turnRate() {
        return turnRate;
    }

    public double secondsSinceFix() {
        return lastTime - lastFixTime;
    }

    /** Where the robot was at {@code time}, interpolating between the samples either side of it. */
    private double[] positionAt(double time) {
        int newer = -1;
        for (int age = 1; age <= historyCount; age++) {
            int index = (historyNext - age + HISTORY_SIZE) % HISTORY_SIZE;
            if (historyTime[index] <= time) {
                if (newer < 0) {
                    return new double[] {historyX[index], historyY[index]};
                }
                double span = historyTime[newer] - historyTime[index];
                double fraction = span > 0 ? (time - historyTime[index]) / span : 0;
                return new double[] {
                        historyX[index] + fraction * (historyX[newer] - historyX[index]),
                        historyY[index] + fraction * (historyY[newer] - historyY[index]),
                };
            }
            newer = index;
        }
        // Older than everything we kept, or no history at all.
        return newer < 0 ? new double[] {x, y} : new double[] {historyX[newer], historyY[newer]};
    }
}
