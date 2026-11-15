package org.firstinspires.ftc.teamcode.core;

import static org.firstinspires.ftc.teamcode.Constants.CAMERA_FORWARD_IN;
import static org.firstinspires.ftc.teamcode.Constants.CAMERA_HEIGHT_IN;
import static org.firstinspires.ftc.teamcode.Constants.CAMERA_LEFT_IN;
import static org.firstinspires.ftc.teamcode.Constants.CAMERA_PITCH_UP_DEG;
import static org.firstinspires.ftc.teamcode.Constants.CAMERA_YAW_DEG;
import static org.firstinspires.ftc.teamcode.Constants.FIX_GAIN;
import static org.firstinspires.ftc.teamcode.Constants.FIX_HEIGHT_TOLERANCE_IN;
import static org.firstinspires.ftc.teamcode.Constants.FIX_MAX_RANGE_IN;
import static org.firstinspires.ftc.teamcode.Constants.FIX_MAX_TURN_DEG_PER_SEC;
import static org.firstinspires.ftc.teamcode.Constants.FIX_MIN_PERCENT_VISIBLE;

import java.util.List;

/**
 * Turns cluster sightings into robot positions.
 *
 * ftcPose points from the camera to the center of the Cell opening, and we know where that is on
 * the field while the Cell is raised. Heading comes from the IMU, which is far steadier than the
 * rotation solved from a tag seen at a steep angle, so the camera only has to supply position.
 */
public final class TagLocalizer {
    private TagLocalizer() {}

    /** A Cell opening as seen from the robot's center. */
    public static final class Target {
        public final double forward;
        public final double left;
        public final double height;

        Target(double forward, double left, double height) {
            this.forward = forward;
            this.left = left;
            this.height = height;
        }
    }

    public static Target toRobotFrame(Sighting sighting) {
        double pitch = Math.toRadians(CAMERA_PITCH_UP_DEG);
        double yaw = Math.toRadians(CAMERA_YAW_DEG);

        // Undo the upward tilt.
        double level = sighting.cameraY * Math.cos(pitch) - sighting.cameraZ * Math.sin(pitch);
        double up = sighting.cameraY * Math.sin(pitch) + sighting.cameraZ * Math.cos(pitch);
        double left = -sighting.cameraX;

        // Undo the camera's yaw, then add where it is mounted on the robot.
        return new Target(
                CAMERA_FORWARD_IN + level * Math.cos(yaw) - left * Math.sin(yaw),
                CAMERA_LEFT_IN + level * Math.sin(yaw) + left * Math.cos(yaw),
                CAMERA_HEIGHT_IN + up);
    }

    /** Where the robot must be for {@code target} to be where the camera sees it. */
    public static Pose2d robotPosition(Target target, double heading, Cell cell) {
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);
        return new Pose2d(
                cell.x() - (target.forward * cos - target.left * sin),
                cell.y() - (target.forward * sin + target.left * cos),
                heading);
    }

    /** Updates the Hive state and pose estimate from one batch of sightings. */
    public static void apply(List<Sighting> sightings, PoseEstimator estimator, HiveTracker hive) {
        for (Sighting sighting : sightings) {
            if (sighting.percentVisible < FIX_MIN_PERCENT_VISIBLE) {
                continue;
            }
            hive.observe(sighting);

            Target target = toRobotFrame(sighting);
            double range = Math.hypot(target.forward, target.left);
            boolean steady = Math.abs(estimator.turnRate()) < Math.toRadians(FIX_MAX_TURN_DEG_PER_SEC);
            boolean heightMakesSense = Math.abs(target.height - Field.OPENING_Z) < FIX_HEIGHT_TOLERANCE_IN;
            if (sighting.isRaised() && steady && heightMakesSense && range < FIX_MAX_RANGE_IN) {
                Pose2d fix = robotPosition(target, estimator.pose().heading, sighting.cell);
                if (!estimator.isTrusted() && !inFrontOf(fix, sighting.cell)) {
                    // A raised Cell can only be seen from in front of its opening. The guessed
                    // heading must be the wrong way round: the robot was set down facing the other wall.
                    estimator.setHeading(estimator.pose().heading + Math.PI);
                    fix = robotPosition(target, estimator.pose().heading, sighting.cell);
                }
                estimator.addFix(sighting.frameTime, fix.x, fix.y, FIX_GAIN * sighting.percentVisible / 100.0);
            }
        }
    }

    private static boolean inFrontOf(Pose2d robot, Cell cell) {
        return cell.outward * (robot.x - cell.x()) > 0;
    }
}
