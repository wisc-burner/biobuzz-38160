package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.core.ShotDetector;
import org.firstinspires.ftc.teamcode.core.SnapGoals;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.ArrayDeque;
import java.util.Date;
import java.util.Deque;
import java.util.Locale;

/**
 * Records every shot with the distance the robot believed it was at, for filling in SHOT_TABLE.
 *
 * Each shot goes to telemetry, the robot log (tag "Shot") and FIRST/shots.csv on the hub. Pull the
 * file with: adb pull /sdcard/FIRST/shots.csv
 *
 * A shot's id is its run (when the OpMode started) plus its number, which matches the #n on the
 * driver hub. The file keeps growing across runs; one with an older layout is renamed out of the way.
 */
public final class ShotLog {
    private static final int SHOTS_SHOWN = 4;
    private static final File FILE = new File(AppUtil.FIRST_FOLDER, "shots.csv");
    private static final String HEADER = "run,shot,seconds,cell,distance_in,trusted,flywheel,target";

    private final ShotDetector detector = new ShotDetector();
    private final Deque<String> recent = new ArrayDeque<>();
    private final String run = new SimpleDateFormat("MM-dd HH:mm:ss", Locale.US).format(new Date());
    private int count;

    /** Call once per loop, after {@link Robot#update}. */
    public void update(Robot robot, double matchSeconds) {
        if (!detector.update(robot.shooter.target(), robot.shooter.velocity())) {
            return;
        }
        count++;
        double distance = SnapGoals.shotDistance(robot.pose(), robot.targetCell());
        boolean trusted = robot.estimator.isTrusted();
        String line = String.format("#%d  %.1f in%s  %.0f / %.0f",
                count, distance, trusted ? "" : " (NOT TRUSTED)",
                detector.launchVelocity(), robot.shooter.target());
        recent.addFirst(line);
        if (recent.size() > SHOTS_SHOWN) {
            recent.removeLast();
        }
        RobotLog.ii("Shot", line);
        append(String.format("%s,%d,%.2f,%s,%.1f,%b,%.0f,%.0f",
                run, count, matchSeconds, robot.targetCell(), distance, trusted,
                detector.launchVelocity(), robot.shooter.target()));
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Shots", "%d  (distance, flywheel / target)", count);
        for (String line : recent) {
            telemetry.addLine("  " + line);
        }
    }

    private static void append(String row) {
        if (FILE.exists() && !HEADER.equals(firstLine())) {
            File old = new File(FILE.getParent(), "shots-old-" + System.currentTimeMillis() + ".csv");
            if (!FILE.renameTo(old)) {
                RobotLog.ee("Shot", "could not move old %s aside", FILE);
            }
        }
        boolean isNew = !FILE.exists();
        try (FileWriter writer = new FileWriter(FILE, true)) {
            if (isNew) {
                writer.write(HEADER + "\n");
            }
            writer.write(row + "\n");
        } catch (IOException e) {
            RobotLog.ee("Shot", e, "could not write %s", FILE);
        }
    }

    private static String firstLine() {
        try (BufferedReader reader = new BufferedReader(new FileReader(FILE))) {
            return reader.readLine();
        } catch (IOException e) {
            return null;
        }
    }
}
