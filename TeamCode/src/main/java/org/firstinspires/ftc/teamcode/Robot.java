package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.core.Alliance;
import org.firstinspires.ftc.teamcode.core.Cell;
import org.firstinspires.ftc.teamcode.core.HiveTracker;
import org.firstinspires.ftc.teamcode.core.Pose2d;
import org.firstinspires.ftc.teamcode.core.PoseEstimator;
import org.firstinspires.ftc.teamcode.core.ShotTable;
import org.firstinspires.ftc.teamcode.core.Sighting;
import org.firstinspires.ftc.teamcode.core.SnapGoals;
import org.firstinspires.ftc.teamcode.core.TagLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Poker;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

import java.util.Collections;
import java.util.List;

/** Owns all the hardware and the robot's sense of where it is. Call {@link #update} once per loop. */
public final class Robot {
    public final Drive drive;
    public final Shooter shooter;
    public final Intake intake;
    public final Poker poker;
    public final Vision vision;
    public final PoseEstimator estimator = new PoseEstimator();
    public final HiveTracker hive = new HiveTracker();
    public final Snap snap;

    public Alliance alliance;
    /** The most recent batch of cluster sightings, for telemetry. */
    public List<Sighting> sightings = Collections.emptyList();

    private final List<LynxModule> hubs;

    public Robot(HardwareMap hardwareMap, Alliance alliance) {
        this.alliance = alliance;
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        drive = new Drive(hardwareMap);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        poker = new Poker(hardwareMap);
        vision = new Vision(hardwareMap);
        snap = new Snap(this);
        update(); // Reads the IMU once so setPose has something to measure from.
    }

    public void update() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }
        estimator.update(System.nanoTime() / 1e9, drive.forwardInches(), drive.yawRadians());
        shooter.update();
        List<Sighting> fresh = vision.poll();
        if (!fresh.isEmpty()) {
            sightings = fresh;
            TagLocalizer.apply(fresh, estimator, hive);
        }
    }

    public Pose2d pose() {
        return estimator.pose();
    }

    /** The Cell we should be shooting at right now. */
    public Cell targetCell() {
        return hive.raised(alliance);
    }

    public double shotVelocity() {
        if (!estimator.isTrusted()) {
            return Constants.DEFAULT_FLYWHEEL_VELOCITY;
        }
        return ShotTable.velocityFor(SnapGoals.shotDistance(pose(), targetCell()));
    }

    public void saveMatchState() {
        MatchState.save(alliance, pose(), targetCell());
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Alliance", alliance);
        telemetry.addData("Target Cell", targetCell());
        telemetry.addData("Pose", "%s %s", pose(), estimator.isTrusted() ? "" : "(NOT TRUSTED)");
        telemetry.addData("Last tag fix", "%.1f s ago", estimator.secondsSinceFix());
        telemetry.addData("Flywheel", "%.0f / %.0f", shooter.velocity(), shooter.target());
        telemetry.addData("Snap", snap.status());
        telemetry.addData("Camera", vision.status());
    }
}
