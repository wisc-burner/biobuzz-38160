package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.core.Cell;
import org.firstinspires.ftc.teamcode.core.Sighting;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.TimeUnit;

/**
 * USB webcam reading the Hive's AprilTag clusters.
 *
 * SDK 12 reports BIOBUZZ tags as one AprilTagClusterDetection per Cell. Do not use its robotPose:
 * the library has no field position for the clusters, so it is not a field pose. See core.Field.
 */
public final class Vision {
    private final AprilTagProcessor tags;
    private final VisionPortal portal;
    private boolean exposureSet;
    private String cameraStatus = "";

    public Vision(HardwareMap hardwareMap) {
        tags = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getBioBuzzTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();
        tags.setDecimation(Constants.TAG_DECIMATION);
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, Constants.WEBCAM))
                .setCameraResolution(new Size(640, 480))
                // Uncompressed YUY2 is the C920's heaviest USB mode; starting it reset the Control Hub.
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(tags)
                .build();
    }

    /** Returns clusters from any frame that arrived since the last call. */
    public List<Sighting> poll() {
        setExposureOnce();
        List<AprilTagDetection> detections = tags.getFreshDetections();
        if (detections == null) {
            return Collections.emptyList();
        }
        List<Sighting> sightings = new ArrayList<>();
        for (AprilTagDetection detection : detections) {
            if (detection instanceof AprilTagClusterDetection) {
                AprilTagClusterDetection cluster = (AprilTagClusterDetection) detection;
                Cell cell = Cell.fromClusterName(cluster.metadata.name);
                if (cell != null) {
                    sightings.add(new Sighting(cell, cluster.ftcPose.x, cluster.ftcPose.y, cluster.ftcPose.z,
                            cluster.ftcPose.roll, cluster.percentClusterFound,
                            cluster.frameAcquisitionNanoTime / 1e9));
                }
            }
        }
        return sightings;
    }

    private void setExposureOnce() {
        if (exposureSet || portal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            return;
        }
        exposureSet = true;
        // Out-of-range values are silently rejected, which can leave the image black. Clamp to
        // what this camera reports, and leave it on auto if it has no manual exposure at all.
        ExposureControl exposure = portal.getCameraControl(ExposureControl.class);
        if (exposure == null || !exposure.isModeSupported(ExposureControl.Mode.Manual)) {
            cameraStatus = "auto exposure (no manual mode)";
            return;
        }
        exposure.setMode(ExposureControl.Mode.Manual);
        long minMs = exposure.getMinExposure(TimeUnit.MILLISECONDS);
        long maxMs = exposure.getMaxExposure(TimeUnit.MILLISECONDS);
        long ms = Math.max(minMs, Math.min(maxMs, Constants.CAMERA_EXPOSURE_MS));
        exposure.setExposure(ms, TimeUnit.MILLISECONDS);
        cameraStatus = "exposure " + ms + "ms (" + minMs + "-" + maxMs + ")";

        GainControl gain = portal.getCameraControl(GainControl.class);
        if (gain != null) {
            int g = Math.max(gain.getMinGain(), Math.min(gain.getMaxGain(), Constants.CAMERA_GAIN));
            gain.setGain(g);
            cameraStatus += ", gain " + g + " (" + gain.getMinGain() + "-" + gain.getMaxGain() + ")";
        }
    }

    /** Camera state and the exposure/gain actually applied, for telemetry. */
    public String status() {
        return portal.getCameraState() + (cameraStatus.isEmpty() ? "" : ", " + cameraStatus);
    }
}
