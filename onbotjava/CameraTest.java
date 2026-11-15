package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

/**
 * Webcam only, SDK defaults, no other hardware. If this shows a picture and the real OpModes do
 * not, the problem is in our code; if this is blank too, it is the camera, cable, or config.
 *
 * INIT it (do not press start), then Driver Station menu > Camera Stream. Tap the image to refresh.
 */
@TeleOp(name = "Camera Test", group = "Tuning")
public class CameraTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        VisionPortal portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();

        while (opModeInInit() || opModeIsActive()) {
            telemetry.addData("Camera state", portal.getCameraState());
            telemetry.addData("FPS", "%.1f", portal.getFps());
            telemetry.addLine("Want STREAMING and FPS above 0.");
            telemetry.update();
            sleep(100);
        }
        portal.close();
    }
}
