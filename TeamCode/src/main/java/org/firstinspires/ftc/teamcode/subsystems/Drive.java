package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants;

/** Two-motor tank drivetrain and the Control Hub's IMU. */
public final class Drive {
    // Change these to match how the Control Hub is mounted, then check yaw in Diagnostics.
    private static final RevHubOrientationOnRobot.LogoFacingDirection HUB_LOGO =
            RevHubOrientationOnRobot.LogoFacingDirection.UP;
    private static final RevHubOrientationOnRobot.UsbFacingDirection HUB_USB =
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

    private final DcMotorEx left;
    private final DcMotorEx right;
    private final IMU imu;
    private double forward;

    public Drive(HardwareMap hardwareMap) {
        // The intake end is the front, so the motor configured as leftDrive is on the robot's right.
        // Flipping the front without swapping sides would leave turns backwards for the IMU.
        left = hardwareMap.get(DcMotorEx.class, Constants.RIGHT_DRIVE);
        right = hardwareMap.get(DcMotorEx.class, Constants.LEFT_DRIVE);
        right.setDirection(DcMotor.Direction.REVERSE);
        for (DcMotorEx motor : new DcMotorEx[] {left, right}) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Encoders still count in this mode.
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        imu = hardwareMap.get(IMU.class, Constants.IMU);
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(HUB_LOGO, HUB_USB)));
    }

    /** @param turn counter-clockwise positive */
    public void arcade(double forward, double turn) {
        this.forward = forward;
        double leftPower = forward - turn;
        double rightPower = forward + turn;
        double scale = Math.max(1, Math.max(Math.abs(leftPower), Math.abs(rightPower)));
        left.setPower(leftPower / scale);
        right.setPower(rightPower / scale);
    }

    public void stop() {
        arcade(0, 0);
    }

    /** The forward power last asked for. */
    public double forwardPower() {
        return forward;
    }

    /** Average distance driven since init. Does not change when turning on the spot. */
    public double forwardInches() {
        return (left.getCurrentPosition() + right.getCurrentPosition()) / 2.0 / Constants.DRIVE_TICKS_PER_IN;
    }

    public double yawRadians() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }
}
