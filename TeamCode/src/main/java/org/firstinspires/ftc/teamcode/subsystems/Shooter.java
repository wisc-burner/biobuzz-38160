package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;

/** Flywheel and feeder servo. Pollen is only fed once the flywheel is up to speed. */
public final class Shooter {
    private final DcMotorEx flywheel;
    private final CRServo feeder;

    private double target;
    private double velocity;
    private boolean feedAllowed;

    public Shooter(HardwareMap hardwareMap) {
        flywheel = hardwareMap.get(DcMotorEx.class, Constants.FLYWHEEL);
        feeder = hardwareMap.get(CRServo.class, Constants.FEEDER);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    /** Call once per loop. */
    public void update() {
        velocity = flywheel.getVelocity();
    }

    public void spin(double ticksPerSecond) {
        if (Math.abs(ticksPerSecond - target) > 0.5) { // Each write to the hub costs loop time.
            target = ticksPerSecond;
            flywheel.setVelocity(ticksPerSecond);
        }
    }

    public void reverse() {
        target = 0;
        flywheel.setPower(Constants.FLYWHEEL_REVERSE_POWER);
    }

    public void stopFlywheel() {
        target = 0;
        feedAllowed = false;
        flywheel.setPower(0);
    }

    public void feed(double power) {
        feeder.setPower(power);
    }

    /**
     * True when it is safe to feed. Feeding starts once the flywheel is within the tolerance, then
     * keeps going through the dip each shot causes until {@link #holdFire} or the flywheel stops.
     * Stopping on the dip would leave a Pollen pressed against the wheel, holding it below speed so
     * feeding never restarted.
     */
    public boolean readyToFeed(double tolerance) {
        if (target <= 0) {
            feedAllowed = false;
        } else if (atSpeed(tolerance)) {
            feedAllowed = true;
        }
        return feedAllowed;
    }

    /** The next feed waits for the flywheel to be up to speed again. */
    public void holdFire() {
        feedAllowed = false;
    }

    public boolean atSpeed(double tolerance) {
        return target > 0 && Math.abs(velocity - target) <= tolerance;
    }

    public double velocity() {
        return velocity;
    }

    public double target() {
        return target;
    }
}
