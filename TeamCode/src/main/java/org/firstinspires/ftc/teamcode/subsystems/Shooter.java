package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;

/** Flywheel and feeder servo. Pollen is only fed once the flywheel is up to speed. */
public final class Shooter {
    private final DcMotorEx flywheel;
    private final CRServo feeder;

    private double target;
    private double velocity;
    private boolean feedAllowed;
    private final ElapsedTime feedTimer = new ElapsedTime();

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
     * True when it is safe to feed. Feeding a Pollen dips the flywheel, so feeding starts inside the
     * tolerance but only stops after a much larger dip, and never sooner than MIN_FEED_SECONDS.
     * Otherwise the feeder would stutter on every shot or stop with a Pollen half way through.
     */
    public boolean readyToFeed(double tolerance) {
        if (target <= 0) {
            feedAllowed = false;
        } else if (Math.abs(velocity - target) <= tolerance) {
            if (!feedAllowed) {
                feedTimer.reset();
            }
            feedAllowed = true;
        } else if (velocity < target - 3 * tolerance
                && feedTimer.seconds() >= Constants.MIN_FEED_SECONDS) {
            feedAllowed = false;
        }
        return feedAllowed;
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
