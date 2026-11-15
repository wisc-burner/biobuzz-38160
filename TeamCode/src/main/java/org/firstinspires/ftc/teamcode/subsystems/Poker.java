package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;

/**
 * Servo arm that combs Pollen out of the bottom of a Flower.
 *
 * Never command this during INIT: G304 and G403 require the robot to stay still, so set it after
 * START.
 */
public final class Poker {
    private final Servo servo;

    public Poker(HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, Constants.POKER);
    }

    public void up() {
        servo.setPosition(Constants.POKER_UP);
    }

    public void down() {
        servo.setPosition(Constants.POKER_DOWN);
    }
}
