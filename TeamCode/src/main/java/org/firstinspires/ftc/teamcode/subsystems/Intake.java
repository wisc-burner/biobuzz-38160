package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;

/** Flap-wheel intake on a Core Hex motor. */
public final class Intake {
    private final DcMotor motor;

    public Intake(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotor.class, Constants.INTAKE);
    }

    public void in() {
        motor.setPower(1);
    }

    public void out() {
        motor.setPower(-1);
    }

    public void stop() {
        motor.setPower(0);
    }
}
