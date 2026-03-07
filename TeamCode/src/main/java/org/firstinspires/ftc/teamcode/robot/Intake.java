package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    private final DcMotor leftIntake;
    private final DcMotor rightIntake;
    private boolean isOn = false;

    public Intake(HardwareMap hardwareMap) {
        leftIntake = hardwareMap.get(DcMotor.class, "leftIntake");
        rightIntake = hardwareMap.get(DcMotor.class, "rightIntake");

        leftIntake.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void toggle() {
        if (isOn)
            stop();
        else
            start();
    }

    private void start() {
        isOn = true;
        leftIntake.setPower(1);
        rightIntake.setPower(1);
    }
    private void stop() {
        isOn = false;
        leftIntake.setPower(0);
        rightIntake.setPower(0);
    }
}
