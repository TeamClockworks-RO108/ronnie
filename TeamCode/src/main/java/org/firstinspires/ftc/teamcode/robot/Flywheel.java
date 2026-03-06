package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Flywheel {
    private final DcMotor leftMotor;
    private final DcMotor rightMotor;
    private boolean isOn = false;

    public Flywheel(HardwareMap hardwareMap) {
        leftMotor = hardwareMap.get(DcMotor.class, "leftFlywheel");
        rightMotor = hardwareMap.get(DcMotor.class, "rightFlywheel");

        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void toggle() {
        isOn = !isOn;
        if (isOn)
            start();
        else
            stop();
    }

    private void start() {
        leftMotor.setPower(1);
        rightMotor.setPower(1);
    }
    private void stop() {
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }
}
