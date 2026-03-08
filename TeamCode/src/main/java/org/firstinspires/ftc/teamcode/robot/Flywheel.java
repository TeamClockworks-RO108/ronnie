package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Flywheel {
    private static final double SHOOTING_VELOCITY = 1300;

    private static final double IDLE_VELOCITY = 500;
    private final DcMotorEx leftMotor;
    private final DcMotorEx rightMotor;
    private final Telemetry telemetry;
    private boolean isOn = false;

    public Flywheel(HardwareMap hardwareMap, Telemetry telemetry) {
        leftMotor = hardwareMap.get(DcMotorEx.class, "leftFlywheel");
        rightMotor = hardwareMap.get(DcMotorEx.class, "rightFlywheel");

        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.telemetry = telemetry;
    }

    public void toggle() {
        isOn = !isOn;
        if (isOn)
            start();
        else
            stop();
    }

    public void update() {
        telemetry.addData("leftFlywheel TPS", leftMotor.getVelocity());
        telemetry.addData("rightFlywheel TPS", rightMotor.getVelocity());
    }

    public void start() {
        leftMotor.setVelocity(SHOOTING_VELOCITY);
        rightMotor.setVelocity(SHOOTING_VELOCITY);
    }

    public void idle(){
        leftMotor.setVelocity(IDLE_VELOCITY);
        rightMotor.setVelocity(IDLE_VELOCITY);
    }
   public void stop() {
        leftMotor.setVelocity(0);
        rightMotor.setVelocity(0);
    }
}
