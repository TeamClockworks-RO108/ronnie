package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Movement;

@TeleOp(name = "TeleOp")
public class Teleop extends OpMode {
    private Movement movement;

    @Override
    public void init() {
        movement = new Movement(hardwareMap);
    }

    @Override
    public void loop() {
        movement.update(gamepad1);
    }
}
