package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Movement;
import org.firstinspires.ftc.teamcode.robot.PedroMovement;

@TeleOp(name = "TeleOp")
public class Teleop extends OpMode {
    private PedroMovement movement = null;

    private Pose startingPose = new Pose(0,0,0);

    @Override
    public void init() {
        movement = new PedroMovement(hardwareMap, telemetry, startingPose);
    }

    @Override
    public void start() {
        movement.startTeleop();
    }

    @Override
    public void loop() {

        movement.updateTeleOp(gamepad1, gamepad2);

    }
}
