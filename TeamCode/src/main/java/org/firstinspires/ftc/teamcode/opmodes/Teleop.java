package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.Flywheel;
import org.firstinspires.ftc.teamcode.robot.Intake;
import org.firstinspires.ftc.teamcode.robot.Movement;
import org.firstinspires.ftc.teamcode.robot.PedroMovement;
import org.firstinspires.ftc.teamcode.robot.Turret;

@TeleOp(name = "TeleOp")
public class Teleop extends OpMode {
    private PedroMovement movement;
    private Intake intake;
    private Turret turret;
    private Flywheel flywheel;
    private ElapsedTime timer;

    @Override
    public void init() {
        movement = new PedroMovement(hardwareMap, telemetry, new Pose(0, 0, 0));
        intake = new Intake(hardwareMap);
        turret = new Turret(hardwareMap);
        flywheel = new Flywheel(hardwareMap, telemetry);

        timer = new ElapsedTime();
    }

    @Override
    public void start() {
        movement.getFollower().startTeleOpDrive();
    }

    @Override
    public void loop() {
        movement.update(gamepad1, gamepad2);

        turret.rotate(gamepad1.left_trigger - gamepad1.right_trigger);
        if (gamepad1.leftBumperWasPressed()) intake.toggle();
        if (gamepad1.rightBumperWasPressed()) flywheel.toggle();
        if (gamepad1.dpadUpWasPressed()) turret.toggleHood();

        flywheel.update();

        telemetry.addData("latency (ms)", timer.milliseconds());
        timer.reset();
    }
}
