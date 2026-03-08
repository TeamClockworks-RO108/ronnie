package org.firstinspires.ftc.teamcode.opmodes;

import android.media.ToneGenerator;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.Flywheel;
import org.firstinspires.ftc.teamcode.robot.Intake;
import org.firstinspires.ftc.teamcode.robot.Movement;
import org.firstinspires.ftc.teamcode.robot.PedroMovement;
import org.firstinspires.ftc.teamcode.robot.Turret;
import org.firstinspires.ftc.teamcode.util.EdgeDetector;

@TeleOp(name = "TeleOp")
public class Teleop extends OpMode {
    private PedroMovement movement;
    private Intake intake;
    private EdgeDetector toggleIntake = new EdgeDetector(false);
    private EdgeDetector launch = new EdgeDetector(false);
    private ElapsedTime timer;

    @Override
    public void init() {
        movement = new PedroMovement(hardwareMap, telemetry, new Pose(0, 0, 0));
        intake = new Intake(hardwareMap, telemetry);
        intake.setupFSM();

        toggleIntake.onPress(() -> intake.command(Intake.Command.TOGGLE_INTAKE));
        launch.onPress(() -> intake.command(Intake.Command.LAUNCH));

        timer = new ElapsedTime();
    }

    @Override
    public void start() {
        movement.getFollower().startTeleOpDrive();
    }

    @Override
    public void loop() {
        movement.update(gamepad1, gamepad2);

        intake.updateFSM();

        toggleIntake.update(gamepad1.triangle);
        launch.update(gamepad1.cross);
//
//        turret.rotate(gamepad1.left_trigger - gamepad1.right_trigger);
//        if (gamepad1.rightBumperWasPressed()) flywheel.toggle();
//        if (gamepad1.dpadUpWasPressed()) turret.toggleHood();
//
//        flywheel.update();

        telemetry.addData("latency (ms)", timer.milliseconds());
        timer.reset();
    }
}
