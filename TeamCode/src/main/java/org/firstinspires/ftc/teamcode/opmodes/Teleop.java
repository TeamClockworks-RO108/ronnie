package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Intake;
import org.firstinspires.ftc.teamcode.robot.PedroMovement;
import org.firstinspires.ftc.teamcode.robot.Turret;

@TeleOp(name = "TeleOp")
public class Teleop extends OpMode {
    private PedroMovement movement;
    private Intake intake;
    private Turret turret;

    private ElapsedTime timer;

    private Telemetry telemetry;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(super.telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());

        movement = new PedroMovement(hardwareMap, telemetry, new Pose(25,  120, Math.toRadians(180)));
        intake = new Intake(hardwareMap, telemetry, movement.getFollower());
        turret = new Turret(hardwareMap, telemetry, movement.getFollower());

        timer = new ElapsedTime();
    }

    @Override
    public void start() {
        movement.getFollower().startTeleOpDrive();
    }

    @Override
    public void loop() {

        if (gamepad1.rightBumperWasPressed())   intake.command(Intake.Command.TOGGLE_INTAKE);
        if (gamepad1.crossWasPressed())         intake.command(Intake.Command.LAUNCH);

        if (gamepad1.dpadUpWasPressed())
            movement.getFollower().setPose(new Pose(25, 120, Math.PI));

        turret.manualOverride(gamepad1.right_trigger - gamepad1.left_trigger);

        movement.update(gamepad1, gamepad2);

        intake.update();
        turret.update();

        telemetry.addData("latency (ms)", timer.milliseconds());
        timer.reset();
    }
}
