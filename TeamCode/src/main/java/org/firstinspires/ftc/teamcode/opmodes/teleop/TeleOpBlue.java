package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.field.TeamColor;
import org.firstinspires.ftc.teamcode.field.TeleOpPoses;
import org.firstinspires.ftc.teamcode.robot.Flywheel;
import org.firstinspires.ftc.teamcode.robot.Intake;
import org.firstinspires.ftc.teamcode.robot.PedroMovement;
import org.firstinspires.ftc.teamcode.robot.Turret;
import org.firstinspires.ftc.teamcode.util.Drawing;

@TeleOp(name = "TeleOp BLUE")
public class TeleOpBlue extends OpMode {
    protected TeamColor color = TeamColor.BLUE;
    protected PedroMovement movement;
    protected Intake intake;
    protected Turret turret;
    private Flywheel flywheel;
    private ElapsedTime timer;
    private TeleOpPoses poses;
    @Override
    public void init() {
        Drawing.init();
        telemetry = new MultipleTelemetry(super.telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
        poses = new TeleOpPoses();

        movement = new PedroMovement(hardwareMap, telemetry, poses.teleopFarStart);

        intake = new Intake(hardwareMap);
        turret = new Turret(hardwareMap, telemetry, movement.getFollower(), poses.blueGoal);
        flywheel = new Flywheel(hardwareMap, telemetry, movement.getFollower(), poses.blueGoal);

        timer = new ElapsedTime();
    }

    @Override
    public void start() {
        movement.getFollower().startTeleOpDrive();
    }

    @Override
    public void loop() {
        if (gamepad1.rightBumperWasPressed()) {
            intake.command(Intake.Command.TOGGLE_INTAKE);
        }
        if (gamepad1.crossWasPressed()) {
            intake.command(Intake.Command.LAUNCH);
        }
        if (gamepad1.circleWasPressed()) {
            intake.command(Intake.Command.REJECT);
        }

        movement.update(gamepad1, gamepad2);

        intake.update();
        turret.update();
        flywheel.update();

        telemetry.addData("latency (ms)", timer.milliseconds());
        telemetry.update();
        drawRobotDashboard(movement.getFollower());
        timer.reset();
    }

    public static void drawRobotDashboard(Follower follower) {
        try {
            Drawing.drawRobot(follower.getPose());
            Drawing.sendPacket();
        } catch (Exception e) {
            throw new RuntimeException("Drawing failed " + e);
        }
    }
}
