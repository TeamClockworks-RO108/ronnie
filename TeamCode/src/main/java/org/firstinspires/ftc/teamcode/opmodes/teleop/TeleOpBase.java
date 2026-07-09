package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.field.poses.Poses;
import org.firstinspires.ftc.teamcode.field.Strategy;
import org.firstinspires.ftc.teamcode.field.TeamColor;
import org.firstinspires.ftc.teamcode.robot.Flywheel;
import org.firstinspires.ftc.teamcode.robot.Intake;
import org.firstinspires.ftc.teamcode.robot.PedroMovement;
import org.firstinspires.ftc.teamcode.robot.Turret;
import org.firstinspires.ftc.teamcode.util.Drawing;

public abstract class TeleOpBase extends OpMode {
    protected PedroMovement movement;
    private Intake intake;
    private Turret turret;
    private Flywheel flywheel;
    private ElapsedTime timer;

    protected final Poses poses;
    private final Strategy strategy;

    public TeleOpBase(TeamColor color, Strategy strategy) {
        poses = color.poses;
        this.strategy = strategy;
    }

    @Override
    public void init() {
        Scheduler.reset();

        Drawing.init();
        telemetry = new MultipleTelemetry(super.telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());

        movement = new PedroMovement(hardwareMap, telemetry, poses.getStart(strategy));

        intake = new Intake(hardwareMap);
        turret = new Turret(hardwareMap, telemetry, movement.getFollower(), poses.goal());
        flywheel = new Flywheel(hardwareMap, telemetry, movement.getFollower(), poses.goal());

        timer = new ElapsedTime();
    }

    @Override
    public void start() {
        movement.getFollower().startTeleOpDrive();
    }

    @Override
    public void loop() {
        if (gamepad1.rightBumperWasPressed()) {
            Scheduler.schedule(intake.getGatherCommand());
        }
        if (gamepad1.crossWasPressed()) {
            Scheduler.schedule(intake.getLaunchCommand());
        }
        if (gamepad1.circleWasPressed()) {
            Scheduler.schedule(intake.getRejectCommand());
        }

        movement.update(gamepad1);

        turret.update();
        flywheel.update();

        telemetry.addData("latency (ms)", timer.milliseconds());
        telemetry.addData("start x", poses.getStart(Strategy.FAR).getX());
        telemetry.addData("start Y", poses.getStart(Strategy.FAR).getY());
        telemetry.update();
        drawRobotDashboard(movement.getFollower());

        Scheduler.execute();

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
