package org.firstinspires.ftc.teamcode.opmodes.teleop;

import android.util.Log;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.field.Strategy;
import org.firstinspires.ftc.teamcode.field.TeamColor;
import org.firstinspires.ftc.teamcode.field.poses.Poses;
import org.firstinspires.ftc.teamcode.robot.DistanceSensor;
import org.firstinspires.ftc.teamcode.robot.Flywheel;
import org.firstinspires.ftc.teamcode.robot.Intake;
import org.firstinspires.ftc.teamcode.robot.Lift;
import org.firstinspires.ftc.teamcode.robot.PedroMovement;
import org.firstinspires.ftc.teamcode.robot.Turret;
import org.firstinspires.ftc.teamcode.util.Drawing;
import org.firstinspires.ftc.teamcode.util.RobotContext;

public abstract class TeleOpBase extends OpMode {
    protected PedroMovement movement;
    private Intake intake;
    private Turret turret;
    private Flywheel flywheel;
    private ElapsedTime timer;

    private final TeamColor color;
    protected final Poses poses;
    private final Strategy strategy;

    private DistanceSensor distanceSensor;

  //  private Lift lift;

    public TeleOpBase(TeamColor color, Strategy strategy) {
        this.color = color;
        this.strategy = strategy;
        poses = color.poses;
    }

    @Override
    public void init() {
        Scheduler.reset();

        Drawing.init();
        telemetry = new MultipleTelemetry(super.telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());

        Pose startPose =  RobotContext.getLastPose().orElse(poses.getStart(strategy));
        movement = new PedroMovement(hardwareMap, telemetry,startPose);
        intake = new Intake(hardwareMap, telemetry);
        turret = new Turret(hardwareMap, telemetry, movement, poses.goal(),
                RobotContext.getLastPose().isEmpty(), color);
        flywheel = new Flywheel(hardwareMap, telemetry, movement.getFollower(), poses);
        distanceSensor = new DistanceSensor(hardwareMap, telemetry);


     //   lift = new Lift(hardwareMap, telemetry, false);


        timer = new ElapsedTime();

        RobotContext.setLastPose(null);
    }

    @Override
    public void start() {
        if(color == TeamColor.RED) movement.flipControls();

        movement.getFollower().startTeleOpDrive();
    }

    @Override
    public void loop() {
        if (gamepad1.rightBumperWasPressed()) {
            Scheduler.schedule(intake.getTogglePowerCommand());
        }
        if (gamepad1.circleWasPressed()) {
            Scheduler.schedule(intake.getToggleDirectionCommand());
        }

//        if(gamepad1.triangleWasPressed()){
//            if(lift.getLiftStatus()){
//                lift.drop();
//            }else lift.lift();
//
//        }


        if(gamepad1.dpadUpWasPressed()) {
            movement.getFollower().setX(poses.getResetPose().getX());
            movement.getFollower().setY(poses.getResetPose().getY());
            movement.getFollower().setHeading(poses.getResetPose().getHeading());
        }

        if(gamepad1.dpadLeftWasPressed()) {
            turret.offset(1);
        }
        if(gamepad1.dpadRightWasPressed()) {
            turret.offset(-1);
        }

        if(gamepad1.left_trigger == 0) {
            if (gamepad1.right_trigger > 0) {
                Scheduler.schedule(intake.getStartCommand());
            }
            if (gamepad1.right_trigger == 0) {
                Scheduler.schedule(intake.getStopCommand());
            }
        }

        if(gamepad1.right_trigger == 0) {
            if (gamepad1.left_trigger > 0) {
                Scheduler.schedule(intake.getManualOpenBarrierCommand());
            } else {
                Scheduler.schedule(intake.getManualCloseBarrierCommand());
            }
        }

        movement.update(gamepad1);

        turret.update();
        flywheel.update();
        distanceSensor.update();
        intake.update();

        telemetry.addData("lt", gamepad1.left_trigger);
        telemetry.addData("rt", gamepad1.right_trigger);
        telemetry.addData("latency (ms)", timer.milliseconds());
        drawRobotDashboard(movement.getFollower());

        Scheduler.execute();

        timer.reset();
    }

    public static void drawRobotDashboard(Follower follower) {
        try {
            Drawing.drawRobot(follower.getPose());
            Drawing.drawRobot(TeamColor.BLUE.poses.goal());
            Drawing.drawRobot(TeamColor.RED.poses.goal());
            Drawing.sendPacket();
        } catch (Exception e) {
            throw new RuntimeException("Drawing failed " + e);
        }
    }
}
