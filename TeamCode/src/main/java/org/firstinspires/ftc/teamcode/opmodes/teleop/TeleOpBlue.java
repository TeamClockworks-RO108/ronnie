package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.field.TeamColor;
import org.firstinspires.ftc.teamcode.field.TeleOpPoses;
import org.firstinspires.ftc.teamcode.robot.Intake;
import org.firstinspires.ftc.teamcode.robot.PedroMovement;
import org.firstinspires.ftc.teamcode.robot.Turret;
import org.firstinspires.ftc.teamcode.robot.Vision;
import org.firstinspires.ftc.teamcode.util.Drawing;

@TeleOp(name = "TeleOp BLUE")
public class TeleOpBlue extends OpMode {
    protected TeamColor color = TeamColor.BLUE;

    protected PedroMovement movement;
    private Intake intake;
    private Turret turret;

    private ElapsedTime timer;


    private TeleOpPoses poses;

    @Override
    public void init() {
        Drawing.init();
        telemetry = new MultipleTelemetry(super.telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
        poses = new TeleOpPoses(color);

        movement = new PedroMovement(hardwareMap, telemetry, poses.teleOpStart);
        intake = new Intake(hardwareMap, telemetry, movement.getFollower(), poses.goalTarget, false);
        turret = new Turret(hardwareMap, telemetry, movement.getFollower(), poses.goalTarget, true, () -> Double.valueOf(gamepad1.right_stick_x), false);

        timer = new ElapsedTime();
    }

    @Override
    public void start() {
        movement.getFollower().startTeleOpDrive();
    }

    boolean flywheelRunning = true;

    @Override
    public void loop() {
        if (gamepad1.rightBumperWasPressed())   intake.command(Intake.Command.TOGGLE_INTAKE);
        if (gamepad1.crossWasPressed())         intake.command(Intake.Command.LAUNCH);
        if (gamepad1.circleWasPressed())        intake.command(Intake.Command.REJECT);
        if (gamepad1.squareWasPressed())        {
            flywheelRunning = !flywheelRunning;
            if (flywheelRunning) {
                intake.getFlywheel().overrideTarget(-1);
            } else {
                intake.getFlywheel().overrideTarget(200);
            }
        }

        // field centric reset
        if (gamepad1.dpadUpWasPressed()) {
            Pose current = movement.getFollower().getPose();
            movement.getFollower().setPose(new Pose(current.getX(), current.getY(), 0 ));
        }

        if(gamepad1.dpadDownWasPressed()){
            movement.getFollower().setPose(poses.gateReset);
        }
        // gate reset
        if (gamepad1.dpadRightWasPressed()){
            movement.getFollower().setX(poses.gateResetCollect.getX());
            movement.getFollower().setY(poses.gateResetCollect.getY());}

        turret.manualOverride((gamepad1.right_trigger - gamepad1.left_trigger + gamepad2.right_trigger - gamepad2.left_trigger)/8);

        movement.update(gamepad1, gamepad2);

        intake.update();
        turret.update();

        telemetry.addData("latency (ms)", timer.milliseconds());
        telemetry.update();
        drawRobotDashboard(movement.getFollower());
        timer.reset();
    }

    public static void drawRobotDashboard(Follower follower) {
        try {
            Drawing.drawRobot(follower.getPose());
            Drawing.drawRobot(new Pose(Turret.goalTargetX, Turret.goalTargetY), Drawing.targetLook);

            Drawing.sendPacket();


        } catch (Exception e) {
            throw new RuntimeException("Drawing failed " + e);
        }
    }
}
