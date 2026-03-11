package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.field.AutoPaths;
import org.firstinspires.ftc.teamcode.field.AutoPoses;
import org.firstinspires.ftc.teamcode.field.TeamColor;
import org.firstinspires.ftc.teamcode.robot.Intake;
import org.firstinspires.ftc.teamcode.robot.PedroMovement;
import org.firstinspires.ftc.teamcode.robot.Turret;

public abstract class AutoBase extends OpMode {
    protected TeamColor color;

    protected PedroMovement movement;
    protected Intake intake;
    protected Turret turret;

    protected AutoPoses poses;
    protected AutoPaths paths;
    protected Pose startingPose;

    @Override
    public void init() {
        setColor();
        poses = new AutoPoses(color);
        setStartingPose();

        movement = new PedroMovement(hardwareMap, telemetry, startingPose);
        paths = new AutoPaths(movement.getFollower(), poses);

        intake = new Intake(hardwareMap, telemetry, movement.getFollower(), poses.goalTarget);
        turret = new Turret(hardwareMap, telemetry, movement.getFollower(), poses.goalTarget);

        setupFSM();
    }

    @Override
    public void start() {
        startFSM();
    }

    @Override
    public void loop() {
        updateFSM();

        movement.update();
        intake.update();
        turret.update();

        telemetry.update();
    }

    protected abstract void setColor();
    protected abstract void setStartingPose();
    protected abstract void setupFSM();
    protected abstract void startFSM();
    protected abstract void updateFSM();
}