package org.firstinspires.ftc.teamcode.robot;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class PedroMovement {
    private final Follower follower;
    private final Telemetry telemetry;


    private final double finePower = 0.25;
    private boolean isRobotCentric = true;
    private boolean areControlsFlipped = false;

    public PedroMovement(HardwareMap hardwareMap, Telemetry telemetry, Pose startingPose) {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);
        follower.update();


        this.telemetry = telemetry;
    }

    // teleop functions
    public void resetHeading(double heading) {
        follower.setPose(follower.getPose().setHeading(heading));
    }

    public void hold(Pose pose) {
        follower.holdPoint(new BezierPoint(pose.getX(), pose.getY()), pose.getHeading());
    }

    public void startTeleop() {
        follower.startTeleOpDrive();
    }

    private void setTeleop(double y, double x, double heading) {
        if (!areControlsFlipped)
            follower.setTeleOpDrive(-y, -x, heading, isRobotCentric);
        else
            follower.setTeleOpDrive(y, x, heading, isRobotCentric);
    }

    public Follower getFollower() {
        return follower;
    }
    public boolean isBusy() {
        return follower.isBusy();
    }

    public void followPath(PathChain path) {
        follower.followPath(path);
    }

    public void goToPose(Pose newPose) {
        follower.followPath(follower.pathBuilder()
                .addPath(new BezierCurve(follower.getPose(), newPose))
                .setLinearHeadingInterpolation(follower.getHeading(), newPose.getHeading())
                .build());
    }

    public void setPose(Pose pose) {
        follower.setPose(pose);
    }

    public void flipControls() {
        areControlsFlipped = true;
    }

    public void breakFollowing() {
        follower.breakFollowing();
        startTeleop();
    }

    public void update() {
        follower.update();
        // telemetry
        telemetry.addData("x, y", follower.getPose().getX() + " " + follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
    }
    public void updateTeleOp(Gamepad gamepad1, Gamepad gamepad2) {
        update();


        double y = -gamepad1.left_stick_y - gamepad2.left_stick_y * finePower;
        double x = -gamepad1.left_stick_x - gamepad2.left_stick_x * finePower;
        double heading = -gamepad1.right_stick_x - gamepad2.right_stick_x * finePower;
        setTeleop(y, x, heading);
    }
}
