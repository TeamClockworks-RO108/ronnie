package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Configurable
public class PedroMovement {
    private final Follower follower;
    private final Telemetry telemetry;
    private boolean areControlsFlipped = false;
    public PedroMovement(HardwareMap hardwareMap, Telemetry telemetry, Pose startingPose) {
        this.telemetry = telemetry;

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);
    }

    public void update() {
        follower.update();

        // telemetry
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
    }

    public void update(Gamepad gamepad1) {
        update();

        double y = gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x ;
        double heading = -gamepad1.right_stick_x;
        setTeleop(-y, -x, heading);
    }

    public void resetHeading(double heading) {
        follower.setPose(follower.getPose().setHeading(heading));
    }

    public void followPath(PathChain path) {
        follower.followPath(path);
    }
    public void goToPose(Pose newPose) {
        follower.followPath(follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(), newPose))
                .setLinearHeadingInterpolation(follower.getHeading(), newPose.getHeading())
                .build());
    }
    public boolean isBusy() {
        return follower.isBusy();
    }

    public void flipControls() {
        areControlsFlipped = true;
    }

    private void setTeleop(double y, double x, double heading) {
        if (!areControlsFlipped)
            follower.setTeleOpDrive(-y, -x, heading, false);
        else
            follower.setTeleOpDrive(y, x, heading, false);
    }

    public Follower getFollower(){
        return follower;
    }

    public Pose getTurretPose() {
        return new Pose(
                getFollower().getPose().getX(),
                getFollower().getPose().getY(),
                getFollower().getHeading()
        );
    }
}