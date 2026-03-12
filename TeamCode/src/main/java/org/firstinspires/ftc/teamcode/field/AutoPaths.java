package org.firstinspires.ftc.teamcode.field;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class AutoPaths {
    private final Follower follower;

    public PathChain GoalStartToShoot;
    public PathChain GoalShootToHome;

    public PathChain goalToIntake3, goalToIntake2, goalToIntake1;
    public PathChain intake3ToShoot, intake2ToShoot, intake1ToShoot;

    public PathChain shootTogate, gateToShoot;

    public AutoPaths(Follower follower, AutoPoses poses) {
        this.follower = follower;

        // preload paths
        GoalStartToShoot = follower.pathBuilder()
                .addPath(new BezierLine(poses.goalStart, poses.centerShoot))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        // go home paths
        GoalShootToHome = createPath(poses.centerShoot, poses.goalHome);

        // intake paths
        goalToIntake3 = createIntakePath(poses.centerShoot, poses.intake3Prep, poses.intake3Take);
        goalToIntake2 = createIntakePath(poses.centerShoot, poses.intake2Prep, poses.intake2Take);
        goalToIntake1 = createIntakePath(poses.centerShoot,poses.intake1Prep, poses.intake1Take);

        // goal shoot paths
        intake3ToShoot = createPath(poses.intake3Take, poses.centerShoot);
        intake2ToShoot = createPath(poses.intake2Take, poses.gateCorner, poses.centerShoot);
        intake1ToShoot = createPath(poses.intake1Take, poses.gateCorner, poses.centerShoot);

        //gate to shoot and vv paths
        shootTogate = createIntakePath(poses.centerShoot, poses.gateIntakePrep , poses.gateIntakeTake);
        gateToShoot = follower.pathBuilder()
                .addPath(new BezierCurve(poses.gateIntakeTake, poses.gateLeaveTurn, poses.centerShoot))
                .setLinearHeadingInterpolation(poses.gateIntakeTake.getHeading(), poses.centerShoot.getHeading(), 0.8)
                .build();
    }

    private PathChain createPath(Pose first, Pose second) {
        return follower.pathBuilder()
                .addPath(new BezierLine(first, second))
                .setLinearHeadingInterpolation(first.getHeading(), second.getHeading())
                .build();
    }
    private PathChain createPath(Pose first, Pose second, Pose third) {
        return follower.pathBuilder()
                .addPath(new BezierCurve(first, second, third))
                .setLinearHeadingInterpolation(first.getHeading(), third.getHeading())
                .build();
    }
    private PathChain createIntakePath(Pose first, Pose second, Pose third) {
        return follower.pathBuilder()
                .addPath(new BezierLine(first, second))
                .setLinearHeadingInterpolation(first.getHeading(), second.getHeading())
                .addPath(new BezierLine(second, third))
                .setLinearHeadingInterpolation(second.getHeading(), third.getHeading())
                .build();
    }
}
