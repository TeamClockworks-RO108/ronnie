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

    public PathChain goPrepareCollectFromHuman, goPrepareCollectFromHuman2,  returnFromHuman, returnFromHuman2, goTo3rdSpike, goTo2ndSpike, goCollectFromHumanEvo, goCollectFromHumanEvo2;

    public PathChain turnToOpenGate;

    // for gate passes
    public PathChain shootTogate, gateToShoot, gateToLeave;

    public PathChain leaveFar;

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
        goalToIntake1 = createIntakePath(poses.centerShoot, poses.intake1Prep, poses.intake1Take);

        turnToOpenGate = createPath(poses.intake2Take, poses.turnToOpenGate, poses.turnToOpenGate0);
        // goal shoot paths
        intake3ToShoot = createPath(poses.intake3Take, poses.centerShoot45);
        intake2ToShoot = createPath(poses.turnToOpenGate0, poses.centerShoot);
        intake1ToShoot = createReverseTangentPath(poses.intake1Take, poses.intake1Prep, poses.centerShootFromThird);

        goTo3rdSpike = createTangentPath(poses.centerShoot, poses.intake1Prep, poses.intake1Take);
        goTo2ndSpike = createTangentPath(poses.centerShoot, poses.intake2Prep, poses.intake2Take);

        // gate intake paths
        shootTogate = createIntakePath(poses.gateShoot, poses.gateIntakePrep, poses.gateIntakeTake);
        gateToShoot = createCurvedPath(poses.gateIntakeTake, poses.gateToShootIntermediary, poses.gateShoot);
        gateToLeave = createCurvedPath(poses.gateIntakeTake, poses.gateToShootIntermediary, poses.leaveShoot);

        leaveFar = createPath(poses.farShoot, poses.farLeave);

        goCollectFromHumanEvo = createTangentCurve(poses.centerShootFromThird, poses.intCollectFromHuman, poses.finalCollectFromHuman);
        goCollectFromHumanEvo2 = createTangentCurve(poses.centerShootFromHuman, poses.intCollectFromHuman2, poses.finalCollectFromHuman2);

        goPrepareCollectFromHuman = follower.pathBuilder()
                .addPath(new BezierLine(poses.centerShoot, poses.goCollectFromHuman))
                .setTangentHeadingInterpolation()
                .build();

        returnFromHuman = follower.pathBuilder()
                .addPath(new BezierLine( poses.goCollectFromHuman, poses.centerShootFromHuman))
                .setConstantHeadingInterpolation(poses.prepareToCollectFromHuman.getHeading())
                .build();

        returnFromHuman2 = follower.pathBuilder()
                .addPath(new BezierLine( poses.goCollectFromHuman, poses.centerShootFromHumanPlusLeave))
                .setConstantHeadingInterpolation(poses.centerShootFromHumanPlusLeave.getHeading())
                .build();
    }
    private PathChain createPath(Pose first, Pose second) {
        return follower.pathBuilder()
                .addPath(new BezierLine(first, second))
                .setLinearHeadingInterpolation(first.getHeading(), second.getHeading())
                .build();
    }

    private PathChain createTangentCurve(Pose first, Pose second, Pose third) {
        return follower.pathBuilder()
                .addPath(new BezierCurve(first, second, third))
                .setTangentHeadingInterpolation()
                .build();
    }
    private PathChain createTangentPath(Pose first,Pose second,  Pose third) {
        return follower.pathBuilder()
                .addPath(new BezierCurve(first, second, third))
                .setTangentHeadingInterpolation()
                .build();
    }

    private PathChain createReverseTangentPath(Pose first, Pose third) {
        return follower.pathBuilder()
                .addPath(new BezierLine(first, third))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();
    }

    private PathChain createReverseTangentPath(Pose first, Pose second , Pose third) {
        return follower.pathBuilder()
                .addPath(new BezierCurve(first, second ,third))
                .setTangentHeadingInterpolation()
                .setReversed()
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

    private PathChain createCurvedPath(Pose first, Pose second, Pose third) {
        return follower.pathBuilder()
                .addPath(new BezierCurve(first, second, third))
                .setLinearHeadingInterpolation(first.getHeading(), third.getHeading())
                .build();
    }



}
