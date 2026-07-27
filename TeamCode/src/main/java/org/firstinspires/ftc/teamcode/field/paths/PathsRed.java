package org.firstinspires.ftc.teamcode.field.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.field.Strategy;
import org.firstinspires.ftc.teamcode.field.TeamColor;
import org.firstinspires.ftc.teamcode.robot.DistanceSensor;

public class PathsRed implements Paths {
    private final double[] nextY = {9, 35};
    private int idx = 0;

    @Override
    public PathChain getFirstRow(Follower follower) {
        return follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                TeamColor.RED.poses.getAutoStart(Strategy.FAR),
                                new Pose(93, 42),
                                new Pose(130, 35.799)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        new BezierLine(
                                new Pose(130, 35.799),
                                TeamColor.RED.poses.shootFar()
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
    }

    @Override
    public PathChain getDump(Follower follower, DistanceSensor sensor) {
        return follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                TeamColor.RED.poses.shootFar(),
                                new Pose(130, nextY[idx % nextY.length])
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        new BezierLine(
                                new Pose(130, nextY[idx++ % nextY.length]),
                                TeamColor.RED.poses.shootFar()
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
    }

    @Override
    public PathChain getReturn(Follower follower) {
        return follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                follower.getPose(),
                                TeamColor.RED.poses.shootFar()
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
    }

    @Override
    public PathChain getHuman(Follower follower) {
        return follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                TeamColor.RED.poses.shootFar(),
                                new Pose(130, 9)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        new BezierLine(
                                new Pose(130, 9),
                                TeamColor.RED.poses.shootFar()
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
    }

    @Override
    public PathChain getPark(Follower follower) {
        return follower.pathBuilder()
                .addPath(new BezierLine(
                        TeamColor.RED.poses.shootFar(),
                        new Pose(110, 9)
                ))
                .build();
    }
}
