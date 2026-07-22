package org.firstinspires.ftc.teamcode.field.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.field.Strategy;
import org.firstinspires.ftc.teamcode.field.TeamColor;
import org.firstinspires.ftc.teamcode.field.poses.Poses;
import org.firstinspires.ftc.teamcode.robot.DistanceSensor;

public class PathsBlue implements Paths {
    private final Poses poses;

    public PathsBlue() {
        poses = TeamColor.BLUE.poses;
    }

    @Override
    public PathChain getFirstRow(Follower follower) {
        return follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                poses.getAutoStart(Strategy.FAR),
                                new Pose(50, 42),
                                new Pose(12, 35.799)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(
                        new BezierLine(
                                new Pose(12, 35.799),
                                poses.getShoot()
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
    }

    @Override
    public PathChain getDump(Follower follower, DistanceSensor sensor) {
        return follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                poses.getShoot(),
                                new Pose(7, 7),
                                new Pose(8, 40)
                        )
                )
                .setTangentHeadingInterpolation()
                .addPath(
                        new BezierLine(
                                new Pose(8, 40),
                                poses.getShoot()
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();
    }

    @Override
    public PathChain getReturn(Follower follower) {
        return follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                follower.getPose(),
                                poses.getShoot()
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();
    }

    @Override
    public PathChain getHuman(Follower follower) {
        return follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                poses.getShoot(),
                                new Pose(9, 9)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(
                        new BezierLine(
                                new Pose(9, 9),
                                poses.getShoot()
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
    }

    @Override
    public PathChain getPark(Follower follower) {
        return null;
    }
}
