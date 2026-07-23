package org.firstinspires.ftc.teamcode.field.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.field.Strategy;
import org.firstinspires.ftc.teamcode.field.TeamColor;
import org.firstinspires.ftc.teamcode.robot.DistanceSensor;

public class PathsBlue implements Paths {
    @Override
    public PathChain getFirstRow(Follower follower) {
        return follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                TeamColor.BLUE.poses.getAutoStart(Strategy.FAR),
                                new Pose(50, 42),
                                new Pose(12, 35.799)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(
                        new BezierLine(
                                new Pose(12, 35.799),
                                TeamColor.BLUE.poses.shootFar()
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
                                TeamColor.BLUE.poses.shootFar(),
                                new Pose(9, 6),
                                new Pose(8, 30)
                        )
                )
                .setTangentHeadingInterpolation()
                .addPath(
                        new BezierLine(
                                new Pose(8, 30),
                                TeamColor.BLUE.poses.shootFar()
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .setBrakingStrength(3)
                .build();
    }

    @Override
    public PathChain getReturn(Follower follower) {
        return follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                follower.getPose(),
                                TeamColor.BLUE.poses.shootFar()
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
                                TeamColor.BLUE.poses.shootFar(),
                                new Pose(13, 9)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(
                        new BezierLine(
                                new Pose(13, 9),
                                TeamColor.BLUE.poses.shootFar()
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
