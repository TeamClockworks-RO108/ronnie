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
    private int idx = 0;

    @Override
    public PathChain getFirstRow(Follower follower) {
        return follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                TeamColor.BLUE.poses.getAutoStart(Strategy.FAR),
                                new Pose(50, 42),
                                new Pose(12.5, 34.216)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(
                        new BezierLine(
                                new Pose(12.5, 34.216),
                                TeamColor.BLUE.poses.shootFar()
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
    }

    @Override
    public PathChain getDump(Follower follower, DistanceSensor sensor) {
//        return follower
//                .pathBuilder()
//                .addPath(
//                        new BezierCurve(
//                                TeamColor.BLUE.poses.shootFar(),
//                                new Pose(12.133712660028449, 0),
//                                new Pose(12, 33)
//                        )
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(110))
//                .addPath(new BezierLine(
//                        new Pose(12, 33),
//                        TeamColor.BLUE.poses.shootFar()
//                ))
//                .setTangentHeadingInterpolation()
//                .setReversed()
//                .build();

            return follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(
                                    TeamColor.BLUE.poses.shootFar(),
                                    new Pose(12, nextY[idx % nextY.length])
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .addPath(
                            new BezierLine(
                                    new Pose(12, nextY[idx++ % nextY.length]),
                                    TeamColor.BLUE.poses.shootFar()
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
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
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
    }

    @Override
    public PathChain getHuman(Follower follower) {
        return follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                TeamColor.BLUE.poses.shootFar(),
                                new Pose(8.262, 12.481)
                        )
                )
//                .addParametricCallback(0.6, () -> {
//                    follower.setMaxPower(0.5);
//                })
//                .addParametricCallback(0.9, () -> {
//                    follower.setMaxPower(1);
//                })
                .setConstantHeadingInterpolation(Math.toRadians(-170))
                .addPath(
                        new BezierLine(
                                new Pose(8.262, 12.481),
                                TeamColor.BLUE.poses.shootFar()
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
    }

    @Override
    public PathChain getPark(Follower follower) {
        return follower.pathBuilder()
                .addPath(new BezierLine(
                        TeamColor.BLUE.poses.shootFar(),
                        new Pose(30, 9)
                ))
                .build();
    }
}
