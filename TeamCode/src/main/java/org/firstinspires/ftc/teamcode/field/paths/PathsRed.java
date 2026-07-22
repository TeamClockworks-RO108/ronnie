package org.firstinspires.ftc.teamcode.field.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.robot.DistanceSensor;

public class PathsRed implements Paths {
    @Override
    public PathChain getFirstRow(Follower follower) {
        return null;
    }

    @Override
    public PathChain getDump(Follower follower, DistanceSensor sensor) {
        return null;
    }

    @Override
    public PathChain getReturn(Follower follower) {
        return null;
    }

    @Override
    public PathChain getPark(Follower follower) {
        return null;
    }

    @Override
    public PathChain getHuman(Follower follower) {
        return null;
    }
}
