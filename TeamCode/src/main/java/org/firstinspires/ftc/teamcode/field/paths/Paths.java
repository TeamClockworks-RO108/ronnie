package org.firstinspires.ftc.teamcode.field.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.robot.DistanceSensor;

public interface Paths {
    PathChain getFirstRow(Follower follower);
    PathChain getDump(Follower follower, DistanceSensor sensor);
    PathChain getHuman(Follower follower);
    PathChain getPark(Follower follower);

    PathChain getReturn(Follower follower);
}
