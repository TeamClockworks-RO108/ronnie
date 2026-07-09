package org.firstinspires.ftc.teamcode.field.poses;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.field.Strategy;

public interface Poses {
    Pose teleopCloseStart();

    Pose teleopFarStart();

    default Pose getStart(Strategy strategy) {
        if (strategy == Strategy.FAR) return teleopFarStart();

        return teleopCloseStart();
    }

    Pose goal();
}
