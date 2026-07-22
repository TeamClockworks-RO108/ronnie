package org.firstinspires.ftc.teamcode.field.poses;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.field.Strategy;

public interface Poses {
    Pose teleopCloseStart();

    Pose teleopFarStart();

    Pose autoFarStart();

    Pose autoCloseStart();

    default Pose getStart(Strategy strategy) {
        if (strategy == Strategy.FAR) return teleopFarStart();

        return teleopCloseStart();
    }

    default Pose getAutoStart(Strategy strategy) {
        if(strategy == Strategy.FAR) return autoFarStart();

        return autoCloseStart();
    }

    Pose shootFar();

    Pose goal();
}
