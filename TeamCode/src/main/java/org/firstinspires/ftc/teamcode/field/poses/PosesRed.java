package org.firstinspires.ftc.teamcode.field.poses;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

@Configurable
public class PosesRed implements Poses {
    public static double goalX = 135;
    public static double goalY = 135.4;

    @Override
    public Pose teleopCloseStart() {
        return null;
    }

    @Override
    public Pose teleopFarStart() {
        return new Pose(131, 9, Math.toRadians(0));
    }

    @Override
    public Pose goal() {
        return new Pose(goalX, goalY);
    }

    @Override
    public Pose autoFarStart() {
        return new Pose(96.9, 9, Math.toRadians(0));
    }

    @Override
    public Pose autoCloseStart() {
        return null;
    }

    @Override
    public Pose shootFar() {
        return new Pose(90, 20.6, Math.toRadians(0));
    }

    @Override
    public Pose getResetPose() {
        return new Pose(8, 9, Math.toRadians(180));
    }
}
