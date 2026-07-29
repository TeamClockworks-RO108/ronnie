package org.firstinspires.ftc.teamcode.field.poses;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

@Configurable
public class PosesRed implements Poses {
    public static double goalX = 130;
    public static double goalY = 135.4;

    @Override
    public Pose teleopCloseStart() {
        return null;
    }

    @Override
    public Pose teleopFarStart() {
        return new Pose(133, 8.5528, Math.toRadians(0));
    }

    @Override
    public Pose goal() {
        return new Pose(goalX, goalY);
    }

    @Override
    public Pose autoFarStart() {
        return new Pose(97.8, 7.999, Math.toRadians(0));
    }

    @Override
    public Pose autoCloseStart() {
        return null;
    }

    @Override
    public Pose shootFar() {
        return new Pose(87.655, 21.244, Math.toRadians(0));
    }

    @Override
    public Pose getResetPose() {
        return new Pose(7.848, 10.596, Math.toRadians(180));
    }
}
