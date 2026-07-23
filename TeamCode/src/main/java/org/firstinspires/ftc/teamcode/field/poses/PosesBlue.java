package org.firstinspires.ftc.teamcode.field.poses;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

@Configurable
public class PosesBlue implements Poses{
    public static double goalX = 2.95;
    public static double goalY = 135.4;

    @Override
    public Pose teleopCloseStart() {
        return new Pose(86, 107, Math.toRadians(-55));
    }

    @Override
    public Pose teleopFarStart() {
        return new Pose(8, 9, Math.toRadians(180));
    }

    @Override
    public Pose autoFarStart() {
        return new Pose(45.226, 7.7, Math.toRadians(180));
    }

    @Override
    public Pose autoCloseStart() {
        return null;
    }

    @Override
    public Pose goal() {
        return new Pose(goalX, goalY, Math.toRadians(0));
    }

    @Override
    public Pose shootFar() {
        return new Pose(46.5, 15);
    }

    @Override
    public Pose getResetPose() {
        return new Pose(131, 9, Math.toRadians(0));
    }
}
