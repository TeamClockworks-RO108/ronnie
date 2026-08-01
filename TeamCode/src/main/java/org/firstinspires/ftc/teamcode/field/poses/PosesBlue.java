package org.firstinspires.ftc.teamcode.field.poses;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

@Configurable
public class PosesBlue implements Poses{
    public static double goalX = 13;
    public static double goalY = 135.4;

    @Override
    public Pose teleopCloseStart() {
        return new Pose(86, 107, Math.toRadians(-55));
    }

    @Override
    public Pose teleopFarStart() {
        return new Pose(7.848, 10.596, Math.toRadians(180));
    }

    @Override
    public Pose autoFarStart() {
        return new Pose(44.135, 10.516, Math.toRadians(180));
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
        return new Pose(46.455, 17.392);
    }

    @Override
    public Pose getResetPose() {
        return new Pose(133, 8.5528, Math.toRadians(0));
    }
}
