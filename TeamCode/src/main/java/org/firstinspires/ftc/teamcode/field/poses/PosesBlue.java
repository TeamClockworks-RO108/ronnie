package org.firstinspires.ftc.teamcode.field.poses;
import com.pedropathing.geometry.Pose;

public class PosesBlue implements Poses{
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
        return new Pose(2.95, 135.4, Math.toRadians(0));
    }

    @Override
    public Pose getShoot() {
        return new Pose(45.226, 15);
    }
}
