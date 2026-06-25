package org.firstinspires.ftc.teamcode.field;
import com.pedropathing.geometry.Pose;

public class TeleOpPoses {
    public Pose teleOpStart;
    public Pose teleopFarStart;
    public Pose humanBase, gateReset, gateResetCollect;
    public Pose blueGoal;

    public TeleOpPoses() {
        teleOpStart = new Pose(86, 107, Math.toRadians(-55));
        humanBase = new Pose(9, 9, Math.toRadians(180));
        gateReset =  new Pose(120 - 3, 72, Math.toRadians(0)); // center robot is center pos, then 129
        gateResetCollect  =  new Pose(144 - 17, 72 - 16, Math.toRadians(0));
        teleopFarStart = new Pose(8, 9, Math.toRadians(180));
        blueGoal = new Pose(6.8, 133.5, Math.toRadians(0));
    }
}
