package org.firstinspires.ftc.teamcode.field;

import android.graphics.Point;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

public class TeleOpPoses extends Poses {
    public Pose teleOpStart;
    public Pose humanBase, gateReset, gateResetCollect;

    public TeleOpPoses(TeamColor color) {
        super(color);
        teleOpStart = createPose( 144 - 63.5, 109, -70); //-70
        humanBase = createPose(9, 9, Math.toRadians(180));
        gateReset =  createPose(120 - 3, 72, 0); // center robot is center pos, then 129
        gateResetCollect  =  createPose(144 - 17, 72 - 16, 0);
    }
}
