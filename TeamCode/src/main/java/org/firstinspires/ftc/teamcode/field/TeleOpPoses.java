package org.firstinspires.ftc.teamcode.field;

import com.pedropathing.geometry.Pose;

public class TeleOpPoses extends Poses {
    public Pose teleOpStart;
    public Pose humanBase, gateReset;

    public TeleOpPoses(TeamColor color) {
        super(color);
        teleOpStart = createPose(120, 120, 0);
        humanBase = createPose(144-9, 9, 0);
        gateReset = createPose(127, 72, 0);
    }
}
