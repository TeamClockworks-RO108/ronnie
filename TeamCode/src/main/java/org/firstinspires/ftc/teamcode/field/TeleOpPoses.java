package org.firstinspires.ftc.teamcode.field;

import com.pedropathing.geometry.Pose;

public class TeleOpPoses extends Poses {
    public Pose teleOpStart;

    public TeleOpPoses(TeamColor color) {
        super(color);
        teleOpStart = createPose(144-25, 120, 0);
    }
}
