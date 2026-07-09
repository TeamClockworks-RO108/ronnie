package org.firstinspires.ftc.teamcode.field;

import org.firstinspires.ftc.teamcode.field.poses.Poses;
import org.firstinspires.ftc.teamcode.field.poses.PosesBlue;
import org.firstinspires.ftc.teamcode.field.poses.PosesRed;

public enum TeamColor {
    RED(new PosesRed()),
    BLUE(new PosesBlue());

    public final Poses poses;
    TeamColor(Poses poses) {
        this.poses = poses;
    }
}
