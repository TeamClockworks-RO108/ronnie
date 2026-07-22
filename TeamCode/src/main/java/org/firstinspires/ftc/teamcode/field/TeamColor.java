package org.firstinspires.ftc.teamcode.field;

import org.firstinspires.ftc.teamcode.field.paths.Paths;
import org.firstinspires.ftc.teamcode.field.paths.PathsBlue;
import org.firstinspires.ftc.teamcode.field.paths.PathsRed;
import org.firstinspires.ftc.teamcode.field.poses.Poses;
import org.firstinspires.ftc.teamcode.field.poses.PosesBlue;
import org.firstinspires.ftc.teamcode.field.poses.PosesRed;

public enum TeamColor {
    RED(new PosesRed(), new PathsRed()),
    BLUE(new PosesBlue(), new PathsBlue());

    public final Poses poses;
    public final Paths paths;

    TeamColor(Poses poses, Paths paths) {
        this.poses = poses;
        this.paths = paths;
    }
}
