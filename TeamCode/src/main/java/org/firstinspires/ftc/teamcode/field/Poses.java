package org.firstinspires.ftc.teamcode.field;

import com.pedropathing.geometry.Pose;

public class Poses {
    protected final TeamColor color;

    public Pose goalTarget;

    public Poses(TeamColor color) {
        this.color = color;
        goalTarget = createPose(138.5, 148, 0); // 144, 140

    }

    protected Pose createPose(double x, double y, double heading) {
        switch (color){
            case BLUE:
                return new Pose(144-x, y, Math.toRadians(180-heading));
            case RED:
                return new Pose(x, y, Math.toRadians(heading));
        }
        return null;
    }
}