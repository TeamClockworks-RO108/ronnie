package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.geometry.Pose;

import java.util.Optional;

public class RobotContext {
    private static Pose lastPose = null;

    public static Optional<Pose> getLastPose() {
        return Optional.ofNullable(lastPose);
    }
    public static void setLastPose(Pose pose) {
        lastPose = pose;
    }
}
