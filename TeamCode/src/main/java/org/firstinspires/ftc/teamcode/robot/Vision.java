package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.field.TeamColor;

public class Vision {
    private final Limelight3A limelight;
    private final TeamColor color;
    private final Follower follower;
    private final Telemetry telemetry;
    private boolean isUpdatingFollower = true;

    public Vision(HardwareMap hardwareMap, Telemetry telemetry, Follower follower, TeamColor color) {
        this.color = color;
        this.follower = follower;
        this.telemetry = telemetry;

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(0);
    }

    public void update() {
        try {
            LLResult data = getLatestData();
            Pose cameraPose = processVisionPose(data);

            if (isUpdatingFollower)
                follower.setPose(cameraPose);

            telemetry.addData("Camera Pose", cameraPose.toString());
        } catch (RuntimeException e) {
            telemetry.addData("Camera Pose", "invalid / out of sight");
        }
        telemetry.addData("Updating follower to camera", isUpdatingFollower);
    }

    public void toggleFollowerUpdate() {
        isUpdatingFollower = !isUpdatingFollower;
    }

    private LLResult getLatestData() throws RuntimeException {
        LLResult result = limelight.getLatestResult();

        if (result == null)
            throw new RuntimeException("Invalid Camera Data");
        if (!result.isValid())
            throw new RuntimeException("Invalid Camera Data");

        return result;
    }

    private Pose processVisionPose(LLResult data) throws RuntimeException {
        Pose3D poseFTC = data.getBotpose();
        Pose pedroPose = new Pose(
                poseFTC.getPosition().y * 39.37008 + 144.0/2,
                -poseFTC.getPosition().x * 39.37008 + 144.0/2,
                poseFTC.getOrientation().getYaw(AngleUnit.RADIANS) + Math.PI * 3/2
        );

        Pose cameraPose;
        // fine tuned. hardcoded offsets seem to not be necessary
        switch (color) {
            case BLUE:
                cameraPose = new Pose(pedroPose.getX(), pedroPose.getY(), pedroPose.getHeading());
                break;
            case RED:
                cameraPose = new Pose(pedroPose.getX(), pedroPose.getY(), pedroPose.getHeading());
                break;
            default:
                cameraPose = new Pose(0, 0, 0); // dw about this
        }

        return cameraPose;
    }
}