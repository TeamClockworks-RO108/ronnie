package field;

import com.pedropathing.geometry.Pose;

public class AutoPoses extends Poses {
    // intake positions
    protected static final double
            INTAKE_START_X = 98,
            INTAKE_ANGLE = 0,
            INTAKE_3_END_X = 144-18.2, INTAKE_3_Y = 84,
            INTAKE_2_END_X = 144-9.8, INTAKE_2_Y = INTAKE_3_Y - 24.3,
            INTAKE_1_END_X = 144-9.8, INTAKE_1_Y = INTAKE_3_Y - 48;

    public Pose
            goalStart, goalHome,
            farStart, farHome;
    public Pose goalShoot, farShoot;
    public Pose
            intake3Prep, intake3Take,
            intake2Oreo, intake2Take,
            intake1Prep, intake1Take;

    public AutoPoses(TeamColor color) {
        super(color);


    }
}
