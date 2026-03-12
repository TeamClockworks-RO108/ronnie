package org.firstinspires.ftc.teamcode.field;

import com.pedropathing.geometry.Pose;

public class AutoPoses extends Poses {
    // intake positions
    protected static final double
            INTAKE_START_X = 98,
            INTAKE_3_END_X = 144-18.2, INTAKE_3_Y = 84,
            INTAKE_2_END_X = 144-9.8, INTAKE_2_Y = INTAKE_3_Y - 24.3,
            INTAKE_1_END_X = 144-9.8, INTAKE_1_Y = INTAKE_3_Y - 48;

    public Pose
            goalStart, goalHome,
            farStart, farHome;
    public Pose goalShoot, centerAutoShoot, farShoot;
    public Pose
            intake3Prep, intake3Take,
            intake2Prep, intake2Take,
            intake1Prep, intake1Take;
    public Pose
            gateCorner,
            gateOpenPrep, gateOpenEnd;

    public Pose gateIntake, gateIntakePrep, gateTurnBeforeShoot;

    public AutoPoses(TeamColor color) {
        super(color);

        // start specific poses
        goalStart = createPose(120,120, 0);
        goalHome = createPose(116, 90, 0);

        // shooting poses
        goalShoot = createPose(96, 96, 0);
        centerAutoShoot = createPose( 84, 84, 0 );

        // intake poses
        intake3Prep = createPose(INTAKE_START_X, INTAKE_3_Y, 0);
        intake3Take = createPose(INTAKE_3_END_X, INTAKE_3_Y, 0);
        intake2Prep = createPose(INTAKE_START_X, INTAKE_2_Y, 0);
        intake2Take = createPose(INTAKE_2_END_X, INTAKE_2_Y, 0);
        intake1Prep = createPose(INTAKE_START_X, INTAKE_1_Y, 0);
        intake1Take = createPose(INTAKE_1_END_X, INTAKE_1_Y, 0);

        // gate poses
        gateCorner = createPose(115, 60, 0);

        gateIntake = createPose ( 132.5, 57.57, 35);
        gateIntakePrep = createPose( 114, 57.5, 35);

        gateTurnBeforeShoot = createPose(100, 84, 0 );
    }
}
