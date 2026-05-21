package org.firstinspires.ftc.teamcode.field;

import com.pedropathing.geometry.Pose;

public class AutoPoses extends Poses {
    // intake positions
    protected static final double
            INTAKE_START_X = 88,
            INTAKE_3_END_X = 144 - 27, INTAKE_3_Y = 84,
            INTAKE_2_END_X = 144 - 23, INTAKE_2_Y = 58,
            INTAKE_1_END_X = 144 - 17, INTAKE_1_Y = 36;

    public Pose
            goalStart, goalHome,
            farStart, farHome, farShoot, farLeave;
    public Pose closeShoot, centerShoot, centerShootFromHuman, centerShootFromHumanPlusLeave , centerShootFromThird;
    public Pose
            intake3Prep, intake3Take,
            intake2Prep, intake2Take,
            intake1Prep, intake1Take;
    public Pose gateCorner;

    public Pose gateIntakeTake, turnToOpenGate, turnToOpenGate0,  gateIntakePrep, gateLeaveTurn, gateTurnBeforeShoot, centerShoot45, intCollectFromHuman2, finalCollectFromHuman2;

    public Pose prepareToCollectFromHuman, goCollectFromHuman;
    public Pose intCollectFromHuman, finalCollectFromHuman;

    public AutoPoses(TeamColor color) {
        super(color);

        // start specific poses
        goalStart = createPose(144 - 26, 144 - 26, 0);
        goalHome = createPose(116, 90, 0);

        // shooting poses
        centerShoot = createPose(144 - 56, 144 - 56, 0);
        centerShootFromHuman = createPose(144 - 56, 144 - 56, -70);
        centerShootFromHumanPlusLeave = createPose(85, 109, -70);
        centerShootFromThird = createPose(144 - 58, 144 - 58, -90);
        centerShoot45 = createPose(144 - 58, 144 - 58, -45);

        farShoot = createPose (0,0, 60);
        farLeave = createPose(5, 0, 0);

        // intake poses
        intake3Prep = createPose(INTAKE_START_X, INTAKE_3_Y, 0);
        intake3Take = createPose(INTAKE_3_END_X, INTAKE_3_Y, 0);
        intake2Prep = createPose(INTAKE_START_X, INTAKE_2_Y, 0);
        intake2Take = createPose(INTAKE_2_END_X, INTAKE_2_Y, 0);
        intake1Prep = createPose(INTAKE_START_X, INTAKE_1_Y, 0);
        intake1Take = createPose(INTAKE_1_END_X, INTAKE_1_Y, 0);

        // gate poses
        gateCorner = createPose(115, 60, 0);

        gateIntakeTake = createPose (132 - 9, 61.5, 36.5);
        gateIntakePrep = createPose(114 - 9, 61.5, 36.5);

        turnToOpenGate = createPose (124, 65, -45);
        turnToOpenGate0 = createPose (124.5, 68, -90);

        gateTurnBeforeShoot = createPose(130 - 9 , 63.5, 0);

        prepareToCollectFromHuman = createPose(144 - 64, 85, -70);
        goCollectFromHuman = createPose ( 120, 22, -80);

        intCollectFromHuman  = createPose (122, 45 ,-70);
        finalCollectFromHuman  = createPose (129, 18.5, - 85);

        intCollectFromHuman2  = createPose (125, 45, - 70);
        finalCollectFromHuman2  = createPose (130, 10, - 85);

        gateLeaveTurn = createPose(100 - 9 , 60, 0);
    }
}
