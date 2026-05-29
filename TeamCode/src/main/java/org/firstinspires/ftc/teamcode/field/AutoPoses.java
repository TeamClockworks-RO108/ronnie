package org.firstinspires.ftc.teamcode.field;

import com.pedropathing.geometry.Pose;

public class AutoPoses extends Poses {
    // intake positions
    protected static final double
            INTAKE_START_X = 95,
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

    public Pose gateIntakeTake, gateIntakePrep, gateToShootIntermediary,
            turnToOpenGate, turnToOpenGate0,
            centerShoot45,
            intCollectFromHuman2, finalCollectFromHuman2;
    public Pose leaveShoot, gateShoot;

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
        centerShootFromHumanPlusLeave = createPose(85, 108, -70);
        centerShootFromThird = createPose(144 - 58, 144 - 58, -90);
        centerShoot45 = createPose(144 - 58, 144 - 58, -45);

        farShoot = createPose(0,0, 60);
        farLeave = createPose(5, 0, 0);

        gateShoot = createPose(144 - 54, 144 - 56, 0);

        // intake poses
        intake3Prep = createPose(INTAKE_START_X, INTAKE_3_Y, 0);
        intake3Take = createPose(INTAKE_3_END_X, INTAKE_3_Y, 0);
        intake2Prep = createPose(INTAKE_START_X, INTAKE_2_Y, 0);
        intake2Take = createPose(INTAKE_2_END_X, INTAKE_2_Y, 0);
        intake1Prep = createPose(INTAKE_START_X, INTAKE_1_Y, 0);
        intake1Take = createPose(INTAKE_1_END_X, INTAKE_1_Y, 0);

        // gate poses
        gateCorner = createPose(115, 60, 0);

        gateToShootIntermediary = createPose(100, 72, 0);
        leaveShoot = createPose(86, 107, -55);

        gateIntakeTake = createPose (135.2, 56.8, 27);
        gateIntakePrep = createPose(108.1, 60.8, 27);

        turnToOpenGate = createPose (120.5, 62.5, -45);
        turnToOpenGate0 = createPose (125, 66.5, -90);

        // human collect poses
        prepareToCollectFromHuman = createPose(144 - 64, 85, -70);
        goCollectFromHuman = createPose ( 120, 22, -80);

        intCollectFromHuman  = createPose (122, 45 ,-70);
        finalCollectFromHuman  = createPose (129, 18.5, - 85);

        intCollectFromHuman2  = createPose (125, 45, - 70);
        finalCollectFromHuman2  = createPose (130, 10, - 85);
    }
}