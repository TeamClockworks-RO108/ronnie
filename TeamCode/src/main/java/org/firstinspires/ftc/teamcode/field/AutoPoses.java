package org.firstinspires.ftc.teamcode.field;

import com.pedropathing.geometry.Pose;

public class AutoPoses extends Poses {
    // intake positions
    protected static final double
            INTAKE_START_X = 98-9,
            INTAKE_3_END_X = 144- 29 , INTAKE_3_Y = 85,
            INTAKE_2_END_X = 144 - 22 , INTAKE_2_Y = 60 ,
            INTAKE_1_END_X = 144-10.5 - 9 , INTAKE_1_Y = INTAKE_2_Y - 22;

    public Pose
            goalStart, goalHome,
            farStart, farHome, farShoot, farLeave;
    public Pose closeShoot, centerShoot, centerShootFromHuman, centerShootFromHumanPlusLeave , centerShootFromThird;
    public Pose
            intake3Prep, intake3Take,
            intake2Prep, intake2Take,
            intake1Prep, intake1Take;
    public Pose gateCorner;

    public Pose gateIntakeTake, turnToOpenGate, turnToOpenGate0,  gateIntakePrep, gateLeaveTurn, gateTurnBeforeShoot;

    public Pose prepareToCollectFromHuman, goCollectFromHuman;
    public Pose intCollectFromHuman, finalCollectFromHuman;

    public AutoPoses(TeamColor color) {
        super(color);

        // start specific poses
        goalStart = createPose(144 - 30 ,121, 0);
        goalHome = createPose(116, 90, 0);

        // shooting poses
      //   closeShoot = createPose(144- 38 - 20, 124 - 20, 0);
        centerShoot = createPose( 144 - 64, 85, 0 );
        centerShootFromHuman = createPose( 144 - 64, 85,  - 70  );
        centerShootFromHumanPlusLeave = createPose( 144 - 63.5, 109, - 70);
        centerShootFromThird = createPose( 144 - 64, 85, - 90);

        farShoot = createPose (0,0 , 60 );
        farLeave = createPose(5, 0, 0 );


        // intake poses
        intake3Prep = createPose(INTAKE_START_X, INTAKE_3_Y, 0);
        intake3Take = createPose(INTAKE_3_END_X, INTAKE_3_Y, 0);
        intake2Prep = createPose(INTAKE_START_X, INTAKE_2_Y, 0);
        intake2Take = createPose(INTAKE_2_END_X, INTAKE_2_Y, 0);
        intake1Prep = createPose(INTAKE_START_X, INTAKE_1_Y, 0);
        intake1Take = createPose(INTAKE_1_END_X, INTAKE_1_Y, 0);

        //

        // gate poses
        gateCorner = createPose(115, 60, 0);

        gateIntakeTake = createPose ( 132 - 9 , 61.5, 36.5);
        gateIntakePrep = createPose( 114 - 9 , 61.5, 36.5);

        turnToOpenGate = createPose ( 144 - 26.5, 66 , -45);
        turnToOpenGate0 = createPose ( 144 - 23 , 68.5, -90);
        // turnToOpenGate0= createPose ( 130 - 27,  75,   -90 );

        gateTurnBeforeShoot = createPose(130 - 9 , 63.5, 0 );

        prepareToCollectFromHuman = createPose(144 - 64 , 85 , - 70 );
        goCollectFromHuman = createPose ( 120 , 22,  -80);

        intCollectFromHuman  = createPose (122, 45 , - 70);
        finalCollectFromHuman  = createPose (130, 22 , - 85);



        gateLeaveTurn = createPose(100 - 9 , 60, 0);
    }

    public void counterOffsetGate(){
        gateIntakeTake = new Pose(gateIntakeTake.getX(), gateIntakePrep.getY() - 2.5);
        gateIntakePrep = new Pose(gateIntakePrep.getX(), gateIntakePrep.getY() - 2.5);
    }
}
