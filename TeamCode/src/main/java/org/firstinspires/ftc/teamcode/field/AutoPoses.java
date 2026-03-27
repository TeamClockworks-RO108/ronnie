package org.firstinspires.ftc.teamcode.field;

import com.pedropathing.geometry.Pose;

public class AutoPoses extends Poses {
    // intake positions
    protected static final double
            INTAKE_START_X = 98,
            INTAKE_3_END_X = 144-19, INTAKE_3_Y = 84,
            INTAKE_2_END_X = 144-15.5, INTAKE_2_Y = INTAKE_3_Y - 25.0,
            INTAKE_1_END_X = 144-10.5, INTAKE_1_Y = INTAKE_3_Y - 48;

    public Pose
            goalStart, goalHome,
            farStart, farHome, farShoot, farLeave;
    public Pose closeShoot, centerShoot;
    public Pose
            intake3Prep, intake3Take,
            intake2Prep, intake2Take,
            intake1Prep, intake1Take;
    public Pose gateCorner;

    public Pose gateIntakeTake, gateIntakePrep, gateLeaveTurn, gateTurnBeforeShoot;

    public AutoPoses(TeamColor color) {
        super(color);

        // start specific poses
        goalStart = createPose(120,120, 0);
        goalHome = createPose(116, 90, 0);

        // shooting poses
        closeShoot = createPose(96, 96, 0);
        centerShoot = createPose( 86, 86, 0 );

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

        gateIntakeTake = createPose ( 132, 53.5, 36.5);
        gateIntakePrep = createPose( 114, 53.5, 36.5);

        gateTurnBeforeShoot = createPose(130, 53.5, 0 );



        gateLeaveTurn = createPose(100, 60, 0);
    }

    public void counterOffsetGate(){
        gateIntakeTake = new Pose(gateIntakeTake.getX(), gateIntakePrep.getY() - 2.5);
        gateIntakePrep = new Pose(gateIntakePrep.getX(), gateIntakePrep.getY() - 2.5);
    }
}
