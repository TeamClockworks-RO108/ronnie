package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Lift {

    private Servo rightLift;
    private Servo leftLift;


    private double  UP_R = 0.35, DOWN_R = 0.3,
                    UP_L = 0.3, DOWN_L = 0.35;


    private boolean robotLifted ;

    public Lift(HardwareMap hardwareMap, Telemetry telemetry, boolean initialPose){
        rightLift = hardwareMap.get(Servo.class, "rightLift");
        leftLift = hardwareMap.get(Servo.class, "leftLift");


        if(initialPose)
            lift();
        else drop();

        robotLifted = initialPose;
    }

    public void lift(){
        rightLift.setPosition(UP_R);
        leftLift.setPosition(UP_L);

        robotLifted = true;
    }


    public void drop(){
        rightLift.setPosition(DOWN_R);
        leftLift.setPosition(DOWN_L);

        robotLifted = false;
    }


    public boolean getLiftStatus(){
        return robotLifted;

    }

}
