package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Turret {
    private static final double HOOD_LIFTED = 0.8, HOOD_LOWERED = 0.2;

    private static final double BARRIER_ON = 0.47, BARRIER_OFF = 0.7;
    private final CRServo headingServo1;
    private final CRServo headingServo2;
    private final Servo hoodServo;
    private Servo barrier;
    private boolean isHoodRaised = false;

    public Turret(HardwareMap hardwareMap) {
        headingServo1 = hardwareMap.get(CRServo.class, "heading1");
        headingServo2 = hardwareMap.get(CRServo.class, "heading2");
        hoodServo = hardwareMap.get(Servo.class, "hood");
        barrier = hardwareMap.get(Servo.class, "barrier");

        headingServo2.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void rotate(double power) {
        headingServo1.setPower(power);
        headingServo2.setPower(power);
    }
    public void toggleHood() {
        if (isHoodRaised)
            lowerHood();
        else
            liftHood();
    }

    public void liftHood() {
        isHoodRaised = true;
        hoodServo.setPosition(HOOD_LIFTED);
    }

    public void liftBarrier() {
        barrier.setPosition(BARRIER_OFF);
    }

    public void lowerBarrier() {
        barrier.setPosition(BARRIER_ON);
    }
    public void lowerHood() {
        isHoodRaised = false;
        hoodServo.setPosition(HOOD_LOWERED);
    }
}
