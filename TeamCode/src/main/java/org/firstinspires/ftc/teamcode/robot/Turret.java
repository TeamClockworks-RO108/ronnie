package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Turret {
    private final CRServo headingServo1;
    private final CRServo headingServo2;
    private final Servo hoodServo;
    private boolean isHoodRaised = false;

    public Turret(HardwareMap hardwareMap) {
        headingServo1 = hardwareMap.get(CRServo.class, "heading1");
        headingServo2 = hardwareMap.get(CRServo.class, "heading2");
        hoodServo = hardwareMap.get(Servo.class, "hood");

        headingServo2.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void rotate(double power) {
        headingServo1.setPower(power);
        headingServo2.setPower(power);
    }
    public void toggleHood() {
        isHoodRaised = !isHoodRaised;
        if (isHoodRaised)
            raiseHood();
        else
            lowerHood();
    }

    private void raiseHood() {
        hoodServo.setPosition(1);
    }
    private void lowerHood() {
        hoodServo.setPosition(0);
    }
}
