package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "turret encoder test")
public class turretEncoderTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        long correctionOffset = 0;

        DcMotorEx turretEncoder = hardwareMap.get(DcMotorEx.class, "leftFront");
        CRServo heading0 = hardwareMap.get(CRServo.class, "heading0");
        CRServo heading1 = hardwareMap.get(CRServo.class, "heading1");

        heading1.setDirection(DcMotorSimple.Direction.REVERSE);
        heading0.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            // hardware reset
            if (gamepad1.squareWasPressed()) {
                turretEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                turretEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                sleep(500);

                correctionOffset = - turretEncoder.getCurrentPosition();
            }

            // software reset
            if (gamepad1.circleWasPressed()) {
                correctionOffset -= turretEncoder.getCurrentPosition();
            }

            // manual controls
            if (Math.abs(gamepad1.right_stick_x) > 0.1) {
                heading0.setPower(-gamepad1.right_stick_x / 8);
                heading1.setPower(-gamepad1.right_stick_x / 8);
            }

            telemetry.addData("Raw encoder", turretEncoder.getCurrentPosition());
            telemetry.addData("Offset encoder", turretEncoder.getCurrentPosition() + correctionOffset);
        }
    }
}
