package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Intake;

@TeleOp(name = "Test Opmode")
public class Test extends OpMode {
    @Override
    public void init() {
        Intake intake = new Intake(hardwareMap, telemetry);
        intake.manual(1);
    }

    @Override
    public void loop() {

    }
}
