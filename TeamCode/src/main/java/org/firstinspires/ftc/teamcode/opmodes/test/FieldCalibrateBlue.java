package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.field.AutoPoses;
import org.firstinspires.ftc.teamcode.field.TeamColor;
import org.firstinspires.ftc.teamcode.opmodes.teleop.TeleOpBlue;
import org.firstinspires.ftc.teamcode.robot.PedroMovement;

@TeleOp(name = "Calibrate BLUE")
public class FieldCalibrateBlue extends TeleOpBlue {
    @Override
    public void init() {
        AutoPoses autoPoses = new AutoPoses(color);

        super.init();

        turret.disable();
        intake.disable();

        movement = new PedroMovement(hardwareMap, telemetry, autoPoses.goalStart);
    }

    @Override
    public void loop() {
        super.loop();
    }
}
