package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.field.TeamColor;

@TeleOp(name = "Calibrate RED")
public class FieldCalibrateRed extends FieldCalibrateBlue {
    @Override
    public void init() {
        color = TeamColor.RED;
        super.init();
    }
}
