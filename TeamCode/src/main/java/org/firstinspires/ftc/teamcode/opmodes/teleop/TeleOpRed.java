package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.field.TeamColor;

@TeleOp(name = "TeleOp RED")
public class TeleOpRed extends TeleOpBlue {
    @Override
    public void init() {
        color = TeamColor.RED;
        super.init();
        movement.flipControls();
    }
}