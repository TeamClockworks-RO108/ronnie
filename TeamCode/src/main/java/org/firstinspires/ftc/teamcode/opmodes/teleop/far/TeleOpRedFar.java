package org.firstinspires.ftc.teamcode.opmodes.teleop.far;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.field.Strategy;
import org.firstinspires.ftc.teamcode.field.TeamColor;
import org.firstinspires.ftc.teamcode.opmodes.teleop.TeleOpBase;

@TeleOp(name = "TeleOp RED FAR")
public class TeleOpRedFar extends TeleOpBase {
    public TeleOpRedFar() {
        super(TeamColor.RED, Strategy.FAR);
    }

    @Override
    public void init() {
        movement.flipControls();
        super.init();
    }
}