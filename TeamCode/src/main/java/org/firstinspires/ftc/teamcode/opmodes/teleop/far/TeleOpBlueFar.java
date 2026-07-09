package org.firstinspires.ftc.teamcode.opmodes.teleop.far;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.field.Strategy;
import org.firstinspires.ftc.teamcode.field.TeamColor;
import org.firstinspires.ftc.teamcode.opmodes.teleop.TeleOpBase;

@TeleOp(name = "TeleOp BLUE FAR")
public class TeleOpBlueFar extends TeleOpBase {
    public TeleOpBlueFar() {
        super(TeamColor.BLUE, Strategy.FAR);
    }
}
