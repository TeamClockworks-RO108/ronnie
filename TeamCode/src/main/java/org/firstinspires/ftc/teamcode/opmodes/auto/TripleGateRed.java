package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.field.TeamColor;

@Autonomous(name = "Golden gate Labubu RED")
public class TripleGateRed extends TripleGateBlue {
    @Override
    protected void setColor() {
        color = TeamColor.RED;
    }
}
