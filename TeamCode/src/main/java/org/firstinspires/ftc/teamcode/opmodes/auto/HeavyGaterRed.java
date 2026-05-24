package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.field.TeamColor;

@Autonomous(name = "Gate RED")
public class HeavyGaterRed extends HeavyGaterBlue {
    @Override
    protected void setColor() {
        color = TeamColor.RED;
    }
}
