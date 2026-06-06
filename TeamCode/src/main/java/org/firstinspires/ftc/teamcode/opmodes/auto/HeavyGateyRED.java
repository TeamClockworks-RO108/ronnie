package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.field.TeamColor;

@Autonomous(name = "muie tureta RED")
public class HeavyGateyRED extends HeavyGateyBLUE {

    @Override
    protected void setColor() {
        color = TeamColor.RED;
    }
}
