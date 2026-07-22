package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.field.TeamColor;

@Autonomous(name = "Auto Far Blue")
public class AutoFarBlue extends AutoFar {
    public AutoFarBlue() {
        super(TeamColor.BLUE);
    }
}