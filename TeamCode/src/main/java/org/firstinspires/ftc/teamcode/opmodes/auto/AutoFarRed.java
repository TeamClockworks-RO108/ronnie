package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.field.TeamColor;

@Autonomous(name = "Auto Far Red")
public class AutoFarRed extends AutoFar {
    public AutoFarRed() {
        super(TeamColor.RED);
    }
}
