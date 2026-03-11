package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.field.TeamColor;

@Autonomous(name = "Goal RED")
public class GoalRed extends GoalBlue {
    @Override
    protected void setColor() {
        color = TeamColor.RED;
    }
}
