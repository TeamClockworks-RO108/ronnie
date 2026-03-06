package org.firstinspires.ftc.teamcode.command.tasks;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.command.Subsystem;

public class WaitTask extends Task {
    ElapsedTime timer;
    long targetTime;

    public WaitTask(long milliseconds, Subsystem... requirements) {
        targetTime = milliseconds;
        addRequirements(requirements);
    }

    @Override
    public void init() {
        timer = new ElapsedTime();
    }

    @Override
    public void update() {}

    @Override
    public boolean isFinished() {
        return timer.milliseconds() >= targetTime;
    }

    @Override
    public void end(boolean interrupted) {}
}
