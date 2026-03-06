package org.firstinspires.ftc.teamcode.command.tasks;

import org.firstinspires.ftc.teamcode.command.Subsystem;

public class InstantTask extends Task{
    private final Runnable action;

    public InstantTask(Runnable action, Subsystem... requirements) {
        this.action = action;
        addRequirements(requirements);
    }

    @Override
    public void init() {
        action.run();
    }
    @Override
    public void update() {}
    @Override
    public boolean isFinished() {
        return true;
    }
    @Override
    public void end(boolean interrupted) {}
}
