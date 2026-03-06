package org.firstinspires.ftc.teamcode.command.tasks;

import java.util.function.Supplier;

public class ConditionalTask extends Task {
    private final Task taskOnTrue;
    private final Supplier<Boolean> condition;
    private boolean conditionMet = false;

    public ConditionalTask(Task taskOnTrue, Supplier<Boolean> condition) {
        this.taskOnTrue = taskOnTrue;
        this.condition = condition;
    }

    @Override
    public void init() {
        conditionMet = condition.get();
        if (conditionMet)
            taskOnTrue.init();
    }
    @Override
    public void update() {
        if (conditionMet) taskOnTrue.update();
    }
    @Override
    public boolean isFinished() {
        return !conditionMet || taskOnTrue.isFinished();
    }
    @Override
    public void end(boolean interrupted) {
        if (conditionMet)
            taskOnTrue.end(interrupted);
    }
}
