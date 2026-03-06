package org.firstinspires.ftc.teamcode.command.tasks;

import org.firstinspires.ftc.teamcode.command.Subsystem;

import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;
import java.util.function.Supplier;

public abstract class Task {
    protected Set<Subsystem> requirements = new HashSet<>();

    public abstract void init();
    public abstract void update();
    public abstract boolean isFinished();
    public abstract void end(boolean interrupted);

    public SequenceTask then(Task next) {
        return new SequenceTask(this, next);
    }
    public ConditionalTask onCondition(Supplier<Boolean> condition) {
        return new ConditionalTask(this, condition);
    }

    public void addRequirements(Subsystem... subsystems) {
        requirements.addAll(Arrays.asList(subsystems));
    }
    public Set<Subsystem> getRequirements() {
        return requirements;
    }
}
