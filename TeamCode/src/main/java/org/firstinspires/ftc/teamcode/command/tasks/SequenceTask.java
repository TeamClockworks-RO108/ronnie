package org.firstinspires.ftc.teamcode.command.tasks;

import java.util.LinkedList;
import java.util.Queue;

public class SequenceTask extends Task {
    private final Queue<Task> taskQueue = new LinkedList<>();
    private Task activeTask = null;

    public SequenceTask(Task... tasks) {
        for (Task task : tasks) {
            taskQueue.add(task);
            this.requirements.addAll(task.getRequirements());
        }
    }

    @Override
    public void init() {
        if (taskQueue.isEmpty()) return;
        activeTask = taskQueue.poll();
        activeTask.init();
    }

    @Override
    public void update() {
        activeTask.update();

        if (activeTask.isFinished()) {
            activeTask.end(false);

            if (taskQueue.isEmpty()) {
                activeTask = null;
                return;
            } else {
                activeTask = taskQueue.poll();
                activeTask.init();
            }
        }
    }

    @Override
    public boolean isFinished() {
        return taskQueue.isEmpty() && activeTask == null;
    }

    @Override
    public void end(boolean interrupted) {
        if (activeTask != null)
            activeTask.end(true);
    }
}
