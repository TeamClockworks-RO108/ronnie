package org.firstinspires.ftc.teamcode.command;

import org.firstinspires.ftc.teamcode.command.tasks.Task;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.Iterator;
import java.util.List;
import java.util.Set;

public class TaskScheduler {
    private final List<Task> activeTasks = new ArrayList<>();

    public void schedule(Task task, boolean isHighPriority) {
        if (isHighPriority) {
            activeTasks.removeIf(existing -> {
                if (!Collections.disjoint(existing.getRequirements(), task.getRequirements())) {
                    existing.end(true); // end tasks of busy systems
                    return true;
                }
                return false;
            });
        } else {
            for (Task active : activeTasks) {
                if (!Collections.disjoint(active.getRequirements(), task.getRequirements()))
                    return; // skip if necessary systems are busy
            }
        }

        // start new task
        task.init();
        activeTasks.add(task);
    }
    public void schedule(Task task) {
        schedule(task, false);
    }

    public void run() {
        Iterator<Task> iterator = activeTasks.iterator();
        while (iterator.hasNext()) {
            Task task = iterator.next();

            task.update();
            if (task.isFinished()) {
                task.end(false);
                iterator.remove();
            }
        }
    }

    public void reset() {
        activeTasks.clear();
    }
}
