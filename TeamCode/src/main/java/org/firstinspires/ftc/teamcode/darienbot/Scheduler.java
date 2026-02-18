package org.firstinspires.ftc.teamcode.darienbot;

import org.firstinspires.ftc.teamcode.darienbot.commands.Command;

import java.util.LinkedList;
import java.util.Queue;

public class Scheduler {
    private final Queue<Command> commandQueue = new LinkedList<>();
    private Command current = null;

    /**
     * Schedule a command to run sequentially
     */
    public void schedule(Command cmd) {
        commandQueue.add(cmd);
    }

    public void update() {
        if (current == null && !commandQueue.isEmpty()) {
            current = commandQueue.poll();
            current.init();
        }

        if (current != null) {
            current.update();
            if (current.isFinished()) {
                current.end();
                current = null;
            }
        }
    }

    /**
     * For debugging / telemetry
     */
    public Command getCurrentCommand() {
        return current;
    }

    /**
     * True if no commands are left to run
     */
    public boolean isFinished() {
        return current == null && commandQueue.isEmpty();
    }
}
