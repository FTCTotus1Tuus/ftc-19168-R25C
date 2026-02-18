package org.firstinspires.ftc.teamcode.darienbot.commands;

import org.firstinspires.ftc.teamcode.darienbot.subsystems.TraySubsystem;

public class TrayCommand implements Command {
    private final TraySubsystem tray;
    private final double targetPosition;
    private final TraySubsystem.Owner owner;
    private boolean accepted = false;

    // NEW: time-based completion
    private long startTime = 0;
    // tweak this if tray is slower/faster
    private static final long MOVE_TIMEOUT_MS = 500;

    public TrayCommand(TraySubsystem tray, double targetPosition, TraySubsystem.Owner owner) {
        this.tray = tray;
        this.targetPosition = targetPosition;
        this.owner = owner;
    }

    @Override
    public void init() {
        accepted = tray.requestOwnership(owner);

        if (accepted) {
            tray.setTrayPosition(targetPosition);
            startTime = System.currentTimeMillis();
        }
    }

    @Override
    public void update() {
        // nothing needed here; subsystem handles movement
    }

    @Override
    public boolean isFinished() {
        if (!accepted) return true;   // allow auto to continue if ownership denied

        // Non-blocking time-based completion
        long elapsed = System.currentTimeMillis() - startTime;
        return elapsed >= MOVE_TIMEOUT_MS;
    }

    @Override
    public void end() {
        if (accepted) {
            tray.releaseOwnership(owner);
        }
    }
}
