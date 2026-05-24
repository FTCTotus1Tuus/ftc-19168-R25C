package org.firstinspires.ftc.teamcode.team.subsystems;

/**
 * Minimal intake control contract used by coordinators.
 */
public interface IntakeControl {
    void startIntaking();
    void reverse();
    void off();
    boolean isIntaking();
}

