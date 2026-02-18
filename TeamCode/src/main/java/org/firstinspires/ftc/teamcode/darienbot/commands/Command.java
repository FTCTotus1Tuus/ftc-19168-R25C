package org.firstinspires.ftc.teamcode.darienbot.commands;

public interface Command {
    void init();            // Called once when command starts

    void update();          // Called every loop

    boolean isFinished();   // Return true when done

    void end();             // Called once when finishing
}
