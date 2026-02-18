package org.firstinspires.ftc.teamcode.darienbot.commands;

import static java.lang.Thread.sleep;

import org.firstinspires.ftc.teamcode.darienbot.subsystems.ScoringSubsystem;

public class ShootArtifactCommand implements Command {

    private final ScoringSubsystem scoring;
    private long startTime = 0;

    // Track which step we are in
    private int stage = 0;
    private boolean finished = false;

    private static final long STAGE1_DELAY = 100;   // after Elevator up
    private static final long STAGE2_DELAY = 600;   // ejection motors running before feeding
    private static final long STAGE3_DELAY = 500;   // feeder up while spinning


    public ShootArtifactCommand(ScoringSubsystem scoring) {
        this.scoring = scoring;
    }

    @Override
    public void init() {
        startTime = System.currentTimeMillis();
        stage = 0;
        finished = false;

        // Stage 0: Move elevator up immediately
        scoring.setElevatorUp();
    }

    @Override
    public void update() {
        long elapsed = System.currentTimeMillis() - startTime;

        switch (stage) {

            case 0:
                if (elapsed >= STAGE1_DELAY) {
                    scoring.startShotgun();
                    stage = 1;
                }
                break;

            case 1:
                if (elapsed >= STAGE1_DELAY + STAGE2_DELAY) {
                    scoring.setFeederUp();
                    stage = 2;
                }
                break;

            case 2:
                if (elapsed >= STAGE1_DELAY + STAGE2_DELAY + STAGE3_DELAY) {
                    scoring.stopShotgun();
                    scoring.setFeederDown();
                    scoring.setElevatorDown();
                    finished = true;
                }
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return finished;
    }

    @Override
    public void end() {
        // Ensure everything is off
        scoring.stopShotgun();
        scoring.setFeederDown();
        scoring.setElevatorDown();
    }
}
