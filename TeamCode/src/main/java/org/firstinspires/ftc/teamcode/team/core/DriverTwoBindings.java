package org.firstinspires.ftc.teamcode.team.core;

/**
 * Immutable snapshot of driver two controls for a single TeleOp loop.
 */
public class DriverTwoBindings {

    public final boolean closeGateRequested;
    public final boolean shootPressed;
    public final boolean shootReleased;
    public final double shootingStickY;

    public final boolean setAllianceRedRequested;
    public final boolean setAllianceBlueRequested;

    public final boolean odometryModeRequested;
    public final boolean cameraModeRequested;

    public final double turretManualAxis;
    public final double turretSpeedTrigger;
    public final boolean turretCenterRequested;

    public final boolean toggleShotgunPowerLatchRequested;
    public final boolean forceShotgunLowPowerRequested;

    public DriverTwoBindings(
            boolean closeGateRequested,
            boolean shootPressed,
            boolean shootReleased,
            double shootingStickY,
            boolean setAllianceRedRequested,
            boolean setAllianceBlueRequested,
            boolean odometryModeRequested,
            boolean cameraModeRequested,
            double turretManualAxis,
            double turretSpeedTrigger,
            boolean turretCenterRequested,
            boolean toggleShotgunPowerLatchRequested,
            boolean forceShotgunLowPowerRequested
    ) {
        this.closeGateRequested = closeGateRequested;
        this.shootPressed = shootPressed;
        this.shootReleased = shootReleased;
        this.shootingStickY = shootingStickY;
        this.setAllianceRedRequested = setAllianceRedRequested;
        this.setAllianceBlueRequested = setAllianceBlueRequested;
        this.odometryModeRequested = odometryModeRequested;
        this.cameraModeRequested = cameraModeRequested;
        this.turretManualAxis = turretManualAxis;
        this.turretSpeedTrigger = turretSpeedTrigger;
        this.turretCenterRequested = turretCenterRequested;
        this.toggleShotgunPowerLatchRequested = toggleShotgunPowerLatchRequested;
        this.forceShotgunLowPowerRequested = forceShotgunLowPowerRequested;
    }
}

