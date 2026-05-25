package org.firstinspires.ftc.teamcode.team.core;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.team.services.LocalizationService;

import java.util.Locale;

/**
 * Coordinates TeleOp odometry reset behavior.
 */
public class OdometryResetCoordinator {

    public void tryResetToHumanPlayerPosition(
            boolean resetRequested,
            String autoAlliance,
            double humanPlayerRedX,
            double humanPlayerRedY,
            double humanPlayerBlueX,
            double humanPlayerBlueY,
            double robotCenterOffsetX,
            double robotCenterOffsetY,
            LocalizationService localizationService,
            Telemetry telemetry
    ) {
        if (!resetRequested) {
            return;
        }

        LocalizationService.SeedResult reset = localizationService.resetToHumanPlayerPosition(
                autoAlliance,
                humanPlayerRedX,
                humanPlayerRedY,
                humanPlayerBlueX,
                humanPlayerBlueY,
                robotCenterOffsetX,
                robotCenterOffsetY
        );

        if ("RED".equals(autoAlliance)) {
            telemetry.addLine("ODOMETRY RESET: Red Human Player Position (0, 0)");
        } else if ("BLUE".equals(autoAlliance)) {
            telemetry.addLine("ODOMETRY RESET: Blue Human Player Position (144, 0)");
        }

        telemetry.addData(
                "New Odometry Position",
                String.format(Locale.US, "(%.1f, %.1f, %.1f)", reset.x, reset.y, Math.toDegrees(reset.headingRad))
        );
    }
}


