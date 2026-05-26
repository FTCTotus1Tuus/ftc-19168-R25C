package org.firstinspires.ftc.teamcode.team.core;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.team.services.LocalizationService;
import org.firstinspires.ftc.teamcode.team.services.PreferencesService;

import java.util.Locale;

/**
 * Coordinates TeleOp startup state restoration and pre-start telemetry.
 */
public class TeleOpInitializationCoordinator {

    public static class InitializationResult {
        public final String autoAlliance;

        public InitializationResult(String autoAlliance) {
            this.autoAlliance = autoAlliance;
        }
    }

    public InitializationResult initialize(
            PreferencesService preferencesService,
            LocalizationService localizationService,
            TurretVisionCoordinator turretVisionCoordinator,
            Telemetry telemetry,
            String unknownAllianceDefault,
            double humanPlayerRedX,
            double humanPlayerRedY,
            double humanPlayerBlueX,
            double humanPlayerBlueY,
            double robotCenterOffsetX,
            double robotCenterOffsetY
    ) {
        String autoAlliance = preferencesService.getAutoAlliance(unknownAllianceDefault);

        // Keep alliance-derived turret target setup at startup.
        turretVisionCoordinator.setAlliance(autoAlliance);

        LocalizationService.SeedResult seedResult = localizationService.seedTeleOpPose(
                autoAlliance,
                preferencesService,
                humanPlayerRedX,
                humanPlayerRedY,
                humanPlayerBlueX,
                humanPlayerBlueY,
                robotCenterOffsetX,
                robotCenterOffsetY
        );

        if (seedResult.loadedFromAuto) {
            telemetry.addLine("=== ODOMETRY LOADED FROM AUTO ===");
            telemetry.addData(
                    "Loaded Position",
                    String.format(Locale.US, "X=%.1f, Y=%.1f, H=%.1f°", seedResult.x, seedResult.y, Math.toDegrees(seedResult.headingRad))
            );
        } else {
            telemetry.addLine("=== NO AUTO DATA - DEFAULT POSITION ===");
            telemetry.addData(
                    "Default Position",
                    String.format(Locale.US, "X=%.1f, Y=%.1f, H=%.1f°", seedResult.x, seedResult.y, Math.toDegrees(seedResult.headingRad))
            );
        }

        return new InitializationResult(autoAlliance);
    }
}


