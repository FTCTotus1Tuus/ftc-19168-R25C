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
            TeleOpInitializationConfig config
    ) {
        String autoAlliance = preferencesService.getAutoAlliance(config.unknownAllianceDefault);

        // Keep alliance-derived turret target setup at startup.
        turretVisionCoordinator.setAlliance(autoAlliance);

        LocalizationService.SeedResult seedResult = localizationService.seedTeleOpPose(
                autoAlliance,
                preferencesService,
                config.humanPlayerRedX,
                config.humanPlayerRedY,
                config.humanPlayerBlueX,
                config.humanPlayerBlueY,
                config.robotCenterOffsetX,
                config.robotCenterOffsetY
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


