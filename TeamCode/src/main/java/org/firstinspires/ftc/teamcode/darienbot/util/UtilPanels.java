package org.firstinspires.ftc.teamcode.darienbot.util;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.Path;

@Configurable // Panels
public class UtilPanels {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance

    public static void draw(Follower follower, Path activePath) {

        /*
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        // Draw active path
        if (activePath != null) {
            Drawing.drawPath(panelsTelemetry.getOverlay(), activePath);
        }

        // Draw robot pose
        Drawing.drawRobot(panelsTelemetry.getOverlay(), follower.getPoseEstimate());

        // Render frame
        panelsTelemetry.update(telemetry);

         */
    }
}
