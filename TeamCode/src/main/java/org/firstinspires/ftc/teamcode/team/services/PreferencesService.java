package org.firstinspires.ftc.teamcode.team.services;

import android.content.Context;
import android.content.SharedPreferences;

import java.util.Objects;

/**
 * Wrapper around SharedPreferences keys used for Auto -> TeleOp handoff.
 */
public class PreferencesService {

    private static final String PREFS_NAME = "ftc_prefs";
    private static final String KEY_AUTO_ALLIANCE = "auto_alliance";
    private static final String KEY_AUTO_FINAL_X = "auto_final_x";
    private static final String KEY_AUTO_FINAL_Y = "auto_final_y";
    private static final String KEY_AUTO_FINAL_HEADING = "auto_final_heading";

    private final SharedPreferences prefs;

    public PreferencesService(Context context) {
        Context safeContext = Objects.requireNonNull(context, "Context must not be null");
        prefs = safeContext.getSharedPreferences(PREFS_NAME, Context.MODE_PRIVATE);
    }

    public void saveAutoAlliance(String alliance) {
        prefs.edit().putString(KEY_AUTO_ALLIANCE, alliance).apply();
    }

    public String getAutoAlliance(String defaultValue) {
        return prefs.getString(KEY_AUTO_ALLIANCE, defaultValue);
    }

    public void saveAutoFinalPose(float x, float y, float headingRad) {
        prefs.edit()
                .putFloat(KEY_AUTO_FINAL_X, x)
                .putFloat(KEY_AUTO_FINAL_Y, y)
                .putFloat(KEY_AUTO_FINAL_HEADING, headingRad)
                .apply();
    }

    public boolean hasAutoFinalPose() {
        return prefs.contains(KEY_AUTO_FINAL_X);
    }

    public float getAutoFinalX(float defaultValue) {
        return prefs.getFloat(KEY_AUTO_FINAL_X, defaultValue);
    }

    public float getAutoFinalY(float defaultValue) {
        return prefs.getFloat(KEY_AUTO_FINAL_Y, defaultValue);
    }

    public float getAutoFinalHeading(float defaultValue) {
        return prefs.getFloat(KEY_AUTO_FINAL_HEADING, defaultValue);
    }
}

