package org.firstinspires.ftc.teamcode.SubSystems.Sensors;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SubSystems.Scoring.ArtifactColor;
import org.firstinspires.ftc.teamcode.SubSystems.Scoring.PatternScorer;

import java.util.ArrayList;
import java.util.List;

/**
 * ColorSensorSubsystem
 *
 * Handles a normalized color sensor, maintains a ramp of detected artifact colors,
 * maps HSV readings to ArtifactColor, and scores against a motif using PatternScorer.
 */
public class ColorSensorSubsystem {

    private final NormalizedColorSensor colorSensor;
    private final View relativeLayout; // optional for controller background tint
    private final Telemetry telemetry;

    private final List<ArtifactColor> detectedRamp = new ArrayList<>();
    private PatternScorer patternScorer;

    // Track the detection state from the previous update loop
    private ArtifactColor lastLoopDetectedColor = null;

    private static final int MAX_RAMP_SIZE = 9;

    /**
     * Constructor
     * @param colorSensor the normalized color sensor hardware
     * @param relativeLayout optional view to tint background (can be null)
     * @param telemetry telemetry for logging (can be null)
     * @param defaultMotif default motif for pattern scoring
     */
    public ColorSensorSubsystem(NormalizedColorSensor colorSensor, View relativeLayout,
                                Telemetry telemetry, ArtifactColor[] defaultMotif) {
        this.colorSensor = colorSensor;
        this.relativeLayout = relativeLayout;
        this.telemetry = telemetry;

        // Enable light if supported
        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight) colorSensor).enableLight(true);
        }

        setMotif(defaultMotif);
    }

    /**
     * Updates sensor readings, maps to ArtifactColor, maintains ramp buffer,
     * updates telemetry, and optionally tints background.
     */
    public void update() {
        final float[] hsvValues = new float[3];
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsvValues);

        // 1. Get the current instantaneous reading (Can be G, P, or null)
        ArtifactColor currentReading = mapHueToArtifact(hsvValues, colors);

        // 2. EDGE DETECTION LOGIC
        // We only add to the list if we are seeing a color NOW, 
        // AND we were NOT seeing this specific color in the previous loop.
        if (currentReading != null) {
            // Check if this is a "Rising Edge" (New object or color change)
            if (lastLoopDetectedColor != currentReading) {
                detectedRamp.add(currentReading);
                
                // Keep list size in check
                while (detectedRamp.size() > MAX_RAMP_SIZE) {
                    detectedRamp.remove(0);
                }
            }
        }

        // 3. Update the "Previous Loop" memory for the next cycle
        lastLoopDetectedColor = currentReading;

        // Telemetry
        if (telemetry != null) {
            telemetry.addData("Red", "%.3f", colors.red);
            telemetry.addData("Green", "%.3f", colors.green);
            telemetry.addData("Blue", "%.3f", colors.blue);
            telemetry.addData("Hue", "%.3f", hsvValues[0]);
            
            telemetry.addData("Current Sight", currentReading == null ? "---" : currentReading.toString());
            
            ArtifactColor[] rampArray = detectedRamp.toArray(new ArtifactColor[0]);
            telemetry.addData("DetectedRamp", PatternScorer.patternString(rampArray));
            telemetry.addData("PatternScore", "%d/%d",
                    patternScorer == null ? 0 : patternScorer.scorePattern(rampArray), MAX_RAMP_SIZE);
        }

        // Optional background tint
        if (relativeLayout != null) {
            final float[] hsvCopy = hsvValues.clone();
            relativeLayout.post(() -> relativeLayout.setBackgroundColor(Color.HSVToColor(hsvCopy)));
        }
    }

    /** Clears the detected ramp buffer */
    public void clearRamp() {
        detectedRamp.clear();
        lastLoopDetectedColor = null; // Reset memory logic
    }

    /** Returns a copy of the detected ramp */
    public ArtifactColor[] getDetectedRamp() {
        return detectedRamp.toArray(new ArtifactColor[0]);
    }

    /** Set the pattern motif for scoring */
    public void setMotif(ArtifactColor[] motif) {
        if (motif == null) {
            this.patternScorer = null;
        } else {
            this.patternScorer = new PatternScorer(motif);
            detectedRamp.clear();
        }
    }

    /** Score the current detected ramp against the motif */
    public int getPatternScore() {
        if (patternScorer == null) return 0;
        return patternScorer.scorePattern(detectedRamp.toArray(new ArtifactColor[0]));
    }

    /** Map HSV to ArtifactColor, returns null if unsure */
    private ArtifactColor mapHueToArtifact(float[] hsv, NormalizedRGBA colors) {
        float hue = hsv[0], sat = hsv[1], val = hsv[2];

        if (sat < 0.15 || val < 0.002) return null;

        if (hue >= 60f && hue <= 180f) return ArtifactColor.GREEN;
        if (hue >= 240f && hue <= 300f) return ArtifactColor.PURPLE;
        if (hue >= 0f && hue < 60f) {
            if (colors.blue > colors.red && colors.blue > colors.green * 0.8) return ArtifactColor.PURPLE;
        }
        if (hue > 300f && colors.blue > colors.red && colors.blue > colors.green * 0.8) return ArtifactColor.PURPLE;
        return null;
    }

    /** Get the current motif used for scoring */
    public ArtifactColor[] getMotif() {
        return patternScorer == null ? null : patternScorer.getMotif();
    }

    /** Optionally enable or disable the sensor light */
    public void setLightEnabled(boolean enabled) {
        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight) colorSensor).enableLight(enabled);
        }
    }
}
