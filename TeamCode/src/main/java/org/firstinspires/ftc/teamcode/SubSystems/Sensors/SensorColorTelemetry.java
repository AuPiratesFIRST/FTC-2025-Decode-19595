package org.firstinspires.ftc.teamcode.SubSystems.Sensors;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.SubSystems.Scoring.ArtifactColor;
import org.firstinspires.ftc.teamcode.SubSystems.Scoring.PatternScorer;

import java.util.ArrayList;
import java.util.List;

/**
 * TeleOp that reports normalized color sensor readings (R/G/B), HSV and alpha to telemetry.
 * Additionally maps detected colors to ArtifactColor (GREEN/PURPLE), maintains a short
 * detected-ramp buffer, and uses PatternScorer to score against a motif.
 * Expects a NormalizedColorSensor configured with the name "sensor_color" in the Robot Configuration.
 */
@TeleOp(name = "Sensor: Color Telemetry", group = "Sensor")
public class SensorColorTelemetry extends LinearOpMode {

    private NormalizedColorSensor colorSensor;
    private View relativeLayout;

    // Buffer of detected artifact colors (up to 9 positions)
    private final List<ArtifactColor> detectedRamp = new ArrayList<>();

    // Default motif (can be changed here) - 3-color motif required by PatternScorer
    private final ArtifactColor[] defaultMotif = new ArtifactColor[]{ArtifactColor.GREEN, ArtifactColor.PURPLE, ArtifactColor.GREEN};
    private PatternScorer patternScorer;

    @Override
    public void runOpMode() {
        // Try to find a RelativeLayout on the robot controller app so we can tint the background
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        // Get the color sensor from the hardware map
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");

        // If the sensor supports a controllable light, turn it on for better readings
        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight) colorSensor).enableLight(true);
        }

        // Create PatternScorer with the default motif
        patternScorer = new PatternScorer(defaultMotif);

        waitForStart();

        final float[] hsvValues = new float[3];

        while (opModeIsActive()) {
            // Read normalized RGBA from the sensor
            NormalizedRGBA colors = colorSensor.getNormalizedColors();

            // Convert to Android HSV values
            Color.colorToHSV(colors.toColor(), hsvValues);

            // Report RGB (3 decimal places)
            telemetry.addData("Red", "%.3f", colors.red);
            telemetry.addData("Green", "%.3f", colors.green);
            telemetry.addData("Blue", "%.3f", colors.blue);

            // Report HSV (Hue may be a larger number)
            telemetry.addData("Hue", "%.3f", hsvValues[0]);
            telemetry.addData("Saturation", "%.3f", hsvValues[1]);
            telemetry.addData("Value", "%.3f", hsvValues[2]);

            // Report Alpha
            telemetry.addData("Alpha", "%.3f", colors.alpha);

            // If the sensor supports distance measurements, report them too
            if (colorSensor instanceof DistanceSensor) {
                telemetry.addData("Distance (cm)", "%.3f", ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM));
            }

            // Map HSV -> ArtifactColor (simple hue-based mapping)
            ArtifactColor mapped = mapHueToArtifact(hsvValues, colors);
            telemetry.addData("MappedArtifact", mapped == null ? "UNKNOWN" : mapped.toString());

            // Add to detected ramp buffer when a new color is detected (and confident)
            if (mapped != null) {
                if (detectedRamp.isEmpty() || detectedRamp.get(detectedRamp.size() - 1) != mapped) {
                    detectedRamp.add(mapped);
                    // keep only last 9
                    while (detectedRamp.size() > 9) {
                        detectedRamp.remove(0);
                    }
                }
            }

            // Show detected ramp as string
            ArtifactColor[] detectedArray = detectedRamp.toArray(new ArtifactColor[0]);
            telemetry.addData("DetectedRamp", PatternScorer.patternString(detectedArray));

            // Score against the expected pattern (patternScorer has a full 9-position pattern generated from motif)
            int score = patternScorer.scorePattern(detectedArray);
            telemetry.addData("PatternScore", "%d/%d", score, 9);

            telemetry.update();

            // Optionally tint the robot controller background to match detected color
            if (relativeLayout != null) {
                final float[] hsvCopy = hsvValues.clone();
                relativeLayout.post(new Runnable() {
                    public void run() {
                        relativeLayout.setBackgroundColor(Color.HSVToColor(hsvCopy));
                    }
                });
            }

            // Gamepad controls:
            // - Y clears the detected ramp buffer
            // - A forces adding the current mapped color (if any)
            if (gamepad1.y) {
                detectedRamp.clear();
            }
            if (gamepad1.a && mapped != null) {
                // ensure we don't add duplicates if already the last entry
                if (detectedRamp.isEmpty() || detectedRamp.get(detectedRamp.size() - 1) != mapped) {
                    detectedRamp.add(mapped);
                    while (detectedRamp.size() > 9) detectedRamp.remove(0);
                }
            }

            idle();
        }
    }

    /**
     * Map HSV to ArtifactColor. Uses hue primarily. Returns null if confidence is low.
     */
    private ArtifactColor mapHueToArtifact(float[] hsv, NormalizedRGBA colors) {
        float hue = hsv[0];
        float sat = hsv[1];
        float val = hsv[2];

        // Basic confidence checks: require some saturation and value
        if (sat < 0.15 || val < 0.08) {
            return null; // too dim/unsaturated to decide
        }

        // Hue ranges: approximate
        // Green: ~60-180
        // Purple: ~240-320 (we'll treat anything outside green as purple for this simple mapping)
        if (hue >= 60f && hue <= 180f) {
            return ArtifactColor.GREEN;
        } else {
            return ArtifactColor.PURPLE;
        }
    }
}
