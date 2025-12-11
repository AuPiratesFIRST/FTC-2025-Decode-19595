package org.firstinspires.ftc.teamcode.SubSystems.Scoring;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SubSystems.Spindexer.OldSpindexerSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Shooter.ShooterSubsystem;

import java.util.ArrayList;
import java.util.List;

/**
 * AutoOuttakeController
 *
 * Non-blocking controller that:
 *  - reads a NormalizedColorSensor
 *  - maintains a rolling detected ramp (up to 9)
 *  - uses PatternScorer to compute motif score
 *  - when score >= threshold: spin shooter, then advance spindexer to outtake
 *
 * Call update() each cycle from your opmode loop. Provide ability to change motif
 * at runtime via setMotif().
 */
public class AutoOuttakeController {

    public enum State { IDLE, WAITING_FOR_SHOOTER, OUTTAKING, COOLDOWN }

    private final NormalizedColorSensor colorSensor;
    private PatternScorer patternScorer;
    private final OldSpindexerSubsystem spindexer;
    private final ShooterSubsystem shooter;
    private final Telemetry telemetry;

    private final List<ArtifactColor> detectedRamp = new ArrayList<>();
    private final int maxRampSize = 9;

    // Configurable
    private int scoreThreshold = 6;
    private double targetShooterRPM = 5200;
    private long shooterTimeoutMs = 4000;
    private long cooldownMs = 1500;

    private State state = State.IDLE;
    private boolean autoEnabled = true;
    private int outtakeCount = 0;
    private int outtakePerformed = 0;
    private long stateStartTime = 0;

    public AutoOuttakeController(NormalizedColorSensor colorSensor,
                                 ArtifactColor[] motif,
                                 OldSpindexerSubsystem spindexer,
                                 ShooterSubsystem shooter,
                                 Telemetry telemetry) {
        this.colorSensor = colorSensor;
        this.spindexer = spindexer;
        this.shooter = shooter;
        this.telemetry = telemetry;
        setMotif(motif);
    }

    // Call every loop
    public void update() {
        // 1) read sensor and update detected ramp
        NormalizedRGBA rgba = colorSensor.getNormalizedColors();
        final float[] hsv = new float[3];
        Color.colorToHSV(rgba.toColor(), hsv);
        ArtifactColor mapped = mapHueToArtifact(hsv, rgba);

        if (mapped != null) {
            if (detectedRamp.isEmpty() || detectedRamp.get(detectedRamp.size() - 1) != mapped) {
                detectedRamp.add(mapped);
                while (detectedRamp.size() > maxRampSize) detectedRamp.remove(0);
            }
        }

        ArtifactColor[] detectedArray = detectedRamp.toArray(new ArtifactColor[0]);
        int score = (patternScorer == null) ? 0 : patternScorer.scorePattern(detectedArray);

        // telemetry
        if (telemetry != null) {
            telemetry.addData("AutoState", state.toString());
            telemetry.addData("DetectedRamp", PatternScorer.patternString(detectedArray));
            telemetry.addData("PatternScore", "%d/%d", score, 9);
            telemetry.addData("Outtake", "%d/%d", outtakePerformed, outtakeCount);
        }

        // 2) state machine
        switch (state) {
            case IDLE:
                if (autoEnabled && patternScorer != null && score >= scoreThreshold && detectedArray.length > 0) {
                    outtakeCount = Math.min(3, detectedArray.length); // tune if needed
                    outtakePerformed = 0;
                    shooter.setTargetRPM(targetShooterRPM);
                    state = State.WAITING_FOR_SHOOTER;
                    stateStartTime = System.currentTimeMillis();
                }
                break;

            case WAITING_FOR_SHOOTER:
                if (shooter.isAtTargetRPM()) {
                    state = State.OUTTAKING;
                    stateStartTime = System.currentTimeMillis();
                    spindexer.setIntakeMode(false); // outtake mode
                } else if (System.currentTimeMillis() - stateStartTime > shooterTimeoutMs) {
                    // timeout - proceed anyway
                    state = State.OUTTAKING;
                    stateStartTime = System.currentTimeMillis();
                    spindexer.setIntakeMode(false);
                    if (telemetry != null) telemetry.addData("AutoOuttake", "Shooter timeout - proceeding");
                }
                break;

            case OUTTAKING:
                // If we've completed required advances
                if (outtakePerformed >= outtakeCount) {
                    state = State.COOLDOWN;
                    stateStartTime = System.currentTimeMillis();
                    spindexer.setIntakeMode(true);
                    shooter.stop();
                    break;
                }

                // Trigger an advance if not moving/settling
                if (!spindexer.isMoving() && !spindexer.isSettling()) {
                    spindexer.goToPosition((spindexer.getIndex() + 1) % 3);
                }

                // Let spindexer update be run; your opmode should call spindexer.update() too.
                spindexer.update();

                // When movement finished count one outtake
                if (!spindexer.isMoving() && !spindexer.isSettling()) {
                    long now = System.currentTimeMillis();
                    if (now - stateStartTime > 100) {
                        outtakePerformed++;
                        stateStartTime = now;
                    }
                }
                break;

            case COOLDOWN:
                if (System.currentTimeMillis() - stateStartTime > cooldownMs) {
                    state = State.IDLE;
                    detectedRamp.clear();
                }
                break;
        }
    }

    // Configuration helpers
    public void setScoreThreshold(int t) { scoreThreshold = t; }
    public void setTargetShooterRPM(double rpm) { targetShooterRPM = rpm; }
    public void setAutoEnabled(boolean enabled) { autoEnabled = enabled; }

    /**
     * Replace motif at runtime; recreates PatternScorer and clears ramp so scoring restarts.
     */
    public void setMotif(ArtifactColor[] motif) {
        if (motif == null) {
            this.patternScorer = null;
            return;
        }
        this.patternScorer = new PatternScorer(motif);
        detectedRamp.clear();
    }

    // Hue->Artifact mapping reused from your SensorColorTelemetry logic
    private ArtifactColor mapHueToArtifact(float[] hsv, NormalizedRGBA colors) {
        float hue = hsv[0];
        float sat = hsv[1];
        float val = hsv[2];

        if (sat < 0.15 || val < 0.002) return null;

        if (hue >= 60f && hue <= 180f) {
            return ArtifactColor.GREEN;
        } else if (hue >= 240f && hue <= 300f) {
            return ArtifactColor.PURPLE;
        } else if (hue >= 0f && hue < 60f) {
            if (colors.blue > colors.red && colors.blue > colors.green * 0.8) {
                return ArtifactColor.PURPLE;
            }
            return null;
        } else {
            if (hue > 300f && colors.blue > colors.red && colors.blue > colors.green * 0.8) {
                return ArtifactColor.PURPLE;
            }
            return null;
        }
    }
}
