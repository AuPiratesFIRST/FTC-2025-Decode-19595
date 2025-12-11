package org.firstinspires.ftc.teamcode.SubSystems.Scoring;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SubSystems.Sensors.ColorSensorSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Spindexer.OldSpindexerSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Shooter.ShooterSubsystem;

import java.util.ArrayList;
import java.util.List;

/**
 * AutoOuttakeController
 *
 * Non-blocking controller that:
 *  - reads a ColorSensorSubsystem
 *  - maintains a rolling detected ramp (up to 9)
 *  - uses PatternScorer to compute motif score
 *  - when score >= threshold: spin shooter, then advance spindexer to outtake
 *
 * Call update() each cycle from your opmode loop. Provide ability to change motif
 * at runtime via setMotif().
 */
public class AutoOuttakeController {

    public enum State { IDLE, WAITING_FOR_SHOOTER, OUTTAKING, COOLDOWN }

    private final ColorSensorSubsystem colorSensorSubsystem;
    private final OldSpindexerSubsystem spindexer;
    private final ShooterSubsystem shooter;
    private final Telemetry telemetry;

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

    public AutoOuttakeController(ColorSensorSubsystem colorSensorSubsystem,
                                 ArtifactColor[] motif,
                                 OldSpindexerSubsystem spindexer,
                                 ShooterSubsystem shooter,
                                 Telemetry telemetry) {
        this.colorSensorSubsystem = colorSensorSubsystem;
        this.spindexer = spindexer;
        this.shooter = shooter;
        this.telemetry = telemetry;
        setMotif(motif);
    }

    // Call every loop
    public void update() {
        // 1) update sensor subsystem
        colorSensorSubsystem.update();

        ArtifactColor[] detectedArray = colorSensorSubsystem.getDetectedRamp();
        int score = colorSensorSubsystem.getPatternScore();

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
                if (autoEnabled && score >= scoreThreshold && detectedArray.length > 0) {
                    outtakeCount = Math.min(3, detectedArray.length);
                    outtakePerformed = 0;
                    shooter.setTargetRPM(targetShooterRPM);
                    state = State.WAITING_FOR_SHOOTER;
                    stateStartTime = System.currentTimeMillis();
                }
                break;

            case WAITING_FOR_SHOOTER:
                if (shooter.isAtTargetRPM()) {
                    spindexer.goToPosition(1); // Move spindexer to second intake position
                    if (spindexer.isAtPosition()) {
                        state = State.OUTTAKING;
                        stateStartTime = System.currentTimeMillis();
                        spindexer.setIntakeMode(false);
                    }
                } else if (System.currentTimeMillis() - stateStartTime > shooterTimeoutMs) {
                    // Timeout - proceed anyway
                    state = State.OUTTAKING;
                    stateStartTime = System.currentTimeMillis();
                    spindexer.setIntakeMode(false);
                    if (telemetry != null) telemetry.addData("AutoOuttake", "Shooter timeout - proceeding");
                }
                break;

            case OUTTAKING:
                if (outtakePerformed >= outtakeCount) {
                    state = State.COOLDOWN;
                    stateStartTime = System.currentTimeMillis();
                    spindexer.setIntakeMode(true);
                    shooter.stop();
                    break;
                }

                if (!spindexer.isMoving() && !spindexer.isSettling()) {
                    spindexer.goToPosition((spindexer.getIndex() + 1) % 3);
                }

                spindexer.update();

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
                    colorSensorSubsystem.clearRamp();
                }
                break;
        }
    }

    // Configuration helpers
    public void setScoreThreshold(int t) { scoreThreshold = t; }
    public void setTargetShooterRPM(double rpm) { targetShooterRPM = rpm; }
    public void setAutoEnabled(boolean enabled) { autoEnabled = enabled; }

    /** Replace motif at runtime */
    public void setMotif(ArtifactColor[] motif) {
        colorSensorSubsystem.setMotif(motif);
    }
}
