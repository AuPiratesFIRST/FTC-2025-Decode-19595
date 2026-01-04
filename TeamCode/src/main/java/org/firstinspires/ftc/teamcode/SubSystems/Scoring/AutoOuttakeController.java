package org.firstinspires.ftc.teamcode.SubSystems.Scoring;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SubSystems.Sensors.ColorSensorSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Spindexer.OldSpindexerSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Shooter.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Funnel.FunnelSubsystem;

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
    private final FunnelSubsystem funnel;
    private final Telemetry telemetry;

    // Configurable
    private int scoreThreshold = 6;
    private double targetShooterRPM = 5200;
    private long shooterTimeoutMs = 4000;
    private long cooldownMs = 1500;
    private long funnelPushDurationMs = 150;  // Time to hold funnel extended (cam flip time)
    private long jamDetectionTimeoutMs = 500; // Time before considering funnel jammed
    
    // Jam detection state
    private long funnelExtendStartTime = 0;
    private boolean jamRecoveryInProgress = false;

    private State state = State.IDLE;
    private boolean autoEnabled = true;
    private int outtakeCount = 0;
    private int outtakePerformed = 0;
    private long stateStartTime = 0;

    public AutoOuttakeController(ColorSensorSubsystem colorSensorSubsystem,
                                 ArtifactColor[] motif,
                                 OldSpindexerSubsystem spindexer,
                                 ShooterSubsystem shooter,
                                 FunnelSubsystem funnel,
                                 Telemetry telemetry) {
        this.colorSensorSubsystem = colorSensorSubsystem;
        this.spindexer = spindexer;
        this.shooter = shooter;
        this.funnel = funnel;
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
                    // Rotate to starting position based on motif (position 0 for first color)
                    spindexer.setIntakeMode(false); // Switch to outtake mode
                    spindexer.rotateToMotifStartPosition(colorSensorSubsystem.getMotif());
                    state = State.OUTTAKING;
                    stateStartTime = System.currentTimeMillis();
                } else if (System.currentTimeMillis() - stateStartTime > shooterTimeoutMs) {
                    // Timeout - proceed anyway
                    spindexer.setIntakeMode(false);
                    spindexer.rotateToMotifStartPosition(colorSensorSubsystem.getMotif());
                    state = State.OUTTAKING;
                    stateStartTime = System.currentTimeMillis();
                    if (telemetry != null) telemetry.addData("AutoOuttake", "Shooter timeout - proceeding");
                }
                break;

            case OUTTAKING:
                // CRITICAL SAFETY: Stop shooter when spindexer is moving
                if (spindexer.isMoving() || !spindexer.isAtPosition()) {
                    shooter.stop();
                    funnel.retract(); // Keep funnel clear while spindexer moves
                    jamRecoveryInProgress = false; // Reset jam detection during movement
                }
                
                if (outtakePerformed >= outtakeCount) {
                    state = State.COOLDOWN;
                    stateStartTime = System.currentTimeMillis();
                    spindexer.setIntakeMode(true);
                    shooter.stop();
                    funnel.retract(); // Clear for next cycle
                    break;
                }

                // Wait for spindexer to reach position before firing
                // CRITICAL: Shooter must be stopped while spindexer is moving
                if (spindexer.isMoving() || !spindexer.isAtPosition()) {
                    shooter.stop(); // Keep shooter stopped during movement
                    funnel.retract(); // Keep funnel clear
                } else if (!spindexer.isSettling() && spindexer.isAtPosition()) {
                    long timeSinceLastMove = System.currentTimeMillis() - stateStartTime;
                    
                    // Check for jam: funnel extended too long without ball clearing
                    if (funnel.isExtended() && (timeSinceLastMove - funnelExtendStartTime) > jamDetectionTimeoutMs) {
                        if (telemetry != null) telemetry.addData("AutoOuttake", "JAM DETECTED - Attempting recovery");
                        jamRecoveryInProgress = true;
                        funnel.retract();
                        // Jiggle spindexer to clear jam
                        if ((timeSinceLastMove - funnelExtendStartTime) > (jamDetectionTimeoutMs + 200)) {
                            spindexer.goToPositionForCurrentMode((spindexer.getIndex() + 1) % 3);
                            jamRecoveryInProgress = false;
                        }
                    }
                    
                    // Normal firing sequence: wait, extend funnel, wait, retract, move to next
                    if (timeSinceLastMove < 200) {
                        // Wait 200ms for spindexer to fully settle
                        shooter.setTargetRPM(targetShooterRPM);
                    } else if (timeSinceLastMove < 200 + funnelPushDurationMs && !jamRecoveryInProgress) {
                        // Extend funnel to push ball
                        if (!funnel.isExtended()) {
                            funnel.extend();
                            funnelExtendStartTime = System.currentTimeMillis();
                        }
                    } else if (timeSinceLastMove >= (200 + funnelPushDurationMs) && !jamRecoveryInProgress) {
                        // Retract funnel and prepare for next position
                        funnel.retract();
                        
                        // Move to next position
                        if (timeSinceLastMove >= (200 + funnelPushDurationMs + 100)) {
                            shooter.stop();
                            int nextIndex = (spindexer.getIndex() + 1) % 3;
                            spindexer.goToPositionForCurrentMode(nextIndex);
                            outtakePerformed++;
                            stateStartTime = System.currentTimeMillis();
                        }
                    }
                }

                spindexer.update();
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
    
    // State accessor
    public State getState() { return state; }

    /** Replace motif at runtime */
    public void setMotif(ArtifactColor[] motif) {
        colorSensorSubsystem.setMotif(motif);
    }
}
