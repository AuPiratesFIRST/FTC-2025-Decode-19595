package org.firstinspires.ftc.teamcode.SubSystems.Scoring;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SubSystems.Sensors.ColorSensorSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Spindexer.OldSpindexerSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Shooter.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Funnel.FunnelSubsystem;

/**
 * AutoOuttakeController - Cam/Funnel Logic Version
 * 
 * Logic Flow:
 * 1. Detect Pattern -> Spin Shooter -> Move Spindexer to Start.
 * 2. Wait for Spindexer & Shooter stability.
 * 3. Extend Cam (Push) -> Wait -> Retract Cam -> Wait.
 * 4. Move Spindexer to next index.
 * 5. Repeat until all balls are shot.
 */
public class AutoOuttakeController {

    public enum State {
        IDLE,               // Waiting for color sensor match
        PREPARING,          // Moving to first slot, spinning up shooter
        READY_TO_SHOOT,     // In position, waiting for stability
        SHOOTING_EXTEND,    // Pushing the ball (Cam Out)
        SHOOTING_RETRACT,   // Resetting the cam (Cam In)
        INDEXING,           // Moving to the next slot
        COOLDOWN            // Done, resetting systems
    }

    // Subsystems
    private final ColorSensorSubsystem colorSensor;
    private final OldSpindexerSubsystem spindexer;
    private final ShooterSubsystem shooter;
    private final FunnelSubsystem funnel;
    private final Telemetry telemetry;

    // Settings
    private int scoreThreshold = 6;
    private double targetShooterRPM = 5220;
    
    // Timing Constants (Adjust these based on mechanical testing!)
    private static final long TIME_TO_EXTEND_MS = 300;   // Time cam stays out to ensure ball fires
    private static final long TIME_TO_RETRACT_MS = 250;  // Time allowed for cam to pull back before indexing
    private static final long SHOOTER_TIMEOUT_MS = 3000; // Give up if shooter never reaches speed
    private static final long INDEX_TIMEOUT_MS = 1500;   // Safety timeout for spindexer rotation

    // State Variables
    private State state = State.IDLE;
    private ArtifactColor[] currentMotif;
    private int shotsFired = 0;
    private int totalShotsToFire = 0;
    
    private final ElapsedTime timer = new ElapsedTime();
    private final ElapsedTime totalStateTimer = new ElapsedTime();

    public AutoOuttakeController(ColorSensorSubsystem colorSensor,
                                 ArtifactColor[] defaultMotif,
                                 OldSpindexerSubsystem spindexer,
                                 ShooterSubsystem shooter,
                                 FunnelSubsystem funnel,
                                 Telemetry telemetry) {
        this.colorSensor = colorSensor;
        this.spindexer = spindexer;
        this.shooter = shooter;
        this.funnel = funnel;
        this.telemetry = telemetry;
        this.currentMotif = defaultMotif;
        
        // Ensure color sensor knows the motif
        this.colorSensor.setMotif(defaultMotif);
    }

    public void update() {
        // 1. Update Sensors
        colorSensor.update();
        int currentScore = colorSensor.getPatternScore();
        ArtifactColor[] detectedRamp = colorSensor.getDetectedRamp();

        // 2. State Machine
        switch (state) {
            case IDLE:
                // Trigger Condition: High Score & Balls Detected
                if (currentScore >= scoreThreshold && detectedRamp.length > 0) {
                    startOuttakeSequence(detectedRamp.length);
                }
                break;

            case PREPARING:
                // Step 1: Lock Intake, Spin Shooter, Move to Start
                spindexer.setIntakeMode(false);
                shooter.setTargetRPM(targetShooterRPM);
                
                // Command move only once
                if (!spindexer.isMoving()) {
                    spindexer.rotateToMotifStartPosition(currentMotif);
                }

                // If shooter takes too long, just go anyway
                boolean timeout = totalStateTimer.milliseconds() > SHOOTER_TIMEOUT_MS;
                
                // Wait until Spindexer arrives AND Shooter is ready
                if ((shooter.isAtTargetRPM() && spindexer.isAtPosition()) || timeout) {
                    state = State.READY_TO_SHOOT;
                    timer.reset();
                }
                break;

            case READY_TO_SHOOT:
                // Brief pause to ensure ball settles into the hole before pushing
                if (timer.milliseconds() > 150) {
                    state = State.SHOOTING_EXTEND;
                    timer.reset();
                }
                break;

            case SHOOTING_EXTEND:
                // Step 2: Push the ball (Cam Out)
                funnel.extend();

                // Wait for the cam to fully extend and ball to launch
                if (timer.milliseconds() > TIME_TO_EXTEND_MS) {
                    state = State.SHOOTING_RETRACT;
                    timer.reset();
                }
                break;

            case SHOOTING_RETRACT:
                // Step 3: Retract the ball (Cam In)
                funnel.retract();

                // Wait for cam to clear the chamber
                if (timer.milliseconds() > TIME_TO_RETRACT_MS) {
                    shotsFired++;
                    
                    if (shotsFired >= totalShotsToFire) {
                        // All done? Go to cooldown
                        state = State.COOLDOWN;
                        timer.reset();
                    } else {
                        // More balls? Go to indexing
                        state = State.INDEXING;
                        timer.reset();
                        
                        // Command the move immediately
                        int nextIndex = (spindexer.getIndex() + 1) % 3;
                        spindexer.goToPositionForCurrentMode(nextIndex);
                    }
                }
                break;

            case INDEXING:
                // Step 4: Rotate Spindexer to next slot
                spindexer.update(); 

                // Wait until we arrive at the new slot
                boolean arrived = spindexer.isAtPosition() && !spindexer.isMoving();
                boolean stuck = timer.milliseconds() > INDEX_TIMEOUT_MS;

                if (arrived || stuck) {
                    state = State.READY_TO_SHOOT; // Loop back to shoot the next ball
                    timer.reset();
                }
                break;

            case COOLDOWN:
                // Step 5: Finish up
                shooter.stop();
                funnel.retract();
                spindexer.setIntakeMode(true); // Return to intake mode
                
                // Wait 1 second before allowing another auto-trigger
                if (timer.milliseconds() > 1000) {
                    colorSensor.clearRamp(); // Clear memory
                    state = State.IDLE;
                }
                break;
        }
        
        // Always update spindexer logic in background (except when we are idle)
        if (state != State.IDLE && state != State.COOLDOWN) {
            spindexer.update();
        }
        
        // Telemetry
        if (telemetry != null) {
            telemetry.addData("Auto State", state);
            telemetry.addData("Shots", shotsFired + "/" + totalShotsToFire);
        }
    }

    private void startOuttakeSequence(int detectedBallCount) {
        // Cap shots at 3 (since Spindexer only holds 3)
        this.totalShotsToFire = Math.min(3, detectedBallCount);
        this.shotsFired = 0;
        
        // Start timers
        this.totalStateTimer.reset();
        this.timer.reset();
        
        this.state = State.PREPARING;
    }

    // --- Configuration Methods ---
    public void updateMotif(ArtifactColor[] motif) {
        this.currentMotif = motif;
        this.colorSensor.setMotif(motif);
    }

    public void setScoreThreshold(int threshold) { this.scoreThreshold = threshold; }
    public void setTargetShooterRPM(double rpm) { this.targetShooterRPM = rpm; }
    public State getState() { return state; }
}