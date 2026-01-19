package org.firstinspires.ftc.teamcode.SubSystems.Scoring;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SubSystems.Sensors.ColorSensorSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Spindexer.OldSpindexerSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Shooter.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Funnel.FunnelSubsystem;
import java.util.Arrays;

/**
 * AutoOuttakeController - Cam/Funnel Logic Version with Slot Tracking
 *
 * Tracks which colors are in which spindexer slots, so that the auto routine
 * can fire balls in the correct motif even if it is run intermittently.
 */
public class AutoOuttakeController {

    public enum State {
        IDLE,
        PREPARING,
        READY_TO_SHOOT,
        SHOOTING_EXTEND,
        SHOOTING_RETRACT,
        INDEXING,
        COOLDOWN
    }

    // --- Subsystems ---
    private final ColorSensorSubsystem colorSensor;
    private final OldSpindexerSubsystem spindexer;
    private final ShooterSubsystem shooter;
    private final FunnelSubsystem funnel;
    private final Telemetry telemetry;

    // --- Settings ---
    private int scoreThreshold = 6;
    private double targetShooterRPM = 5220;

    private static final long TIME_TO_EXTEND_MS = 300;
    private static final long TIME_TO_RETRACT_MS = 250;
    private static final long SHOOTER_TIMEOUT_MS = 3000;
    private static final long INDEX_TIMEOUT_MS = 1500;
    private static final long COOLDOWN_DELAY_MS = 1000;

    // --- State Variables ---
    private State state = State.IDLE;
    private ArtifactColor[] currentMotif;
    private int shotsFired = 0;
    private int totalShotsToFire = 0;

    private final ElapsedTime stateTimer = new ElapsedTime();

    // --- Slot Tracking ---
    private final ArtifactColor[] spindexerSlots = new ArtifactColor[3]; // null = empty

    // --- Constructor ---
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
        this.colorSensor.setMotif(defaultMotif);
    }

    // --- Main Update Loop ---
    public void update() {
        // Update sensors
        colorSensor.update();
        int currentScore = colorSensor.getPatternScore();
        ArtifactColor[] detectedRamp = colorSensor.getDetectedRamp();

        // Register new incoming balls into first empty slots
        for (ArtifactColor color : detectedRamp) {
            registerIncomingBall(color);
        }

        // --- State Machine ---
        switch (state) {
            case IDLE:
                if (currentScore >= scoreThreshold && hasBalls()) {
                    startOuttakeSequence(Math.min(3, getBallCount()));
                }
                break;

            case PREPARING:
                spindexer.setIntakeMode(false);
                shooter.setTargetRPM(targetShooterRPM);

                if (!spindexer.isMoving()) {
                    int startIndex = getNextSlotToShoot();
                    if (startIndex >= 0) {
                        spindexer.goToPositionForCurrentMode(startIndex);
                    }
                }

                boolean shooterReady = shooter.isAtTargetRPM() && spindexer.isAtPosition();
                boolean timeout = stateTimer.milliseconds() > SHOOTER_TIMEOUT_MS;

                if (shooterReady || timeout) {
                    state = State.READY_TO_SHOOT;
                    stateTimer.reset();
                }
                break;

            case READY_TO_SHOOT:
                if (stateTimer.milliseconds() > 150) { // brief settle delay
                    state = State.SHOOTING_EXTEND;
                    stateTimer.reset();
                }
                break;

            case SHOOTING_EXTEND:
                funnel.extend();
                if (stateTimer.milliseconds() > TIME_TO_EXTEND_MS) {
                    state = State.SHOOTING_RETRACT;
                    stateTimer.reset();
                }
                break;

            case SHOOTING_RETRACT:
                funnel.retract();
                if (stateTimer.milliseconds() > TIME_TO_RETRACT_MS) {
                    int currentSlot = spindexer.getIndex();
                    clearSlot(currentSlot); // remove ball from slot
                    shotsFired++;

                    if (shotsFired >= totalShotsToFire) {
                        state = State.COOLDOWN;
                        stateTimer.reset();
                    } else {
                        state = State.INDEXING;
                        stateTimer.reset();
                        int nextIndex = getNextSlotToShoot();
                        if (nextIndex >= 0) {
                            spindexer.goToPositionForCurrentMode(nextIndex);
                        }
                    }
                }
                break;

            case INDEXING:
                spindexer.update();

                boolean arrived = spindexer.isAtPosition() && !spindexer.isMoving();
                boolean stuck = stateTimer.milliseconds() > INDEX_TIMEOUT_MS;

                if (arrived || stuck) {
                    if (stuck && telemetry != null) {
                        telemetry.addData("WARNING", "Spindexer stuck, skipping to next");
                    }
                    state = State.READY_TO_SHOOT;
                    stateTimer.reset();
                }
                break;

            case COOLDOWN:
                shooter.stop();
                if (funnel.isExtended()) {
                    funnel.retract();
                }
                spindexer.setIntakeMode(true);

                if (stateTimer.milliseconds() > COOLDOWN_DELAY_MS) {
                    state = State.IDLE;
                    stateTimer.reset();
                }
                break;
        }

        // Always update spindexer unless idle
        if (state != State.IDLE && state != State.COOLDOWN) {
            spindexer.update();
        }

        // Telemetry
        if (telemetry != null) {
            telemetry.addData("Auto State", state);
            telemetry.addData("Shots Fired", shotsFired + "/" + totalShotsToFire);
            telemetry.addData("Spindexer Slots", Arrays.toString(spindexerSlots));
        }
    }

    // --- Helper: Register ball in first empty slot ---
    private void registerIncomingBall(ArtifactColor color) {
        for (int i = 0; i < spindexerSlots.length; i++) {
            if (spindexerSlots[i] == null) {
                spindexerSlots[i] = color;
                break;
            }
        }
    }

    // --- Helper: Clear a slot after shooting ---
    private void clearSlot(int index) {
        if (index >= 0 && index < spindexerSlots.length) {
            spindexerSlots[index] = null;
        }
    }

    // --- Helper: Check if there are any balls ---
    private boolean hasBalls() {
        for (ArtifactColor color : spindexerSlots) {
            if (color != null) return true;
        }
        return false;
    }

    // --- Helper: Count balls in spindexer ---
    private int getBallCount() {
        int count = 0;
        for (ArtifactColor color : spindexerSlots) {
            if (color != null) count++;
        }
        return count;
    }

    // --- Helper: Find next slot to shoot based on motif ---
    private int getNextSlotToShoot() {
        // Try to match current motif order
        for (int i = 0; i < spindexerSlots.length; i++) {
            ArtifactColor slotColor = spindexerSlots[i];
            if (slotColor != null && slotColor == currentMotif[shotsFired]) {
                return i;
            }
        }
        // Fallback: first non-empty slot
        for (int i = 0; i < spindexerSlots.length; i++) {
            if (spindexerSlots[i] != null) return i;
        }
        return -1; // nothing left
    }

    // --- Start Auto Sequence ---
    private void startOuttakeSequence(int detectedBallCount) {
        this.totalShotsToFire = Math.min(3, detectedBallCount);
        this.shotsFired = 0;
        this.stateTimer.reset();
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

    // --- Manual Abort ---
    public void abort() {
        state = State.IDLE;
        shooter.stop();
        funnel.retract();
        spindexer.setIntakeMode(true);
        Arrays.fill(spindexerSlots, null);
        stateTimer.reset();
    }
}
