package org.firstinspires.ftc.teamcode.SubSystems.Spindexer;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SubSystems.Scoring.ArtifactColor;

@Configurable
public class OldSpindexerSubsystem {

    private final DcMotorEx spindexerMotor;
    private final Telemetry telemetry;

    private static final double TICKS_PER_REVOLUTION = 2150.8;

    // Position definitions
    public static final int[] INTAKE_POSITIONS = { 10, 717, 1434 };
    public static final int[] OUTTAKE_POSITIONS = { 268, 451, 630 };

    // COMPETITION TUNED COEFFICIENTS - Aggressive "Lockdown" Tuning
    // Base coefficients for empty (intake) mode
    private static final double kP_EMPTY = 0.02350000000000001;     // High P for immediate reaction
    private static final double kI_EMPTY = 0.00038000000000000035;   // I builds up the "Active Hold" force
    private static final double kD_EMPTY = 0.01059999999999999;    // D prevents high-speed shaking
    
    // Enhanced coefficients for loaded (outtake) mode - prevents sag under load
    private static final double kP_LOADED = 0.035;   // ~1.5× boost for faster response under load
    private static final double kI_LOADED = 0.0005;  // Slightly higher integral for steady hold
    private static final double kD_LOADED = 0.012;   // Increased damping to prevent overshoot
    
    // Active coefficients (dynamically selected based on mode)
    private double kP = kP_EMPTY;
    private double kI = kI_EMPTY;
    private double kD = kD_EMPTY;

    private double lastError = 0;
    private double integralSum = 0;
    private boolean hasPrevError = false;
    private int targetPosition = 0;

    private static final int POSITION_TOLERANCE = 8; // Tighter tolerance for accuracy
    private static final double SPEED_MULTIPLIER = 0.4; // Increased from 0.35 for better response
    private static final double GRAVITY_FEEDFORWARD = 0.08; // 8% power to counter gravity in vertical spindexer

    private boolean pidEnabled = false;
    private boolean tuningMode = false;
    private boolean intakeMode = true;

    public enum SpindexerPosition {
        POSITION_1(10), POSITION_2(717), POSITION_3(1434);
        private final int ticks;
        SpindexerPosition(int ticks) { this.ticks = ticks; }
        public int getTicks() { return ticks; }
    }

    public OldSpindexerSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        spindexerMotor = hardwareMap.get(DcMotorEx.class, "Spindexer");

        spindexerMotor.setDirection(DcMotor.Direction.REVERSE);
        spindexerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexerMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // CRITICAL: Brake prevents physical movement when motor is at 0 power
        spindexerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * CORE CONTROL LOOP
     * Implements "Active Force Feedback" to resist external pressure.
     * Uses dynamic PID coefficients based on load state (intake vs outtake mode).
     */
    public void update() {
        if (!pidEnabled) return;

        // Dynamically adjust PID coefficients based on mode (loaded vs empty)
        if (intakeMode) {
            kP = kP_EMPTY;
            kI = kI_EMPTY;
            kD = kD_EMPTY;
        } else {
            // Outtake mode = loaded with balls, use higher gains to prevent sag
            kP = kP_LOADED;
            kI = kI_LOADED;
            kD = kD_LOADED;
        }

        int currentPosition = spindexerMotor.getCurrentPosition();
        double error = shortestError(targetPosition, currentPosition);

        // LOCKDOWN: We no longer stop at the deadband.
        // The motor stays active to "Pre-load" the gears and fight back.

        if (!hasPrevError) {
            lastError = error;
            hasPrevError = true;
        }

        // Build Active Hold Force
        integralSum += error;

        // Anti-windup: limit hold force to 40% motor power to prevent over-stalling
        integralSum = Range.clip(integralSum, -0.4 / (kI + 1e-9), 0.4 / (kI + 1e-9));

        double derivative = (error - lastError) * kD;
        lastError = error;

        double output = (error * kP) + (integralSum * kI) + derivative;

        // FEEDFORWARD NUDGE: Overcomes gear stiction/friction
        // If we are slightly off, apply a 5% nudge to force it to the exact tick.
        if (Math.abs(error) > 1 && Math.abs(output) < 0.05) {
            output = Math.signum(error) * 0.05;
        }

        // GRAVITY FEEDFORWARD: For vertical spindexer, add constant force to counter gravity
        // This helps prevent sag when loaded (outtake mode) without waiting for PID to react
        double feedforward = 0;
        if (!intakeMode) {
            // Outtake mode = loaded, apply feedforward to hold against gravity
            feedforward = GRAVITY_FEEDFORWARD;
            // If moving downward (target below current), assist the movement
            if (error < 0) {
                feedforward = -feedforward;
            }
        }

        // Combine PID output with feedforward
        double finalOutput = output + feedforward;
        spindexerMotor.setPower(Range.clip(finalOutput, -1.0, 1.0) * SPEED_MULTIPLIER);
    }

    // === COMPATIBILITY METHODS (Required for your existing code) ===

    public int getIndex() {
        double d1 = Math.abs(shortestError(INTAKE_POSITIONS[0], targetPosition));
        double d2 = Math.abs(shortestError(INTAKE_POSITIONS[1], targetPosition));
        double d3 = Math.abs(shortestError(INTAKE_POSITIONS[2], targetPosition));
        if (d1 <= d2 && d1 <= d3) return 0;
        if (d2 <= d1 && d2 <= d3) return 1;
        return 2;
    }

    public double[] getPIDCoefficients() { return new double[]{kP, kI, kD}; }

    public void setPIDCoefficients(double p, double d) { this.kP = p; this.kD = d; }

    public void setPIDCoefficients(double p, double i, double d) { this.kP = p; this.kI = i; this.kD = d; }

    public void updateTelemetry() {
        if (telemetry == null) return;
        telemetry.addData("Spindexer Target", targetPosition);
        telemetry.addData("Spindexer Current", spindexerMotor.getCurrentPosition());
        telemetry.addData("Hold Power", "%.2f", spindexerMotor.getPower());
    }

    public void goToPosition(int index) {
        if (index >= 0 && index <= 2) goToPositionForCurrentMode(index);
        else {
            targetPosition = normalizeTicks(index);
            setPIDEnabled(true);
        }
    }

    public void goToPosition(SpindexerPosition position) {
        targetPosition = normalizeTicks(position.getTicks());
        setPIDEnabled(true);
    }

    public void goToPositionForCurrentMode(int index) {
        int ticks = intakeMode ? INTAKE_POSITIONS[index] : OUTTAKE_POSITIONS[index];
        targetPosition = normalizeTicks(ticks);
        setPIDEnabled(true);
        resetPIDOnly();
    }

    /**
     * Switch to outtake mode using forward-only movement to prevent jamming.
     * Call this when switching from intake → outtake to ensure balls don't get pushed backward.
     * 
     * @param index The position index (0-2) to move to in outtake mode
     */
    public void switchToOuttakeModeForwardOnly(int index) {
        if (index < 0 || index > 2) return;
        intakeMode = false;
        int targetTicks = OUTTAKE_POSITIONS[index];
        goToPositionForwardOnly(targetTicks);
    }

    public void lockCurrentPosition() {
        targetPosition = normalizeTicks(spindexerMotor.getCurrentPosition());
        setPIDEnabled(true);
        resetPIDOnly();
    }

    /**
     * Move to target position using FORWARD-ONLY (clockwise) movement.
     * Ensures shortest forward path without going backward.
     */
    public void goToPositionForwardOnly(int ticksTarget) {
        int current = normalizeTicks(spindexerMotor.getCurrentPosition());
        int normalized = normalizeTicks(ticksTarget);
        
        // Calculate forward distance
        int forwardDistance = (normalized - current + (int) TICKS_PER_REVOLUTION) % (int) TICKS_PER_REVOLUTION;
        
        // Set target to current + forward distance to ensure forward movement
        targetPosition = current + forwardDistance;
        setPIDEnabled(true);
        resetPIDOnly();
    }

    /**
     * Move to target position using BACKWARD-ONLY (counter-clockwise) movement.
     * Ensures movement in reverse direction.
     */
    public void goToPositionBackwardOnly(int ticksTarget) {
        int current = normalizeTicks(spindexerMotor.getCurrentPosition());
        int normalized = normalizeTicks(ticksTarget);
        
        // Calculate backward distance
        int backwardDistance = (current - normalized + (int) TICKS_PER_REVOLUTION) % (int) TICKS_PER_REVOLUTION;
        
        // If backward distance is 0, go full revolution backward
        if (backwardDistance == 0) {
            backwardDistance = (int) TICKS_PER_REVOLUTION;
        }
        
        // Set target to current - backward distance to ensure backward movement
        targetPosition = current - backwardDistance;
        setPIDEnabled(true);
        resetPIDOnly();
    }

    // === UTILITY METHODS ===

    public void reset() {
        spindexerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexerMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        targetPosition = 0;
        resetPIDOnly();
        pidEnabled = false;
    }

    public void resetPIDOnly() {
        lastError = 0;
        integralSum = 0;
        hasPrevError = false;
    }

    public boolean isAtPosition() {
        return Math.abs(shortestError(targetPosition, spindexerMotor.getCurrentPosition())) <= POSITION_TOLERANCE;
    }

    public boolean isMoving() {
        // Report moving if we are active and significantly away from target
        return pidEnabled && Math.abs(shortestError(targetPosition, spindexerMotor.getCurrentPosition())) > POSITION_TOLERANCE;
    }

    public double shortestError(int target, int current) {
        int nTarget = normalizeTicks(target);
        int nCurrent = normalizeTicks(current);
        double raw = (double) nTarget - nCurrent;
        if (raw > TICKS_PER_REVOLUTION / 2.0) raw -= TICKS_PER_REVOLUTION;
        if (raw < -TICKS_PER_REVOLUTION / 2.0) raw += TICKS_PER_REVOLUTION;
        return raw;
    }

    public int normalizeTicks(int ticks) {
        int normalized = ticks % (int) TICKS_PER_REVOLUTION;
        if (normalized < 0) normalized += (int) TICKS_PER_REVOLUTION;
        return normalized;
    }

        public void stopManual() {
        // Stop manual control and return to idle
        spindexerMotor.setPower(0);
        pidEnabled = false;
    }

    public void setPIDEnabled(boolean enabled) { this.pidEnabled = enabled; }
    
    /**
     * Set intake mode. When switching modes, PID is reset to prevent windup from previous mode.
     * 
     * @param intake true for intake mode (empty), false for outtake mode (loaded)
     */
    public void setIntakeMode(boolean intake) { 
        boolean modeChanged = this.intakeMode != intake;
        this.intakeMode = intake;
        // Reset PID when switching modes to prevent windup from previous mode's error
        if (modeChanged) {
            resetPIDOnly();
        }
    }
    
    public void setTuningMode(boolean enabled) { this.tuningMode = enabled; }
    public boolean isSettling() { return false; }
    public int getCurrentPosition() { return spindexerMotor.getCurrentPosition(); }
    public int getTargetPosition() { return targetPosition; }
    public void setManualPower(double power) { pidEnabled = false; spindexerMotor.setPower(power); }
    public void rotateToMotifStartPosition(ArtifactColor[] motif) { goToPositionForCurrentMode(0); }
    public static double getRecommendedManualPowerMultiplier() { return 0.75; }
}