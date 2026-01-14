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
    private static final int[] INTAKE_POSITIONS = { 10, 717, 1434 };
    private static final int[] OUTTAKE_POSITIONS = { 268, 451, 630 };

    // COMPETITION TUNED COEFFICIENTS - Aggressive "Lockdown" Tuning
    private double kP = 0.02350000000000001;     // High P for immediate reaction
    private double kI = 0.00038000000000000035;   // I builds up the "Active Hold" force
    private double kD =  0.01059999999999999;    // D prevents high-speed shaking

    private double lastError = 0;
    private double integralSum = 0;
    private boolean hasPrevError = false;
    private int targetPosition = 0;

    private static final int POSITION_TOLERANCE = 8; // Tighter tolerance for accuracy
    private static final double SPEED_MULTIPLIER = 0.46; // 0.75 for torque and smoothness

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
     */
    public void update() {
        if (!pidEnabled) return;

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

        spindexerMotor.setPower(Range.clip(output, -1.0, 1.0) * SPEED_MULTIPLIER);
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

    public void lockCurrentPosition() {
        targetPosition = normalizeTicks(spindexerMotor.getCurrentPosition());
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
    public void setIntakeMode(boolean intake) { this.intakeMode = intake; }
    public void setTuningMode(boolean enabled) { this.tuningMode = enabled; }
    public boolean isSettling() { return false; }
    public int getCurrentPosition() { return spindexerMotor.getCurrentPosition(); }
    public int getTargetPosition() { return targetPosition; }
    public void setManualPower(double power) { pidEnabled = false; spindexerMotor.setPower(power); }
    public void rotateToMotifStartPosition(ArtifactColor[] motif) { goToPositionForCurrentMode(0); }
    public static double getRecommendedManualPowerMultiplier() { return 0.75; }
}