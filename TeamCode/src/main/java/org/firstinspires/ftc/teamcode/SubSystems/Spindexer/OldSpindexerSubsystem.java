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
    private static final double SPEED_MULTIPLIER = 0.75; // 0.75 for torque and smoothness

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
     * FORCED ONE-WAY LOGIC (IMPROVED)
     * This ensures the spindexer never backs into the mechanical ramp.
     * ✅ Also avoids forcing full revolution if already at target (prevents deadlock).
     */
    public double forwardOnlyError(int target, int current) {
        int nTarget = normalizeTicks(target);
        int nCurrent = normalizeTicks(current);
        
        double error = (double) nTarget - nCurrent;
        
        // If we're already basically at target, don't force a full lap
        if (Math.abs(error) <= POSITION_TOLERANCE) {
            return 0;
        }
        
        // If error is negative, it means the target is "behind" us.
        // We add a full revolution to force it to go the "long way" around (Clockwise).
        if (error < 0) {
            error += TICKS_PER_REVOLUTION;
        }
        
        return error;
    }

    /**
     * CORE CONTROL LOOP
     * Implements "Active Force Feedback" to resist external pressure.
     * NOW WITH ONE-WAY SAFETY: Uses forwardOnlyError to prevent backward motion into ramp.
     */
    public void update() {
        if (!pidEnabled) return;

        int currentPosition = spindexerMotor.getCurrentPosition();
        
        // ✅ CHANGE: Use forwardOnlyError instead of shortestError
        double error = forwardOnlyError(targetPosition, currentPosition);

        if (!hasPrevError) {
            lastError = error;
            hasPrevError = true;
        }

        integralSum += error;
        integralSum = Range.clip(integralSum, -0.25 / (kI + 1e-9), 0.25 / (kI + 1e-9));

        double derivative = (error - lastError) * kD;
        lastError = error;

        double output = (error * kP) + (integralSum * kI) + derivative;

        // ✅ SAFETY CLIP: Only allow positive (Forward/Clockwise) power
        // This is the "insurance policy" for your ramp.
        // ✅ ADD MINIMUM FEEDFORWARD: Overcome static friction + BRAKE drag
        double minPower = 0.12; // Tune between 0.10–0.18 based on mechanism load
        double commanded = Range.clip(output, 0.0, 1.0);
        
        double finalPower = commanded > 0.001
                ? (minPower + commanded * (1.0 - minPower)) * SPEED_MULTIPLIER
                : 0.0;
        
        spindexerMotor.setPower(finalPower);
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
            resetPIDOnly(); // ✅ Reset accumulated integral on new target
            setPIDEnabled(true);
        }
    }

    public void goToPosition(SpindexerPosition position) {
        targetPosition = normalizeTicks(position.getTicks());
        resetPIDOnly(); // ✅ Reset accumulated integral on new target
        setPIDEnabled(true);
    }

    public void goToPositionForCurrentMode(int index) {
        // NOTE: Avoid index=0 at startup (only 10 ticks). Use index=1 for meaningful motion verification.
        int ticks = intakeMode ? INTAKE_POSITIONS[index] : OUTTAKE_POSITIONS[index];
        targetPosition = normalizeTicks(ticks);
        resetPIDOnly(); // ✅ Reset accumulated integral on new target
        setPIDEnabled(true);
    }

    public void lockCurrentPosition() {
        targetPosition = normalizeTicks(spindexerMotor.getCurrentPosition());
        setPIDEnabled(true);
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
        // ✅ CONSISTENCY FIX: Use forwardOnlyError (same model as PID control)
        // This prevents PID & state machine from disagreeing about position
        double error = forwardOnlyError(targetPosition, spindexerMotor.getCurrentPosition());
        return error <= POSITION_TOLERANCE;
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

    public void setPIDEnabled(boolean enabled) { this.pidEnabled = enabled; }
    public void setIntakeMode(boolean intake) { this.intakeMode = intake; }
    public void setTuningMode(boolean enabled) { this.tuningMode = enabled; }
    public boolean isSettling() { return false; }
    public int getCurrentPosition() { return spindexerMotor.getCurrentPosition(); }
    public int getTargetPosition() { return targetPosition; }
    public void setManualPower(double power) {
        // ✅ TRUE MANUAL MODE: Direct motor control for real hardware verification
        // This bypasses PID entirely for testing and emergency control
        pidEnabled = false;
        spindexerMotor.setPower(Range.clip(power, -0.6, 0.6));
    }

    public void stopManual() {
        // Stop manual control and return to idle
        spindexerMotor.setPower(0);
        pidEnabled = false;
    }
    public void rotateToMotifStartPosition(ArtifactColor[] motif) { goToPositionForCurrentMode(0); }
    public static double getRecommendedManualPowerMultiplier() { return 0.75; }
}