package org.firstinspires.ftc.teamcode.SubSystems.Spindexer;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Vertical spindexer subsystem with PID control for precise position control.
 * Uses GoBILDA 312 RPM motor with three positions for three active arts.
 */
public class SpindexerSubsystem {

    private final DcMotorEx spindexerMotor;
    private final Telemetry telemetry;

    // Motor specifications
    private static final double MOTOR_MAX_RPM = 312.0; // GoBILDA 12 RPM motor
    private static final double TICKS_PER_REVOLUTION = 2150.8; // Standard GoBILDA encoder ticks
    private static final double DEGREES_PER_TICK = 360.0 / TICKS_PER_REVOLUTION;

    // Three positions for three active arts (in encoder ticks)
    // These values should be calibrated based on your physical setup
    // Position calculations: evenly spaced at 120° intervals (360° / 3 = 120°)
    private static final int POSITION_1_TICKS = 0; // First art position (0°)
    private static final int POSITION_2_TICKS = (int) (TICKS_PER_REVOLUTION / 3.0); // 120 degrees (560/3 ≈ 187 ticks)
    private static final int POSITION_3_TICKS = (int) (TICKS_PER_REVOLUTION * 2.0 / 3.0); // 240 degrees (560×2/3 ≈ 373
                                                                                          // ticks)

    // PID coefficients - tune these for your specific setup
    // Start with these values and tune using the systematic process:
    // 1. Tune kP first (increase until overshoot occurs)
    // 2. Tune kD to dampen oscillation (increase until smooth settling)
    // 3. Tune kI last (small values, 0.0001-0.005, to eliminate steady-state error)
    // Recommended starting ranges: kP: 0.005-0.05, kD: 0.01-0.2, kI: 0.0001-0.005
    private double kP = 0.0010; // Start here for P-term tuning
    private double kI = 0.0; // Start at 0, add later for I-term tuning
    private double kD = 0.02; // Start here for D-term tuning after P is set

    // PID state variables
    private double integral = 0;
    private double lastError = 0;
    private int targetPosition = 0;

    // Position tolerance (in ticks)
    private static final int POSITION_TOLERANCE = 15; // Increased tolerance for better stopping

    // Speed multiplier for testing (0.25 = quarter speed)
    private static final double SPEED_MULTIPLIER = 0.1;

    // Minimum power threshold - motor won't move below this (prevents jitter)
    private static final double MIN_POWER_THRESHOLD = 0.05;

    public enum SpindexerPosition {
        POSITION_1(POSITION_1_TICKS),
        POSITION_2(POSITION_2_TICKS),
        POSITION_3(POSITION_3_TICKS);

        private final int ticks;

        SpindexerPosition(int ticks) {
            this.ticks = ticks;
        }

        public int getTicks() {
            return ticks;
        }
    }

    public SpindexerSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        // Initialize motor
        spindexerMotor = hardwareMap.get(DcMotorEx.class, "Spindexer");

        configureMotor();
    }

    private void configureMotor() {
        // Set motor mode for position control
        spindexerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexerMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set zero power behavior
        spindexerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset PID state
        integral = 0;
        lastError = 0;
        targetPosition = 0;
    }

    /**
     * Move to a specific position
     * 
     * @param position SpindexerPosition enum value
     */
    public void goToPosition(SpindexerPosition position) {
        targetPosition = normalizeTicks(position.getTicks());
        integral = 0; // Reset integral when changing targets
        lastError = 0;
    }

    /**
     * Move to a specific position by index (0, 1, or 2)
     * 
     * @param index Position index (0, 1, or 2)
     */
    public void goToPosition(int index) {
        switch (index) {
            case 0:
                goToPosition(SpindexerPosition.POSITION_1);
                break;
            case 1:
                goToPosition(SpindexerPosition.POSITION_2);
                break;
            case 2:
                goToPosition(SpindexerPosition.POSITION_3);
                break;
            default:
                if (telemetry != null) {
                    telemetry.addData("Spindexer Error", "Invalid position index: " + index);
                }
        }
    }

    /**
     * Update PID control - call this in your main loop.
     * 
     * Implements PID (Proportional-Integral-Derivative) control algorithm:
     * - P term: error × kP (proportional to current error, provides initial response)
     * - I term: Σ(error) × kI (eliminates steady-state error, with anti-windup)
     * - D term: (error - lastError) × kD (damping term, reduces overshoot and oscillation)
     * 
     * Output equation: power = (error × kP) + (integral × kI) + (derivative × kD)
     * When error is decreasing (approaching target), derivative is negative, reducing power.
     * This provides smooth deceleration and prevents overshoot.
     * 
     * Tuning Process:
     * 1. Start with kI=0, kD=0. Increase kP until system overshoots/oscillates
     * 2. Keep kP from step 1. Increase kD until oscillation is dampened and system settles smoothly
     * 3. Keep kP and kD from step 2. Add small kI (0.0001-0.005) to eliminate steady-state error
     * 
     * Anti-windup: limits integral accumulation to ±500 to prevent excessive correction.
     */
    public void update() {
        int currentPosition = normalizeTicks(spindexerMotor.getCurrentPosition());
        double error = shortestError(targetPosition, currentPosition);

        // Check if at position - stop motor if within tolerance
        if (isAtPosition()) {
            spindexerMotor.setPower(0);
            // Reset integral when at position to prevent windup
            integral = 0;
            lastError = 0;

            if (telemetry != null) {
                telemetry.addData("Spindexer Status", "AT POSITION - STOPPED");
                telemetry.addData("Spindexer Target", targetPosition);
                telemetry.addData("Spindexer Current", currentPosition);
                telemetry.addData("Spindexer Error", "%.1f", error);
                telemetry.addData("Spindexer Power", "0.00 (stopped)");
            }
            return;
        }

        // Proportional term: immediate response to error
        // P = error × kP (larger error = larger correction)
        double proportional = error * kP;

        // Integral term: accumulates error over time to eliminate steady-state error
        // I = Σ(error) × kI (handles persistent small errors)
        // Only accumulate integral if error is significant (prevents windup near
        // target)
        if (Math.abs(error) > POSITION_TOLERANCE) {
            integral += error;
        } else {
            // Reset integral when close to target to prevent overshoot
            integral = 0;
        }
        // Anti-windup: limit integral to prevent excessive correction
        if (integral > 500) {
            integral = 500;
        } else if (integral < -500) {
            integral = -500;
        }
        double integralTerm = integral * kI;

        // Derivative term: rate of change of error (damping)
        // D = (error - lastError) × kD
        // When error is decreasing (approaching target), this is negative, which should reduce power
        double derivative = (error - lastError) * kD;
        lastError = error;

        // PID output equation: output = P + I + D
        // D is added because when error decreases, derivative is negative, reducing power (damping)
        // This provides smooth deceleration as the system approaches the target
        double output = proportional + integralTerm + derivative;

        // Clip output to valid motor power range [-1.0, 1.0]
        output = Range.clip(output, -1.0, 0.5);

        // Apply speed multiplier for testing (quarter speed)
        output *= SPEED_MULTIPLIER;

        // Apply minimum power threshold - if output is too small, set to zero
        // This prevents jitter and continuous small movements
        if (Math.abs(output) < MIN_POWER_THRESHOLD) {
            output = 0;
        }

        // Apply power to motor
        spindexerMotor.setPower(output);

        if (telemetry != null) {
            telemetry.addData("Spindexer Target", targetPosition);
            telemetry.addData("Spindexer Current", currentPosition);
            telemetry.addData("Spindexer Error", "%.1f", error);
            telemetry.addData("Spindexer Power", "%.2f", output);
            telemetry.addData("At Position", isAtPosition());
            telemetry.addData("--- PID Tuning Guide ---", "");
            telemetry.addData("P Term", "%.3f (kP=%.3f)", proportional, kP);
            telemetry.addData("I Term", "%.3f (kI=%.3f)", integralTerm, kI);
            telemetry.addData("D Term", "%.3f (kD=%.3f)", derivative, kD);
            telemetry.addData("", "");
            telemetry.addData("PID Tuning Instructions:", "");
            telemetry.addData("Step 1 - Tune P:", "Set kI=0, kD=0. Increase kP until overshoot occurs");
            telemetry.addData("Step 2 - Tune D:", "Keep kP from Step 1. Increase kD until smooth settling");
            telemetry.addData("Step 3 - Tune I:", "Keep kP, kD. Add small kI (0.0001-0.005) if needed");
            telemetry.addData("Recommended Ranges:", "kP: 0.005-0.05, kD: 0.01-0.2, kI: 0.0001-0.005");
            telemetry.addData("Current Speed:", "%.0f%% (testing mode)", SPEED_MULTIPLIER * 100);
        }
    }

    /**
     * Check if spindexer is at target position
     * 
     * @return True if within tolerance
     */
    public boolean isAtPosition() {
        int currentPosition = normalizeTicks(spindexerMotor.getCurrentPosition());
        return Math.abs(shortestError(targetPosition, currentPosition)) <= POSITION_TOLERANCE;
    }

    /**
     * Get current position in ticks
     * 
     * @return Current encoder position
     */
    public int getCurrentPosition() {
        return normalizeTicks(spindexerMotor.getCurrentPosition());
    }

    /**
     * Get target position in ticks
     * 
     * @return Target encoder position
     */
    public int getTargetPosition() {
        return targetPosition;
    }

    /**
     * Set PID coefficients
     * 
     * @param kP Proportional gain
     * @param kI Integral gain
     * @param kD Derivative gain
     */
    public void setPIDCoefficients(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    /**
     * Get current PID coefficients
     * 
     * @return Array of [kP, kI, kD]
     */
    public double[] getPIDCoefficients() {
        return new double[] { kP, kI, kD };
    }

    /**
     * Set position tolerance
     * 
     * @param tolerance Tolerance in encoder ticks
     */
    public void setPositionTolerance(int tolerance) {
        // Note: This would require making POSITION_TOLERANCE non-final
        // For now, this is a placeholder for future enhancement
    }

    /**
     * Manually set motor power (for testing)
     * 
     * @param power Motor power (-1.0 to 1.0)
     */
    public void setManualPower(double power) {
        spindexerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        spindexerMotor.setPower(Range.clip(power, -1.0, 1.0));
    }

    /**
     * Reset encoder and PID state
     */
    public void reset() {
        spindexerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexerMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        targetPosition = 0;
        integral = 0;
        lastError = 0;
    }

    /**
     * Update telemetry with spindexer information
     */
    public void updateTelemetry() {
        if (telemetry != null) {
            telemetry.addData("Spindexer Status", "Active");
            telemetry.addData("Current Position", getCurrentPosition());
            telemetry.addData("Target Position", getTargetPosition());
            telemetry.addData("Position Error", getTargetPosition() - getCurrentPosition());
            telemetry.addData("At Position", isAtPosition());
            telemetry.addData("PID Coefficients", "P: %.3f, I: %.3f, D: %.3f", kP, kI, kD);
        }
    }

    /**
     * Normalize an absolute encoder reading into the native 0-TICKS_PER_REV window.
     */
    private int normalizeTicks(int ticks) {
        int normalized = ticks % (int) TICKS_PER_REVOLUTION;
        if (normalized < 0) {
            normalized += TICKS_PER_REVOLUTION;
        }
        return normalized;
    }

    /**
     * Return the shortest signed error between two positions accounting for wrap.
     */
    private double shortestError(int target, int current) {
        int rawError = target - current;
        double revolutions = TICKS_PER_REVOLUTION;
        if (rawError > revolutions / 2) {
            rawError -= revolutions;
        } else if (rawError < -revolutions / 2) {
            rawError += revolutions;
        }
        return rawError;
    }
}
