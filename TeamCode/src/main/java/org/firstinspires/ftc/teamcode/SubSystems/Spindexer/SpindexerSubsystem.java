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
    private static final double TICKS_PER_REVOLUTION = 560.0; // Standard GoBILDA encoder ticks
    private static final double DEGREES_PER_TICK = 360.0 / TICKS_PER_REVOLUTION;

    // Three positions for three active arts (in encoder ticks)
    // These values should be calibrated based on your physical setup
    // Position calculations: evenly spaced at 120° intervals (360° / 3 = 120°)
    private static final int POSITION_1_TICKS = 0; // First art position (0°)
    private static final int POSITION_2_TICKS = (int) (TICKS_PER_REVOLUTION / 3.0); // 120 degrees (560/3 ≈ 187 ticks)
    private static final int POSITION_3_TICKS = (int) (TICKS_PER_REVOLUTION * 2.0 / 3.0); // 240 degrees (560×2/3 ≈ 373
                                                                                          // ticks)

    // PID coefficients - tune these for your specific setup
    private double kP = 0.05; // Proportional gain
    private double kI = 0.001; // Integral gain
    private double kD = 0.01; // Derivative gain

    // PID state variables
    private double integral = 0;
    private double lastError = 0;
    private int targetPosition = 0;

    // Position tolerance (in ticks)
    private static final int POSITION_TOLERANCE = 10;

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
        spindexerMotor = hardwareMap.get(DcMotorEx.class, "spindexer");

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
        targetPosition = position.getTicks();
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
     * - P term: error × kP (proportional to current error)
     * - I term: Σ(error) × kI (eliminates steady-state error, with anti-windup)
     * - D term: (error - lastError) × kD (reduces overshoot and oscillation)
     * 
     * Output equation: power = (error × kP) + (integral × kI) - (derivative × kD)
     * The derivative is subtracted because it represents rate of change (damping).
     * 
     * Anti-windup: limits integral accumulation to ±1000 to prevent excessive
     * correction.
     */
    public void update() {
        int currentPosition = spindexerMotor.getCurrentPosition();
        double error = targetPosition - currentPosition;

        // Proportional term: immediate response to error
        // P = error × kP (larger error = larger correction)
        double proportional = error * kP;

        // Integral term: accumulates error over time to eliminate steady-state error
        // I = Σ(error) × kI (handles persistent small errors)
        integral += error;
        // Anti-windup: limit integral to prevent excessive correction
        if (integral > 1000)
            integral = 1000;
        if (integral < -1000)
            integral = -1000;
        double integralTerm = integral * kI;

        // Derivative term: rate of change of error (damping)
        // D = (error - lastError) × kD (reduces overshoot and oscillation)
        double derivative = (error - lastError) * kD;
        lastError = error;

        // PID output equation: output = P + I - D
        // D is subtracted because it represents damping (slows down movement)
        double output = proportional + integralTerm - derivative;

        // Clip output to valid motor power range [-1.0, 1.0]
        output = Range.clip(output, -1.0, 1.0);

        // Apply power to motor
        spindexerMotor.setPower(output);

        if (telemetry != null) {
            telemetry.addData("Spindexer Target", targetPosition);
            telemetry.addData("Spindexer Current", currentPosition);
            telemetry.addData("Spindexer Error", "%.1f", error);
            telemetry.addData("Spindexer Power", "%.2f", output);
            telemetry.addData("At Position", isAtPosition());
        }
    }

    /**
     * Check if spindexer is at target position
     * 
     * @return True if within tolerance
     */
    public boolean isAtPosition() {
        int currentPosition = spindexerMotor.getCurrentPosition();
        return Math.abs(currentPosition - targetPosition) <= POSITION_TOLERANCE;
    }

    /**
     * Get current position in ticks
     * 
     * @return Current encoder position
     */
    public int getCurrentPosition() {
        return spindexerMotor.getCurrentPosition();
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
}
