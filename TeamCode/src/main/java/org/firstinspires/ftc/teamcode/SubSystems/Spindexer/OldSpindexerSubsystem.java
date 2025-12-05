package org.firstinspires.ftc.teamcode.SubSystems.Spindexer;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Vertical spindexer subsystem with PID control for precise position control,
 * now including a two-step sequence for ball settling after rotation.
 */
public class OldSpindexerSubsystem {

    private final DcMotorEx spindexerMotor;
    private final Telemetry telemetry;

    // Motor specifications - GoBILDA RS-555 with 19.2:1 planetary gearbox
    private static final double MOTOR_MAX_RPM = 312.0;
    private static final double TICKS_PER_REVOLUTION = 2150.8;
    private static final double DEGREES_PER_TICK = 360.0 / TICKS_PER_REVOLUTION;

    // Three positions for three active arts (in encoder ticks)
    private static final int POSITION_1_TICKS = 0;
    private static final int POSITION_2_TICKS = (int) (TICKS_PER_REVOLUTION / 3.0);
    private static final int POSITION_3_TICKS = (int) (TICKS_PER_REVOLUTION * 2.0 / 3.0);

    // ***************************************************************
    // ⚙️ BALL SETTLING CONSTANTS AND STATE
    // ***************************************************************
    private static final int BALL_SETTLE_TICKS = 34; // Distance for the small correction
    private static final double SETTLE_POWER = 0.4;  // Power limit for the slow correction move

    // Tracking the state machine progress
    private boolean isSettling = false;              // Tracks if the settling move is active
    private int settlingTarget = 0;                  // Stores the target position for the settling move
    private boolean mainTargetReached = false;       // Tracks if the main target has been reached but settling hasn't started
    // ***************************************************************

    // PID coefficients - tune these for your specific setup
    private double kP = 0.0430;
    private double kI = 0.0009;
    private double kD = 0.02;

    // PID state variables
    private double integral = 0;
    private double lastError = 0;
    private int targetPosition = 0;

    // Position tolerance (in ticks)
    private static final int POSITION_TOLERANCE = 15;

    // Speed multiplier for testing (0.25 = quarter speed)
    private static final double SPEED_MULTIPLIER = 1;

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

    public OldSpindexerSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        spindexerMotor = hardwareMap.get(DcMotorEx.class, "Spindexer");
        configureMotor();
    }

    private void configureMotor() {
        // Set motor direction to REVERSE
        spindexerMotor.setDirection(DcMotor.Direction.REVERSE);

        spindexerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexerMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        spindexerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Reset PID state variables
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
        integral = 0;
        lastError = 0;
        // Reset settling flags when a new main target is set
        isSettling = false;
        mainTargetReached = false;
    }

    /**
     * Move to a specific position by index (0, 1, or 2)
     *
     * @param index Position index (0, 1, or 2)
     */
    public void goToPosition(int index) {
        switch (index) {
            case 0: goToPosition(SpindexerPosition.POSITION_1); break;
            case 1: goToPosition(SpindexerPosition.POSITION_2); break;
            case 2: goToPosition(SpindexerPosition.POSITION_3); break;
            default:
                if (telemetry != null) {
                    telemetry.addData("Spindexer Error", "Invalid position index: " + index);
                }
        }
    }

    /**
     * Update PID control - call this in your main loop.
     */
    public void update() {
        int currentPosition = normalizeTicks(spindexerMotor.getCurrentPosition());
        double error = shortestError(targetPosition, currentPosition);

        // ------------------------------------------------------------
        // ➡️ LOGIC FOR BALL SETTLING STATE MACHINE
        // ------------------------------------------------------------

        // 1. If the motor is currently settling and the settling target is reached, stop the process.
        if (isSettling && Math.abs(shortestError(settlingTarget, currentPosition)) <= POSITION_TOLERANCE) {
            spindexerMotor.setPower(0);
            isSettling = false; // Settling is complete
            mainTargetReached = true; // Mark final position as reached
            if (telemetry != null) {
                telemetry.addData("Spindexer Status", "SETTLING COMPLETE - STOPPED");
            }
            return;
        }

        // 2. If the main target is reached AND the settling move hasn't started, initiate the settling move.
        // This is where the settling logic is triggered.
        if (!isSettling && !mainTargetReached && isAtPosition()) {

            // Calculate the settling target (a small movement backwards/downwards)
            settlingTarget = targetPosition - BALL_SETTLE_TICKS;

            // Temporarily change the target position to the settling target
            // Note: targetPosition is normalized, so settlingTarget must also be normalized
            targetPosition = normalizeTicks(settlingTarget);

            // Set the state to settling, which will use the main PID loop for the small move
            isSettling = true;

            // Do NOT return here. Continue to the PID calculation to drive motor toward new target.
        }

        // 3. Determine the power limit: lower power for settling, normal for main move.
        double currentPowerLimit = isSettling ? SETTLE_POWER : 1;

        // 4. If the spindexer is completely at rest (main target reached and settling done), stop and exit.
        if (isAtPosition() && !isSettling && mainTargetReached) {
            spindexerMotor.setPower(0);
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

        // ------------------------------------------------------------
        // ➡️ Standard PID Control Calculation
        // ------------------------------------------------------------

        // Error calculation must be against the current target (either main or settling)
        error = shortestError(targetPosition, currentPosition);

        // Proportional term
        double proportional = error * kP;

        // Integral term (with anti-windup logic)
        if (Math.abs(error) > POSITION_TOLERANCE) {
            integral += error;
        } else {
            integral = 0;
        }
        integral = Range.clip(integral, -500, 500);
        double integralTerm = integral * kI;

        // Derivative term
        double derivative = (error - lastError) * kD;
        lastError = error;

        // PID output equation
        double output = proportional + integralTerm + derivative;

        // Clip output using the dynamic power limit
        output = Range.clip(output, -currentPowerLimit, currentPowerLimit);

        // Apply speed multiplier (for debugging/testing)
        output *= SPEED_MULTIPLIER;

        // Apply minimum power threshold
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
            telemetry.addData("Is Settling", isSettling);
            telemetry.addData("Power Limit", currentPowerLimit);
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
     * Check if spindexer is at current target position.
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
     */
    public void setPositionTolerance(int tolerance) {
        // Placeholder for future enhancement
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
        isSettling = false;
        mainTargetReached = false;
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

