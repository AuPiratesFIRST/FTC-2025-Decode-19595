package org.firstinspires.ftc.teamcode.SubSystems.Spindexer;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SubSystems.Scoring.ArtifactColor;

/**
 * Vertical spindexer subsystem with PID control for precise position control,
 * now including a two-step sequence for ball settling after rotation.
 */
@Configurable
public class OldSpindexerSubsystem {

    private final DcMotorEx spindexerMotor;
    private final Telemetry telemetry;

    // Motor specifications - GoBILDA RS-555 with 19.2:1 planetary gearbox
    private static final double MOTOR_MAX_RPM = 312.0;
    private static final double TICKS_PER_REVOLUTION = 2150.8;
    private static final double DEGREES_PER_TICK = 360.0 / TICKS_PER_REVOLUTION;

    // Three positions for three active parts (in encoder ticks)
    private static final int POSITION_1_TICKS = 0;
    private static final int POSITION_2_TICKS = (int) (TICKS_PER_REVOLUTION / 3.0);
    private static final int POSITION_3_TICKS = (int) (TICKS_PER_REVOLUTION * 2.0 / 3.0);

    // Intake positions (evenly spaced 120° apart)
    private static final int[] INTAKE_POSITIONS = {
            POSITION_1_TICKS, // 0
            POSITION_2_TICKS, // ~717
            POSITION_3_TICKS // ~1434
    };

    // Outtake positions (raw encoder values from physical measurement)
    // These are the actual encoder ticks when physically moved to outtake positions
    // Using raw values instead of normalized to avoid position calculation issues
    private static final int[] OUTTAKE_POSITIONS = {
            2060, // Position 0 - raw encoder value
            1885, // Position 1 - raw encoder value
            1716 // Position 2 - raw encoder value
    };

    // Ball settling constants
    // BALL_SETTLE_TICKS: Small counterclockwise movement to push ball past bar
    // SETTLE_POWER: Power used during settling (increased for better torque to push
    // past bar)
    private static final int BALL_SETTLE_TICKS = 34;
    private static final double SETTLE_POWER = 0.6; // Increased from 0.4 for better torque

    private boolean isSettling = false;
    private int settlingTarget = 0;
    private boolean mainTargetReached = false;

    // PID coefficients
    private double kP = 0.012000;
    private double kI = 0.001500;
    private double kD = 0.01800;

    private double integral = 0;
    private double lastError = 0;
    private int targetPosition = 0;

    private static final int POSITION_TOLERANCE = 15;
    private static final double SPEED_MULTIPLIER = 1;
    private static final double MIN_POWER_THRESHOLD = 0.10;

    private boolean pidEnabled = false;
    private boolean testMode = false;

    // ------------------- Intake / Outtake Mode -------------------

    private boolean intakeMode = true;

    public void setIntakeMode(boolean intake) {
        this.intakeMode = intake;
    }

    public boolean getIntakeMode() {
        return intakeMode;
    }

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

    public void setTestMode(boolean enabled) {
        testMode = enabled;
    }

    public void setPIDEnabled(boolean enabled) {
        pidEnabled = enabled;
    }

    private void configureMotor() {
        spindexerMotor.setDirection(DcMotor.Direction.REVERSE);
        spindexerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexerMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        spindexerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        integral = 0;
        lastError = 0;
        targetPosition = 0;
    }

    public void goToPosition(SpindexerPosition position) {
        pidEnabled = true;
        targetPosition = normalizeTicks(position.getTicks());
        integral = 0;
        lastError = 0;
        isSettling = false;
        mainTargetReached = false;
    }

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
                if (telemetry != null && testMode) {
                    telemetry.addData("Spindexer Error", "Invalid position index: " + index);
                }
        }
    }

    public void update() {
        if (!pidEnabled) {
            spindexerMotor.setPower(0);
            return;
        }

        int currentPosition = spindexerMotor.getCurrentPosition();
        double error = shortestError(targetPosition, currentPosition);

        // --- Ball Settling State Machine ---
        if (isSettling && Math.abs(shortestError(settlingTarget, currentPosition)) <= POSITION_TOLERANCE) {
            spindexerMotor.setPower(0);
            isSettling = false;
            mainTargetReached = true;
            if (telemetry != null && testMode) {
                telemetry.addData("Spindexer Status", "SETTLING COMPLETE - STOPPED");
            }
            return;
        }

        if (!isSettling && !mainTargetReached && isAtPosition()) {
            settlingTarget = targetPosition - BALL_SETTLE_TICKS;
            targetPosition = normalizeTicks(settlingTarget);
            isSettling = true;
        }

        double currentPowerLimit = isSettling ? SETTLE_POWER : 1;

        if (isAtPosition() && !isSettling && mainTargetReached) {
            spindexerMotor.setPower(0);
            integral = 0;
            lastError = 0;
            if (telemetry != null && testMode) {
                telemetry.addData("Spindexer Status", "AT POSITION - STOPPED");
            }
            return;
        }

        // --- PID calculation ---
        error = shortestError(targetPosition, currentPosition);
        double proportional = error * kP;

        if (Math.abs(error) > POSITION_TOLERANCE) {
            integral += error;
        } else {
            integral = 0;
        }
        integral = Range.clip(integral, -500, 500);
        double integralTerm = integral * kI;

        double derivative = (error - lastError) * kD;
        lastError = error;

        double output = proportional + integralTerm + derivative;
        output = Range.clip(output, -currentPowerLimit, currentPowerLimit);
        output *= SPEED_MULTIPLIER;

        if (Math.abs(output) < MIN_POWER_THRESHOLD) {
            output = 0;
        }

        spindexerMotor.setPower(output);

        if (telemetry != null && testMode) {
            telemetry.addData("Spindexer Target", targetPosition);
            telemetry.addData("Spindexer Current", currentPosition);
            telemetry.addData("Spindexer Error", "%.1f", error);
            telemetry.addData("Spindexer Power", "%.2f", output);
            telemetry.addData("At Position", isAtPosition());
            telemetry.addData("Is Settling", isSettling);
        }
    }

    public boolean isAtPosition() {
        int currentPosition = spindexerMotor.getCurrentPosition();
        return Math.abs(shortestError(targetPosition, currentPosition)) <= POSITION_TOLERANCE;
    }

    public int getCurrentPosition() {
        return spindexerMotor.getCurrentPosition();
    }

    public int getTargetPosition() {
        return targetPosition;
    }

    public void setPIDCoefficients(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public double[] getPIDCoefficients() {
        return new double[] { kP, kI, kD };
    }

    /**
     * Set manual power for direct motor control (used in manual mode).
     * Motor runs in RUN_WITHOUT_ENCODER mode for maximum torque.
     * 
     * @param power Motor power (-1.0 to 1.0). Positive = forward, Negative =
     *              reverse
     */
    public void setManualPower(double power) {
        spindexerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // No power scaling here - let TeleOp control the multiplier for manual mode
        spindexerMotor.setPower(Range.clip(power, -1.0, 1.0));
    }

    /**
     * Get recommended manual power multiplier for TeleOp.
     * Higher value = more torque but less control.
     * For 312 RPM motor with high torque, 0.7-0.8 is recommended.
     * 
     * @return Recommended manual power multiplier (0.0 to 1.0)
     */
    public static double getRecommendedManualPowerMultiplier() {
        return 0.75; // 75% power for good balance of torque and control
    }

    public void reset() {
        spindexerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexerMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        targetPosition = 0;
        integral = 0;
        lastError = 0;
        isSettling = false;
        mainTargetReached = false;
    }

    public void resetPIDOnly() {
        integral = 0;
        lastError = 0;
        isSettling = false;
        mainTargetReached = false;
    }
    
    private int normalizeTicks(int ticks) {
        int normalized = ticks % (int) TICKS_PER_REVOLUTION;
        if (normalized < 0) {
            normalized += TICKS_PER_REVOLUTION;
        }
        return normalized;
    }

    private double shortestError(int target, int current) {
        double raw = target - current;

        // FORCE always moving FORWARD from position 2 to 0
        // when target == 0 AND current is near the high end of the rotation
        if (target == 0 && current > (TICKS_PER_REVOLUTION * 0.5)) {
            return (TICKS_PER_REVOLUTION - current); // forward to finish the circle
        }

        // --- Normal shortest path logic ---
        if (raw > TICKS_PER_REVOLUTION / 2)
            raw -= TICKS_PER_REVOLUTION;
        if (raw < -TICKS_PER_REVOLUTION / 2)
            raw += TICKS_PER_REVOLUTION;

        return raw;
    }

    public void updateTelemetry() {
        if (telemetry == null)
            return;

        int current = getCurrentPosition();
        double error = shortestError(targetPosition, current);

        telemetry.addData("Spindexer Target", targetPosition);
        telemetry.addData("Spindexer Current", current);
        telemetry.addData("Spindexer Error", "%.1f", error);
        telemetry.addData("At Position", isAtPosition());
        telemetry.addData("Is Settling", isSettling);
        telemetry.addData("Main Target Reached", mainTargetReached);
    }

    // ============================================================
    // *** REQUIRED METHODS ADDED BELOW ***
    // ============================================================

    /** Returns whether the PID controller is currently rotating */
    public boolean isMoving() {
        return pidEnabled && !mainTargetReached;
    }

    /** Public accessor for isSettling so AutoOuttakeController can read it */
    public boolean isSettlingPublic() {
        return isSettling;
    }

    /** Returns which index (0/1/2) the target tick position corresponds to */
    public int getIndex() {
        int t = targetPosition;

        int pos1 = POSITION_1_TICKS;
        int pos2 = POSITION_2_TICKS;
        int pos3 = POSITION_3_TICKS;

        double d1 = Math.abs(shortestError(pos1, t));
        double d2 = Math.abs(shortestError(pos2, t));
        double d3 = Math.abs(shortestError(pos3, t));

        if (d1 <= d2 && d1 <= d3)
            return 0;
        if (d2 <= d1 && d2 <= d3)
            return 1;
        return 2;
    }

    // Make old call compatible: AutoOuttakeController expects isSettling()
    public boolean isSettling() {
        return isSettling;
    }

    // ============================================================
    // *** INTAKE/OUTTAKE POSITION METHODS ***
    // ============================================================

    /**
     * Get intake position ticks for a given index (0, 1, or 2)
     * 
     * @param index Position index (0, 1, or 2)
     * @return Encoder ticks for the intake position, or -1 if invalid index
     */
    public static int getIntakePositionTicks(int index) {
        if (index >= 0 && index < INTAKE_POSITIONS.length) {
            return INTAKE_POSITIONS[index];
        }
        return -1;
    }

    /**
     * Get outtake position ticks for a given index (0, 1, or 2)
     * Returns raw encoder values (not normalized) for direct position control.
     * 
     * @param index Position index (0, 1, or 2)
     * @return Raw encoder ticks for the outtake position, or -1 if invalid index
     */
    public static int getOuttakePositionTicks(int index) {
        if (index >= 0 && index < OUTTAKE_POSITIONS.length) {
            return OUTTAKE_POSITIONS[index];
        }
        return -1;
    }

    /**
     * Get position ticks for current mode (intake or outtake) and index
     * 
     * @param index Position index (0, 1, or 2)
     * @return Encoder ticks for the position based on current mode, or -1 if
     *         invalid index
     */
    public int getPositionTicksForCurrentMode(int index) {
        if (intakeMode) {
            return getIntakePositionTicks(index);
        } else {
            return getOuttakePositionTicks(index);
        }
    }

    /**
     * Go to position based on current mode (intake/outtake) and index
     * This is the recommended method for TeleOp use as it automatically
     * selects the correct position based on the current mode.
     * 
     * For intake positions: Uses normalized ticks (0-2150 range)
     * For outtake positions: Uses raw encoder values (direct positioning)
     * 
     * @param index Position index (0, 1, or 2)
     */
    public void goToPositionForCurrentMode(int index) {
        int ticks = getPositionTicksForCurrentMode(index);
        if (ticks >= 0) {
            pidEnabled = true;
            // For outtake positions (raw values), use them directly without normalization
            // For intake positions, normalize to ensure they're in valid range
            if (intakeMode) {
                targetPosition = normalizeTicks(ticks);
            } else {
                // Outtake positions are raw values - use directly but ensure they're valid
                targetPosition = ticks;
            }
            integral = 0;
            lastError = 0;
            isSettling = false;
            mainTargetReached = false;
        } else if (telemetry != null && testMode) {
            telemetry.addData("Spindexer Error", "Invalid position index: " + index);
        }
    }

    /**
     * Get all intake positions as an array
     * 
     * @return Array of intake position ticks
     */
    public static int[] getIntakePositions() {
        return INTAKE_POSITIONS.clone();
    }

    /**
     * Get all outtake positions as an array (raw encoder values)
     * 
     * @return Array of outtake position ticks (raw encoder values)
     */
    public static int[] getOuttakePositions() {
        return OUTTAKE_POSITIONS.clone();
    }

    /**
     * Get the starting spindexer position index based on motif pattern.
     * The first color in the motif determines which position should be shot first.
     * 
     * For motif GPG:
     * - Position 0 has first color (G) - should shoot first
     * - Returns index 0
     * 
     * @param motif 3-color motif array (e.g., [GREEN, PURPLE, GREEN])
     * @return Starting position index (0, 1, or 2) for shooting sequence
     */
    public static int getStartingPositionFromMotif(ArtifactColor[] motif) {
        if (motif == null || motif.length < 3) {
            return 0; // Default to position 0 if invalid motif
        }

        // The first color in the motif is at position 0
        // So we start shooting from position 0
        // This assumes positions 0, 1, 2 correspond to motif colors 0, 1, 2
        return 0; // Always start at position 0 (first ball to shoot)
    }

    /**
     * Rotate spindexer to the correct starting position based on motif.
     * This should be called when you want to prepare for shooting based on detected
     * motif.
     * 
     * @param motif 3-color motif array
     */
    public void rotateToMotifStartPosition(ArtifactColor[] motif) {
        int startIndex = getStartingPositionFromMotif(motif);
        goToPositionForCurrentMode(startIndex);
    }
}
