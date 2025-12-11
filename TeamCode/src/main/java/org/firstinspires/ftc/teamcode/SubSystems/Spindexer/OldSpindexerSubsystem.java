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

    // Three positions for three active parts (in encoder ticks)
    private static final int POSITION_1_TICKS = 0;
    private static final int POSITION_2_TICKS = (int) (TICKS_PER_REVOLUTION / 3.0);
    private static final int POSITION_3_TICKS = (int) (TICKS_PER_REVOLUTION * 2.0 / 3.0);

    // Ball settling constants
    private static final int BALL_SETTLE_TICKS = 34;
    private static final double SETTLE_POWER = 0.4;

    private boolean isSettling = false;
    private int settlingTarget = 0;
    private boolean mainTargetReached = false;

    // PID coefficients
    private double kP = 0.0430;
    private double kI = 0.0009;
    private double kD = 0.02;

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
        SpindexerPosition(int ticks) { this.ticks = ticks; }
        public int getTicks() { return ticks; }
    }

    public OldSpindexerSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        spindexerMotor = hardwareMap.get(DcMotorEx.class, "Spindexer");
        configureMotor();
    }

    public void setTestMode(boolean enabled) { testMode = enabled; }

    public void setPIDEnabled(boolean enabled) { pidEnabled = enabled; }

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
            case 0: goToPosition(SpindexerPosition.POSITION_1); break;
            case 1: goToPosition(SpindexerPosition.POSITION_2); break;
            case 2: goToPosition(SpindexerPosition.POSITION_3); break;
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

        if (Math.abs(error) > POSITION_TOLERANCE) { integral += error; }
        else { integral = 0; }
        integral = Range.clip(integral, -500, 500);
        double integralTerm = integral * kI;

        double derivative = (error - lastError) * kD;
        lastError = error;

        double output = proportional + integralTerm + derivative;
        output = Range.clip(output, -currentPowerLimit, currentPowerLimit);
        output *= SPEED_MULTIPLIER;

        if (Math.abs(output) < MIN_POWER_THRESHOLD) { output = 0; }

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

    public int getTargetPosition() { return targetPosition; }

    public void setPIDCoefficients(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public double[] getPIDCoefficients() { return new double[] { kP, kI, kD }; }

    public void setManualPower(double power) {
        spindexerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        spindexerMotor.setPower(Range.clip(power, -1.0, 1.0));
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

    private int normalizeTicks(int ticks) {
        int normalized = ticks % (int) TICKS_PER_REVOLUTION;
        if (normalized < 0) { normalized += TICKS_PER_REVOLUTION; }
        return normalized;
    }

    private double shortestError(int target, int current) {
        int rawError = target - current;
        double revolutions = TICKS_PER_REVOLUTION;
        if (rawError > revolutions / 2) { rawError -= revolutions; }
        else if (rawError < -revolutions / 2) { rawError += revolutions; }
        return rawError;
    }

    public void updateTelemetry() {
        if (telemetry == null) return;

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
    //              *** REQUIRED METHODS ADDED BELOW ***
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

        if (d1 <= d2 && d1 <= d3) return 0;
        if (d2 <= d1 && d2 <= d3) return 1;
        return 2;
    }

    // Make old call compatible: AutoOuttakeController expects isSettling()
    public boolean isSettling() {
        return isSettling;
    }
}
