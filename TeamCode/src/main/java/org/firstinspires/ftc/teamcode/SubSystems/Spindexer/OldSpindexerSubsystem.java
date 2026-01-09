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

    private static final int POSITION_1_TICKS = 10;
    private static final int POSITION_2_TICKS = (int) (TICKS_PER_REVOLUTION / 3.0);
    private static final int POSITION_3_TICKS = (int) (TICKS_PER_REVOLUTION * 2.0 / 3.0);

    private static final int[] INTAKE_POSITIONS = { POSITION_1_TICKS, POSITION_2_TICKS, POSITION_3_TICKS };
    private static final int[] OUTTAKE_POSITIONS = { 2060, 1885, 1716 };

    // PID Coefficients
    private double kP = 0.008; 
    private double kI = 0.0001; // The Active Hold Force
    private double kD = 0.0008;

    private double lastError = 0;
    private double integralSum = 0;
    private boolean hasPrevError = false;
    private int targetPosition = 0;

    private static final int POSITION_TOLERANCE = 15;
    
    // UPDATED: Speed set to 0.75 for torque and smoothness
    private static final double SPEED_MULTIPLIER = 0.75;

    private boolean pidEnabled = false;
    private boolean tuningMode = false;
    private boolean intakeMode = true;

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

    private void configureMotor() {
        spindexerMotor.setDirection(DcMotor.Direction.REVERSE);
        spindexerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexerMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // BRAKE ensures physical motor resistance even when power is 0
        spindexerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public int getIndex() {
        double d1 = Math.abs(shortestError(POSITION_1_TICKS, targetPosition));
        double d2 = Math.abs(shortestError(POSITION_2_TICKS, targetPosition));
        double d3 = Math.abs(shortestError(POSITION_3_TICKS, targetPosition));
        if (d1 <= d2 && d1 <= d3) return 0;
        if (d2 <= d1 && d2 <= d3) return 1;
        return 2;
    }

    public double[] getPIDCoefficients() {
        return new double[]{kP, kI, kD};
    }

    public void setPIDCoefficients(double p, double d) {
        this.kP = p;
        this.kD = d;
    }

    public void setPIDCoefficients(double p, double i, double d) {
        this.kP = p;
        this.kI = i;
        this.kD = d;
    }

    public void updateTelemetry() {
        if (telemetry == null) return;
        telemetry.addData("Spindexer Target", targetPosition);
        telemetry.addData("Spindexer Current", spindexerMotor.getCurrentPosition());
        telemetry.addData("Spindexer Error", "%.1f", shortestError(targetPosition, spindexerMotor.getCurrentPosition()));
        telemetry.addData("Active Hold Power", "%.2f", spindexerMotor.getPower());
    }

    public void goToPosition(int index) {
        if (index >= 0 && index <= 2) {
            goToPositionForCurrentMode(index);
        } else {
            targetPosition = normalizeTicks(index);
            setPIDEnabled(true);
        }
    }

    public void goToPosition(SpindexerPosition position) {
        targetPosition = normalizeTicks(position.getTicks());
        setPIDEnabled(true);
    }

    public void reset() {
        spindexerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexerMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        targetPosition = 0;
        resetPIDOnly();
        pidEnabled = false;
    }

    public void resetPIDOnly() {
        lastError = 0;
        integralSum = 0; // Clear the hold force
        hasPrevError = false;
    }

    public int getCurrentPosition() { return spindexerMotor.getCurrentPosition(); }
    public int getTargetPosition() { return targetPosition; }
    public void setTuningMode(boolean enabled) { this.tuningMode = enabled; }
    public boolean isSettling() { return false; }

    public void setPIDEnabled(boolean enabled) {
        this.pidEnabled = enabled;
        if (enabled) {
            spindexerMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            resetPIDOnly();
        }
    }

    public void setManualPower(double power) {
        pidEnabled = false;
        spindexerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        spindexerMotor.setPower(Range.clip(power, -1.0, 1.0));
    }

    public void lockCurrentPosition() {
        targetPosition = normalizeTicks(spindexerMotor.getCurrentPosition());
        setPIDEnabled(true);
    }

    public void update() {
        if (!pidEnabled) return;

        int currentPosition = spindexerMotor.getCurrentPosition();
        double error = shortestError(targetPosition, currentPosition);

        if (!hasPrevError) {
            lastError = error;
            hasPrevError = true;
        }

        // Active Hold Calculation
        integralSum += error;
        integralSum = Range.clip(integralSum, -0.25 / (kI + 1e-9), 0.25 / (kI + 1e-9));

        double derivative = (error - lastError) * kD;
        lastError = error;

        // Final PID Output
        double output = (error * kP) + (integralSum * kI) + derivative;

        // Final power clip to avoid overheating but keep it strong
        spindexerMotor.setPower(Range.clip(output, -1.0, 1.0) * SPEED_MULTIPLIER);
    }

    public boolean isAtPosition() {
        return Math.abs(shortestError(targetPosition, spindexerMotor.getCurrentPosition())) <= POSITION_TOLERANCE;
    }

    public boolean isMoving() {
        return pidEnabled && Math.abs(shortestError(targetPosition, spindexerMotor.getCurrentPosition())) > POSITION_TOLERANCE;
    }

    public int normalizeTicks(int ticks) {
        int normalized = ticks % (int) TICKS_PER_REVOLUTION;
        if (normalized < 0) normalized += (int) TICKS_PER_REVOLUTION;
        return normalized;
    }

    public double shortestError(int target, int current) {
        int nTarget = normalizeTicks(target);
        int nCurrent = normalizeTicks(current);
        double raw = (double) nTarget - nCurrent;
        if (raw > TICKS_PER_REVOLUTION / 2.0) raw -= TICKS_PER_REVOLUTION;
        if (raw < -TICKS_PER_REVOLUTION / 2.0) raw += TICKS_PER_REVOLUTION;
        return raw;
    }

    public void setIntakeMode(boolean intake) { this.intakeMode = intake; }
    public boolean getIntakeMode() { return intakeMode; }
    public void goToPositionForCurrentMode(int index) {
        int ticks = intakeMode ? INTAKE_POSITIONS[index] : OUTTAKE_POSITIONS[index];
        targetPosition = normalizeTicks(ticks);
        setPIDEnabled(true);
    }
    public void rotateToMotifStartPosition(ArtifactColor[] motif) { goToPositionForCurrentMode(0); }

    public static double getRecommendedManualPowerMultiplier() { return 0.75; }
}