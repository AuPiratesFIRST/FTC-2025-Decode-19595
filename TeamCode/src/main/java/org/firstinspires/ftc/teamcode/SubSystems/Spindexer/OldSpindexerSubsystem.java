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

    // FIXED: goBILDA 5202-2402-0019 (19.2:1) publishes 537.7 "Pulses per Rotation"
    // at the output shaft, and that number IS the fully quadrature-decoded tick
    // count getCurrentPosition() returns per revolution (28 counts at the encoder
    // shaft x 19.2 gearing = 537.7; REV's own SDK uses this the same way, e.g.
    // ticksPerRev = 28 x gearing for other goBILDA ratios). Do NOT multiply by 4
    // again -- that was the previous bug (2150.8 = 537.7 x 4), which made the
    // shortest-path wraparound window 4x too wide.
    private static final double TICKS_PER_REVOLUTION = 537.7;

    // Position definitions (unchanged -- these were tuned against real hardware
    // ticks, which were always genuinely 537.7-scale regardless of the old,
    // incorrect wraparound constant, so they don't need to be rescaled)
    public static final int[] INTAKE_POSITIONS = { 0, 210, 383 };
    public static final int[] OUTTAKE_POSITIONS = { 268, 451, 630 };

    // COMPETITION TUNED COEFFICIENTS - Aggressive "Lockdown" Tuning
    private static final double kP_EMPTY = 0.02350000000000001;
    private static final double kI_EMPTY = 0;
    private static final double kD_EMPTY = 0;

    private static final double kP_LOADED = 0.035;
    private static final double kI_LOADED = 0;
    private static final double kD_LOADED = 0;

    private double kP = kP_EMPTY;
    private double kI = kI_EMPTY;
    private double kD = kD_EMPTY;

    private double lastError = 0;
    private double integralSum = 0;
    private boolean hasPrevError = false;
    private int targetPosition = 0;
    private int encoderOffset = 0;

    // FIXED (Bug 2/1 root cause): continuous, unwrapped target that the motion
    // profiler chases. Lives in the SAME domain as getCurrentPosition() (which is
    // already continuous/unbounded -- it's never wrapped). targetPosition (above)
    // stays as a normalized [0, TICKS_PER_REVOLUTION) value for index lookups /
    // telemetry / compatibility, but all profiling and error math now uses this
    // continuous value instead of re-normalizing every tick.
    private double continuousTarget = 0.0;

    private static final int POSITION_TOLERANCE = 8;
    private static final double SPEED_MULTIPLIER = 0.2;
    private static final double GRAVITY_FEEDFORWARD = 0.08;
    private static final double kS = 0.05;
    private static final double kV = 0.0002;
    private static final double kA = 0.00002;
    private static final double GRAVITY_DIRECTION_ERROR_DEADBAND = 4.0;
    private static final double GRAVITY_DIRECTION_VELOCITY_DEADBAND = 20.0;

    private static final boolean MOTION_PROFILE_ENABLED = true;
    private static final double MAX_PROFILE_VELOCITY = 900.0;
    private static final double MAX_PROFILE_ACCELERATION = 2200.0;

    private boolean pidEnabled = false;
    private boolean tuningMode = false;
    private boolean intakeMode = true;
    private int ballCount = 0;
    private double profiledTargetPosition = 0.0;
    private double profileVelocity = 0.0;
    private double lastProfileVelocity = 0.0;
    private long lastUpdateNanos = 0L;
    private double gravityDirectionSign = 1.0;
    private double lastFinalOutput = 0.0;
    private double lastProfileAcceleration = 0.0;
    private double lastStaticFrictionFF = 0.0;
    private double lastGravityFF = 0.0;
    private double lastDynamicFF = 0.0;

    public enum SpindexerPosition {
        POSITION_1(10), POSITION_2(215), POSITION_3(392);
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

        if (intakeMode || ballCount == 0) {
            kP = kP_EMPTY;
            kI = kI_EMPTY;
            kD = kD_EMPTY;
        } else {
            double scale = 0.7 + (0.1 * ballCount); // 1->0.8, 2->0.9, 3->1.0
            kP = kP_LOADED * scale;
            kI = kI_LOADED * scale;
            kD = kD_LOADED * scale;
        }

        int currentPosition = getCurrentPosition();
        long now = System.nanoTime();
        double dtSec = lastUpdateNanos == 0L ? 0.02 : Math.max(0.001, (now - lastUpdateNanos) / 1_000_000_000.0);
        lastUpdateNanos = now;

        if (!MOTION_PROFILE_ENABLED) {
            // FIXED: chase the continuous target directly, not a re-normalized one
            profiledTargetPosition = continuousTarget;
            profileVelocity = 0.0;
        } else {
            updateMotionProfile(dtSec);
        }

        // FIXED (Bug 1/2): plain continuous subtraction. profiledTargetPosition and
        // currentPosition are both already unwrapped/continuous and were brought
        // into agreement when the move was issued (see setNormalizedTarget /
        // goToPositionForwardOnly / goToPositionBackwardOnly), so no shortestError()
        // re-normalization is needed -- or wanted -- here.
        double error = profiledTargetPosition - currentPosition;

        if (!hasPrevError) {
            lastError = error;
            hasPrevError = true;
        }

        integralSum += error;

        // FIXED (Bug 4): clamp the integral's power contribution directly instead
        // of dividing by kI (which blew up to +-4e8 when kI == 0, disabling
        // anti-windup entirely). When kI is zero/negative, the integral term
        // contributes nothing, so just keep the accumulator at zero.
        if (kI > 1e-9) {
            double maxIntegralPower = 0.4;
            integralSum = Range.clip(integralSum, -maxIntegralPower / kI, maxIntegralPower / kI);
        } else {
            integralSum = 0;
        }

        double derivative = ((error - lastError) / dtSec) * kD;
        lastError = error;

        double output = (error * kP) + (integralSum * kI) + derivative;

        double staticFrictionFF = 0;
        if (Math.abs(error) > 1) {
            staticFrictionFF = Math.signum(error) * kS;
        }

        double gravityFF = 0;
        if (!intakeMode) {
            if (Math.abs(profileVelocity) > GRAVITY_DIRECTION_VELOCITY_DEADBAND) {
                gravityDirectionSign = Math.signum(profileVelocity);
            } else if (Math.abs(error) > GRAVITY_DIRECTION_ERROR_DEADBAND) {
                gravityDirectionSign = Math.signum(error);
            }
            gravityFF = GRAVITY_FEEDFORWARD * gravityDirectionSign;
        }

        double profileAcceleration = (profileVelocity - lastProfileVelocity) / dtSec;
        lastProfileVelocity = profileVelocity;
        double dynamicFF = (kV * profileVelocity) + (kA * profileAcceleration);

        double finalOutput = output + staticFrictionFF + gravityFF + dynamicFF;
        lastProfileAcceleration = profileAcceleration;
        lastStaticFrictionFF = staticFrictionFF;
        lastGravityFF = gravityFF;
        lastDynamicFF = dynamicFF;
        lastFinalOutput = finalOutput;
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
        telemetry.addData("Spindexer Continuous Target", "%.1f", continuousTarget);
        telemetry.addData("Spindexer Profiled Target", "%.1f", profiledTargetPosition);
        telemetry.addData("Spindexer Current", getCurrentPosition());
        telemetry.addData("Ball Count", ballCount);
        telemetry.addData("Encoder Raw", spindexerMotor.getCurrentPosition());
        telemetry.addData("Encoder Offset", encoderOffset);
        telemetry.addData("Hold Power", "%.2f", spindexerMotor.getPower());
        telemetry.addData("FF Breakdown", "S:%.2f, G:%.2f, V/A:%.2f",
                getLastStaticFrictionFF(), getLastGravityFF(), getLastDynamicFF());
        telemetry.addData("Profile Vel", "%.2f", getProfileVelocity());
    }

    public void goToPosition(int index) {
        if (index >= 0 && index <= 2) goToPositionForCurrentMode(index);
        else {
            setNormalizedTarget(index);
            setPIDEnabled(true);
        }
    }

    public void goToPosition(SpindexerPosition position) {
        setNormalizedTarget(position.getTicks());
        setPIDEnabled(true);
    }

    public void goToPositionForCurrentMode(int index) {
        int ticks = intakeMode ? INTAKE_POSITIONS[index] : OUTTAKE_POSITIONS[index];
        setNormalizedTarget(ticks);
        setPIDEnabled(true);
        resetPIDOnly();
    }

    /**
     * Switch to outtake mode using forward-only movement to prevent jamming.
     * Call this when switching from intake -> outtake to ensure balls don't get pushed backward.
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
        int current = getCurrentPosition();
        targetPosition = normalizeTicks(current);
        continuousTarget = current;
        setPIDEnabled(true);
        resetPIDOnly();
    }

    /**
     * Calibrates the spindexer by setting the current physical position as a specific index.
     * Useful for fixing the "starting position" issue if the robot is initialized misaligned.
     */
    public void calibrateCurrentPosition(int index) {
        int expectedTicks = intakeMode ? INTAKE_POSITIONS[index] : OUTTAKE_POSITIONS[index];
        encoderOffset = expectedTicks - spindexerMotor.getCurrentPosition();
        setNormalizedTarget(expectedTicks);
        resetPIDOnly();
    }

    /**
     * Move to target position using FORWARD-ONLY (clockwise) movement.
     * Ensures shortest forward path without going backward.
     *
     * FIXED (Bug 2): continuousTarget is now set directly to current + forwardDistance
     * (a continuous, unbounded value) and the motion profiler chases continuousTarget
     * with plain subtraction -- no shortestError() re-normalization in the loop that
     * could snap the direction back to "nearest," which is what silently overrode the
     * forward-only intent before.
     */
    public void goToPositionForwardOnly(int ticksTarget) {
        int current = getCurrentPosition();
        int normalizedCurrent = normalizeTicks(current);
        int normalized = normalizeTicks(ticksTarget);

        int forwardDistance = (normalized - normalizedCurrent + (int) TICKS_PER_REVOLUTION) % (int) TICKS_PER_REVOLUTION;

        targetPosition = normalized;
        continuousTarget = current + forwardDistance;
        setPIDEnabled(true);
        resetPIDOnly();
    }

    /**
     * Move to target position using BACKWARD-ONLY (counter-clockwise) movement.
     * Ensures movement in reverse direction.
     */
    public void goToPositionBackwardOnly(int ticksTarget) {
        int current = getCurrentPosition();
        int normalizedCurrent = normalizeTicks(current);
        int normalized = normalizeTicks(ticksTarget);

        int backwardDistance = (normalizedCurrent - normalized + (int) TICKS_PER_REVOLUTION) % (int) TICKS_PER_REVOLUTION;
        if (backwardDistance == 0) {
            backwardDistance = (int) TICKS_PER_REVOLUTION;
        }

        targetPosition = normalized;
        continuousTarget = current - backwardDistance;
        setPIDEnabled(true);
        resetPIDOnly();
    }

    // === UTILITY METHODS ===

    public void reset() {
        spindexerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexerMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        encoderOffset = 0;
        targetPosition = 0;
        continuousTarget = 0.0;
        resetPIDOnly();
        pidEnabled = false;
    }

    public void resetPIDOnly() {
        lastError = 0;
        integralSum = 0;
        hasPrevError = false;
        profileVelocity = 0.0;
        lastProfileVelocity = 0.0;
        profiledTargetPosition = getCurrentPosition();
        lastUpdateNanos = System.nanoTime();
    }

    /**
     * Sets a normalized (index-style) target and computes the nearest continuous
     * absolute equivalent to chase, using shortestError ONCE at command time (not
     * every tick). This is the correct place for "nearest direction" wraparound
     * logic -- it should decide direction when a move is issued, not get
     * re-applied every control-loop tick while the move is in progress.
     */
    private void setNormalizedTarget(int ticks) {
        int normalized = normalizeTicks(ticks);
        int current = getCurrentPosition();
        targetPosition = normalized;
        continuousTarget = current + shortestError(normalized, current);
    }

    public boolean isAtPosition() {
        return Math.abs(continuousTarget - getCurrentPosition()) <= POSITION_TOLERANCE;
    }

    public boolean isMoving() {
        return pidEnabled && Math.abs(continuousTarget - getCurrentPosition()) > POSITION_TOLERANCE;
    }

    /** Shortest-path wraparound distance. Use only for one-shot "nearest direction"
     *  decisions (e.g. picking a continuous target, or comparing indices) -- NOT
     *  inside the continuous motion-profile loop. */
    public double shortestError(int target, int current) {
        int nTarget = normalizeTicks(target);
        int nCurrent = normalizeTicks(current);
        double raw = (double) nTarget - nCurrent;
        if (raw > TICKS_PER_REVOLUTION / 2.0) raw -= TICKS_PER_REVOLUTION;
        if (raw < -TICKS_PER_REVOLUTION / 2.0) raw += TICKS_PER_REVOLUTION;
        return raw;
    }

    public double shortestError(double target, double current) {
        double nTarget = normalizeTicks(target);
        double nCurrent = normalizeTicks(current);
        double raw = nTarget - nCurrent;
        if (raw > TICKS_PER_REVOLUTION / 2.0) raw -= TICKS_PER_REVOLUTION;
        if (raw < -TICKS_PER_REVOLUTION / 2.0) raw += TICKS_PER_REVOLUTION;
        return raw;
    }

    public int normalizeTicks(int ticks) {
        int normalized = ticks % (int) TICKS_PER_REVOLUTION;
        if (normalized < 0) normalized += (int) TICKS_PER_REVOLUTION;
        return normalized;
    }

    public double normalizeTicks(double ticks) {
        double normalized = ticks % TICKS_PER_REVOLUTION;
        if (normalized < 0) normalized += TICKS_PER_REVOLUTION;
        return normalized;
    }

    /**
     * FIXED (Bug 1/2): operates entirely in continuous ticks now. profiledTargetPosition
     * chases continuousTarget with plain subtraction and is never wrapped back into
     * [0, TICKS_PER_REVOLUTION) mid-move, so a forced-direction target that's
     * legitimately more than half a revolution away stays correct for the whole move.
     */
    private void updateMotionProfile(double dtSec) {
        double positionErrorToGoal = continuousTarget - profiledTargetPosition;
        double direction = Math.signum(positionErrorToGoal);
        double stoppingDistance = (profileVelocity * profileVelocity) / (2.0 * MAX_PROFILE_ACCELERATION);

        double desiredVelocity;
        if (Math.abs(positionErrorToGoal) <= stoppingDistance) {
            desiredVelocity = 0.0;
        } else {
            desiredVelocity = direction * MAX_PROFILE_VELOCITY;
        }

        double maxDeltaV = MAX_PROFILE_ACCELERATION * dtSec;
        double velocityDelta = desiredVelocity - profileVelocity;
        velocityDelta = Range.clip(velocityDelta, -maxDeltaV, maxDeltaV);
        profileVelocity += velocityDelta;

        double step = profileVelocity * dtSec;
        if (Math.abs(step) > Math.abs(positionErrorToGoal)) {
            step = positionErrorToGoal;
            profileVelocity = 0.0;
        }

        profiledTargetPosition += step;
    }

    public void stopManual() {
        spindexerMotor.setPower(0);
        pidEnabled = false;
    }

    /**
     * FIXED (Bug 3): resets PID/profile state automatically whenever PID transitions
     * from disabled -> enabled, so re-enabling after setManualPower() (or any other
     * disabled period) can never resume with a stale integral or a stale
     * profiledTargetPosition. Command methods can still call resetPIDOnly() explicitly
     * afterward too (harmless / idempotent) -- this is just a safety net so no call
     * site can forget it.
     */
    public void setPIDEnabled(boolean enabled) {
        if (enabled && !this.pidEnabled) {
            resetPIDOnly();
        }
        this.pidEnabled = enabled;
    }

    public void setIntakeMode(boolean intake) {
        boolean modeChanged = this.intakeMode != intake;
        this.intakeMode = intake;
        if (modeChanged) {
            resetPIDOnly();
        }
    }

    public void setBallCount(int count) {
        this.ballCount = Range.clip(count, 0, 3);
    }

    public void setTuningMode(boolean enabled) { this.tuningMode = enabled; }
    public boolean isSettling() { return false; }
    public int getCurrentPosition() { return spindexerMotor.getCurrentPosition() + encoderOffset; }
    public int getTargetPosition() { return targetPosition; }
    public double getContinuousTargetPosition() { return continuousTarget; }
    public double getProfiledTargetPosition() { return profiledTargetPosition; }
    public double getProfileVelocity() { return profileVelocity; }
    public double getProfileAcceleration() { return lastProfileAcceleration; }
    public double getLastControlOutput() { return lastFinalOutput; }
    public double getLastStaticFrictionFF() { return lastStaticFrictionFF; }
    public double getLastGravityFF() { return lastGravityFF; }
    public double getLastDynamicFF() { return lastDynamicFF; }
    public void setManualPower(double power) { pidEnabled = false; spindexerMotor.setPower(power); }
    public void rotateToMotifStartPosition(ArtifactColor[] motif) { goToPositionForCurrentMode(0); }
    public static double getRecommendedManualPowerMultiplier() { return 0.75; }
}