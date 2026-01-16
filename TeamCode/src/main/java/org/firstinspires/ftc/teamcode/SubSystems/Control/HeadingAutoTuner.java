package org.firstinspires.ftc.teamcode.SubSystems.Control;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Relay-based PID auto-tuner for heading control.
 * 
 * ⚠️ IMPORTANT REQUIREMENTS:
 * - Robot MUST be on the floor (or rollers)
 * - Robot MUST be able to physically rotate
 * - DO NOT run while robot is lifted/blocked
 * - Requires clear space to oscillate ±20-30°
 * 
 * Uses relay feedback method (Ziegler-Nichols variant)
 * to automatically determine optimal PID gains.
 */
public class HeadingAutoTuner {

    private final double relayPower;
    private final double targetAngle;

    private double lastError = 0;
    private double maxError = 0;
    private double minError = 0;

    private boolean outputPositive = true;
    private int zeroCrossings = 0;

    private final ElapsedTime timer = new ElapsedTime();
    private double periodSum = 0;
    private double lastCrossTime = 0;
    private double lastUpdateTime = 0;

    private static final double ZERO_THRESHOLD = 1e-4; // small error tolerance
    private static final double STUCK_TIMEOUT = 2.0; // seconds without zero crossing

    public HeadingAutoTuner(double targetAngleRad, double relayPower) {
        this.targetAngle = targetAngleRad;
        this.relayPower = relayPower;
        timer.reset();
    }

    /**
     * Call repeatedly during tuning. Returns motor turn power.
     * 
     * @param currentAngleRad Current robot heading in radians
     * @return Turn power to apply (relay output: ±relayPower)
     */
    public double update(double currentAngleRad) {
        double error = AngleUnit.normalizeRadians(targetAngle - currentAngleRad);
        double now = timer.seconds();

        maxError = Math.max(maxError, error);
        minError = Math.min(minError, error);

        // Detect zero crossing (error changes sign)
        if (Math.signum(error) != Math.signum(lastError) && Math.abs(lastError) > ZERO_THRESHOLD) {
            zeroCrossings++;

            // Track oscillation period
            if (lastCrossTime > 0) {
                periodSum += (now - lastCrossTime);
            }
            lastCrossTime = now;
            lastUpdateTime = now;

            // 🔥 CRITICAL: Flip relay to create oscillation
            flipOutput();
        }

        lastError = error;

        // Relay output (bang-bang control)
        return outputPositive ? relayPower : -relayPower;
    }

    private void flipOutput() {
        outputPositive = !outputPositive;
    }

    /**
     * Check if tuning is complete.
     * @return true after 8 zero crossings (4 full oscillations)
     */
    public boolean isFinished() {
        return zeroCrossings >= 8; // 4 full oscillations
    }

    /**
     * Check if tuning appears stuck (no progress).
     * Useful for detecting blocked wheels or IMU issues.
     * @return true if no zero crossings detected for STUCK_TIMEOUT seconds
     */
    public boolean isStuck() {
        // Avoid false trigger on slow first oscillation
        double timeSinceStart = timer.seconds();
        return zeroCrossings == 0 && timeSinceStart > STUCK_TIMEOUT;
    }

    public double[] getStats() {
        return new double[]{
                zeroCrossings,
                Math.toDegrees(maxError),
                Math.toDegrees(minError),
                periodSum
        };
    }

    public PIDGains getGains() {
        if (!isFinished()) {
            throw new IllegalStateException("Tuning not finished - call isFinished() first");
        }

        double amplitude = (maxError - minError) / 2.0;
        
        // Safety: Prevent divide-by-zero or insane gains from tiny oscillation
        if (amplitude < Math.toRadians(1.0)) { // < 1° oscillation
            throw new IllegalStateException(
                String.format("Oscillation amplitude too small (%.2f°) - increase RELAY_POWER or check for mechanical binding",
                    Math.toDegrees(amplitude)));
        }
        
        double Ku = (4 * relayPower) / (Math.PI * amplitude);
        double Tu = periodSum / (zeroCrossings / 2.0);

        // Ziegler–Nichols (robust version)
        double kp = 0.6 * Ku;
        double ki = 1.2 * Ku / Tu;
        double kd = 0.075 * Ku * Tu;

        return new PIDGains(kp, ki, kd);
    }

    public static class PIDGains {
        public final double kp, ki, kd;

        public PIDGains(double kp, double ki, double kd) {
            this.kp = kp;
            this.ki = ki;
            this.kd = kd;
        }

        @Override
        public String toString() {
            return String.format("kP=%.3f, kI=%.3f, kD=%.3f", kp, ki, kd);
        }
    }
}
