package org.firstinspires.ftc.teamcode.SubSystems.Control;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Relay-based PID auto-tuner for heading control.
 * 
 * Uses bang-bang relay method to induce controlled oscillations,
 * then applies Ziegler-Nichols rules to compute optimal PID gains.
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

    public HeadingAutoTuner(double targetAngleRad, double relayPower) {
        this.targetAngle = targetAngleRad;
        this.relayPower = relayPower;
        timer.reset();
    }

    /**
     * Call repeatedly during tuning.
     * Returns motor turn power.
     */
    public double update(double currentAngleRad) {
        double error = AngleUnit.normalizeRadians(targetAngle - currentAngleRad);

        maxError = Math.max(maxError, error);
        minError = Math.min(minError, error);

        // Detect zero crossing
        if (Math.signum(error) != Math.signum(lastError) && lastError != 0) {
            zeroCrossings++;

            double now = timer.seconds();
            if (lastCrossTime > 0) {
                periodSum += (now - lastCrossTime);
            }
            lastCrossTime = now;
        }

        lastError = error;

        // Relay output
        return outputPositive ? relayPower : -relayPower;
    }

    public void flipOutput() {
        outputPositive = !outputPositive;
    }

    public boolean isFinished() {
        return zeroCrossings >= 8; // 4 full oscillations
    }

    /**
     * Get tuning statistics for telemetry display.
     * 
     * @return Array: [zeroCrossings, maxError (degrees), minError (degrees), periodSum]
     */
    public double[] getStats() {
        return new double[] {
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
