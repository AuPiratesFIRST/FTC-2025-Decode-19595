package org.firstinspires.ftc.teamcode.SubSystems.Control;

import com.bylazar.configurables.annotations.Configurable;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Configurable
public class HeadingPID {

    // 🔥 Paste your auto-tuned values here (single source of truth)
    public static double kP = 3.2;
    public static double kI = 0.0;
    public static double kD = 0.18;

    private double integral = 0;
    private double lastError = 0;
    private long lastTime = System.nanoTime();

    private static final double MAX_INTEGRAL = 1.0; // prevent windup

    public HeadingPID() {
        // Uses static configurable gains
    }

    public double update(double targetRad, double currentRad) {
        double error = AngleUnit.normalizeRadians(targetRad - currentRad);

        long now = System.nanoTime();
        double dt = (now - lastTime) / 1e9;
        lastTime = now;

        integral += error * dt;
        integral = clamp(integral, -MAX_INTEGRAL, MAX_INTEGRAL);

        double derivative = dt > 0
                ? AngleUnit.normalizeRadians(error - lastError) / dt
                : 0;

        lastError = error;

        return kP * error + kI * integral + kD * derivative;
    }

    public void reset() {
        integral = 0;
        lastError = 0;
        lastTime = System.nanoTime();
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
