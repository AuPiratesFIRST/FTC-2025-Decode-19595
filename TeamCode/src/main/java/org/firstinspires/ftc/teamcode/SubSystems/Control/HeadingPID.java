package org.firstinspires.ftc.teamcode.SubSystems.Control;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class HeadingPID {

    private double kp, ki, kd;
    private double integral = 0;
    private double lastError = 0;
    private long lastTime = System.nanoTime();

    public HeadingPID(double kp, double ki, double kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
    }

    public double update(double targetRad, double currentRad) {
        double error = AngleUnit.normalizeRadians(targetRad - currentRad);

        long now = System.nanoTime();
        double dt = (now - lastTime) / 1e9;
        lastTime = now;

        integral += error * dt;
        double derivative = dt > 0 ? (error - lastError) / dt : 0;
        lastError = error;

        return kp * error + ki * integral + kd * derivative;
    }

    public void reset() {
        integral = 0;
        lastError = 0;
        lastTime = System.nanoTime();
    }
}
