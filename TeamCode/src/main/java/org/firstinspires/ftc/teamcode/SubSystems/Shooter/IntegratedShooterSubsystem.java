package org.firstinspires.ftc.teamcode.SubSystems.Shooter;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SubSystems.Drive.TileCoordinate;
import org.firstinspires.ftc.teamcode.Constants.FieldConstants;

/**
 * Integrated Shooter Subsystem for DECODE 2025-2026.
 * Features:
 * - Voltage-compensated Feedforward (kF)
 * - Linear Interpolation for distance-based RPM
 * - Alliance-aware goal targeting
 */
@Configurable
public class IntegratedShooterSubsystem {

    private final DcMotorEx leftShooterMotor;
    private final DcMotorEx rightShooterMotor;
    private final VoltageSensor batteryVoltageSensor;
    private final Telemetry telemetry;

    // ================= MOTOR & BALLISTICS CONSTANTS =================
    private static final double MOTOR_MAX_RPM = 6000.0;
    private static final double TICKS_PER_REV = 28.0;
    private static final double NOMINAL_VOLTAGE = 13.0;
    private static final double FIXED_LAUNCH_ANGLE = 39.2; // Calculated from your CAD

    // ================= PIDF TUNING =================
    public static double kP = 35.0;
    public static double kI = 0.0;
    public static double kD = 12.0;
    public static double baseKf = 13.2; // Base feedforward at 13V

    // ================= SHOT LOOKUP TABLE =================
    // Format: { distance_inches, target_RPM }
    // Tuned for 39.2 degree angle and 5.257" high-compression gap
    private static final double[][] SHOT_PROFILE = new double[][]{
            {40,  3200}, // Close (Goal Triangle)
            {72,  3800}, // Mid
            {100, 4400}, // Front of Small Triangle
            {125, 4850}, // Middle of Small Triangle
            {140, 5200}  // Back of Small Triangle
    };

    private double targetRPM = 0.0;

    public IntegratedShooterSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        leftShooterMotor = hardwareMap.get(DcMotorEx.class, "shooterL");
        rightShooterMotor = hardwareMap.get(DcMotorEx.class, "shooterR");
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        configureMotors();
        updateVoltageCompensation();
    }

    private void configureMotors() {
        leftShooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightShooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightShooterMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    /**
     * Updates kF based on current battery voltage to ensure consistent shots.
     * Call this in your main loop.
     */
    public void updateVoltageCompensation() {
        double voltage = batteryVoltageSensor.getVoltage();
        // Prevent division by zero if sensor glitches
        double currentVoltage = Math.max(voltage, 1.0);

        double compensatedKf = baseKf * (NOMINAL_VOLTAGE / currentVoltage);

        PIDFCoefficients pidf = new PIDFCoefficients(kP, kI, kD, compensatedKf);
        leftShooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        rightShooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
    }

    /**
     * Automated shooting logic:
     * 1. Determines distance to correct goal
     * 2. Interpolates RPM from SHOT_PROFILE
     * 3. Commands the motors
     */
    public void autoAimAndSpin(TileCoordinate robotPosition, boolean isBlueAlliance) {
        // Get correct goal coordinates from constants
        double goalX = isBlueAlliance ? FieldConstants.GOAL_X_BLUE : FieldConstants.GOAL_X_RED;
        double goalY = isBlueAlliance ? FieldConstants.GOAL_Y_BLUE : FieldConstants.GOAL_Y_RED;
        TileCoordinate goal = new TileCoordinate(goalX, goalY);

        double distance = robotPosition.distanceTo(goal);
        double neededRPM = interpolateRPM(distance);

        setTargetRPM(neededRPM);

        if (telemetry != null) {
            telemetry.addData("Dist to Goal", "%.1f in", distance);
            telemetry.addData("Shooter Status", isAtTargetRPM() ? "READY" : "WARMING");
        }
    }

    /**
     * Performs linear interpolation between data points in the SHOT_PROFILE table.
     */
    private double interpolateRPM(double distance) {
        if (distance <= SHOT_PROFILE[0][0]) return SHOT_PROFILE[0][1];
        if (distance >= SHOT_PROFILE[SHOT_PROFILE.length - 1][0]) return SHOT_PROFILE[SHOT_PROFILE.length - 1][1];

        for (int i = 0; i < SHOT_PROFILE.length - 1; i++) {
            if (distance < SHOT_PROFILE[i + 1][0]) {
                double d0 = SHOT_PROFILE[i][0];
                double d1 = SHOT_PROFILE[i + 1][0];
                double r0 = SHOT_PROFILE[i][1];
                double r1 = SHOT_PROFILE[i + 1][1];

                // Linear Interpolation formula: y = y0 + (x - x0) * ((y1 - y0) / (x1 - x0))
                return r0 + (distance - d0) * ((r1 - r0) / (d1 - d0));
            }
        }
        return SHOT_PROFILE[SHOT_PROFILE.length - 1][1];
    }

    public void setTargetRPM(double rpm) {
        targetRPM = Range.clip(rpm, 0.0, MOTOR_MAX_RPM);
        double ticksPerSecond = (targetRPM * TICKS_PER_REV) / 60.0;

        leftShooterMotor.setVelocity(ticksPerSecond);
        rightShooterMotor.setVelocity(ticksPerSecond);
    }

    public void stop() {
        setTargetRPM(0);
    }

    public double getCurrentRPM() {
        return (leftShooterMotor.getVelocity() + rightShooterMotor.getVelocity()) / 2.0 * 60.0 / TICKS_PER_REV;
    }

    public boolean isAtTargetRPM() {
        if (targetRPM < 100) return true;
        return Math.abs(getCurrentRPM() - targetRPM) < (targetRPM * 0.03); // 3% tolerance
    }

    public void updateTelemetry() {
        telemetry.addData("Shooter RPM", "Target: %.0f, Actual: %.0f", targetRPM, getCurrentRPM());
        telemetry.addData("Voltage", "%.2fV", batteryVoltageSensor.getVoltage());
    }
}