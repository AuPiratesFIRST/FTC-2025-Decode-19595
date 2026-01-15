package org.firstinspires.ftc.teamcode.SubSystems.Shooter;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SubSystems.Drive.TileCoordinate;
import org.firstinspires.ftc.teamcode.Constants.FieldConstants;

import java.util.HashMap;
import java.util.Map;

/**
 * Tile-first shooter system for DECODE 2025–2026
 *
 * PRIORITY ORDER:
 * 1) Tile position (C1/D1 hard lock)
 * 2) Per-tile tuning (C4/D4)
 * 3) Distance interpolation (triangle only)
 * 4) Heading-based trim
 * 5) Vision (optional blend only)
 *
 * No hard AprilTag dependency.
 */
@Configurable
public class IntegratedShooterSubsystem {

    /* ================= HARDWARE ================= */

    private final DcMotorEx leftMotor;
    private final DcMotorEx rightMotor;
    private final VoltageSensor voltageSensor;
    private final Telemetry telemetry;

    /* ================= CONSTANTS ================= */

    private static final double TICKS_PER_REV = 28.0;
    private static final double MAX_RPM = 6000;
    private static final double NOMINAL_VOLTAGE = 13.0;

    /* Heading trim */
    private static final double MAX_TRIM_RPM = 50.0;
    private static final double TRIM_FULL_SCALE_RAD = Math.toRadians(8);

    /* Shot detect */
    private static final double RPM_DIP_THRESHOLD = 180;

    /* ================= PIDF ================= */

    public static double kP = 35;
    public static double kI = 0;
    public static double kD = 12;
    public static double baseKf = 13.2;

    /* ================= TILE RPM MAP ================= */

    private static final Map<String, Double> TILE_RPM = new HashMap<>();

    static {
        TILE_RPM.put("C1", 5225.0);
        TILE_RPM.put("D1", 5225.0);

        TILE_RPM.put("C4", 4400.0);
        TILE_RPM.put("D4", 4400.0);
    }

    /* ================= TRIANGLE ================= */

    private static final TileCoordinate TRI_A6 =
            new TileCoordinate(0, 6, 12, 12);

    private static final TileCoordinate TRI_F6 =
            new TileCoordinate(5, 6, 12, 12);

    private static final TileCoordinate TRI_X3 =
            new TileCoordinate('X', 3);

    /* ================= STATE ================= */

    private double targetRPM = 0;
    private double lastRPM = 0;

    /* ================= INIT ================= */

    public IntegratedShooterSubsystem(HardwareMap hw, Telemetry telemetry) {
        this.telemetry = telemetry;

        leftMotor = hw.get(DcMotorEx.class, "shooterL");
        rightMotor = hw.get(DcMotorEx.class, "shooterR");
        voltageSensor = hw.voltageSensor.iterator().next();

        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        updateVoltageCompensation();
    }

    /* ================= PUBLIC API ================= */

    /**
     * MAIN ENTRY POINT
     */
    public void aimAndSpin(
            TileCoordinate robotPos,
            double robotHeading,
            boolean isBlueAlliance,
            Double visionRPM,
            Double visionConfidence
    ) {
        updateVoltageCompensation();

        double tileRPM = getTileRPM(robotPos);
        double tileConfidence = tileRPM > 0 ? 1.0 : 0.0;

        double finalRPM = tileRPM;

        // Triangle distance shot
        if (tileRPM <= 0 && insideTriangle(robotPos)) {
            double goalX = isBlueAlliance
                    ? FieldConstants.GOAL_X_BLUE
                    : FieldConstants.GOAL_X_RED;

            double goalY = isBlueAlliance
                    ? FieldConstants.GOAL_Y_BLUE
                    : FieldConstants.GOAL_Y_RED;

            TileCoordinate goal = new TileCoordinate(goalX, goalY);
            double dist = robotPos.distanceTo(goal);
            finalRPM = interpolateRPM(dist);
            tileConfidence = 0.6;
        }

        // Vision blend (optional)
        if (visionRPM != null && visionConfidence != null) {
            finalRPM = blend(finalRPM, tileConfidence, visionRPM, visionConfidence);
        }

        // Heading trim
        double goalAngle = robotPos.angleTo(
                new TileCoordinate(
                        isBlueAlliance ? FieldConstants.GOAL_X_BLUE : FieldConstants.GOAL_X_RED,
                        isBlueAlliance ? FieldConstants.GOAL_Y_BLUE : FieldConstants.GOAL_Y_RED
                )
        );

        finalRPM += headingTrim(robotHeading, goalAngle);

        setTargetRPM(finalRPM);

        telemetry.addData("Tile", robotPos.getTilePosition());
        telemetry.addData("Target RPM", "%.0f", finalRPM);
    }

    /* ================= TILE LOGIC ================= */

    private double getTileRPM(TileCoordinate pos) {
        String tile = pos.getTilePosition();
        return TILE_RPM.getOrDefault(tile, -1.0);
    }

    /* ================= TRIANGLE CHECK ================= */

    private boolean insideTriangle(TileCoordinate p) {
        return sameSide(p, TRI_A6, TRI_F6, TRI_X3)
                && sameSide(p, TRI_F6, TRI_X3, TRI_A6)
                && sameSide(p, TRI_X3, TRI_A6, TRI_F6);
    }

    private boolean sameSide(
            TileCoordinate p1,
            TileCoordinate p2,
            TileCoordinate a,
            TileCoordinate b
    ) {
        double cp1 = cross(b, a, p1);
        double cp2 = cross(b, a, p2);
        return cp1 * cp2 >= 0;
    }

    private double cross(TileCoordinate a, TileCoordinate b, TileCoordinate c) {
        return (b.getX() - a.getX()) * (c.getY() - a.getY()) -
                (b.getY() - a.getY()) * (c.getX() - a.getX());
    }

    /* ================= RPM LOGIC ================= */

    private double interpolateRPM(double dist) {
        if (dist < 60) return 3600;
        if (dist < 90) return 4200;
        if (dist < 120) return 4700;
        return 5200;
    }

    private double headingTrim(double heading, double goalAngle) {
        double err = goalAngle - heading;

        while (err > Math.PI) err -= 2 * Math.PI;
        while (err < -Math.PI) err += 2 * Math.PI;

        return Range.clip(err / TRIM_FULL_SCALE_RAD, -1, 1) * MAX_TRIM_RPM;
    }

    private double blend(double a, double ac, double b, double bc) {
        return (a * ac + b * bc) / (ac + bc);
    }

    /* ================= MOTOR ================= */

    public void setTargetRPM(double rpm) {
        targetRPM = Range.clip(rpm, 0, MAX_RPM);
        double ticksPerSec = (targetRPM * TICKS_PER_REV) / 60.0;

        leftMotor.setVelocity(ticksPerSec);
        rightMotor.setVelocity(ticksPerSec);
    }

    public double getCurrentRPM() {
        return (leftMotor.getVelocity() + rightMotor.getVelocity())
                * 30 / TICKS_PER_REV;
    }

    public boolean isAtSpeed() {
        return Math.abs(getCurrentRPM() - targetRPM) < targetRPM * 0.03;
    }

    /* ================= SHOT CONFIRM ================= */

    public boolean shotDetected() {
        double rpm = getCurrentRPM();
        boolean dip = lastRPM - rpm > RPM_DIP_THRESHOLD;
        lastRPM = rpm;
        return dip;
    }

    /* ================= VOLTAGE ================= */

    private void updateVoltageCompensation() {
        double v = Math.max(voltageSensor.getVoltage(), 1);
        double kF = baseKf * (NOMINAL_VOLTAGE / v);

        PIDFCoefficients pidf =
                new PIDFCoefficients(kP, kI, kD, kF);

        leftMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        rightMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
    }
}