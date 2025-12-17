package org.firstinspires.ftc.teamcode.SubSystems.Shooter;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Shooter subsystem for GoBILDA 6000 RPM motors with 3-inch wheels.
 * FTC-CORRECT flywheel velocity control with voltage-compensated kF.
 */
public class ShooterSubsystem {

    private final DcMotorEx leftShooterMotor;
    private final DcMotorEx rightShooterMotor;
    private final Telemetry telemetry;

    // ================= VOLTAGE =================
    private final VoltageSensor batteryVoltageSensor;
    private static final double NOMINAL_VOLTAGE = 13.0;

    // ================= PIDF =================
    private double kP = 35.0;
    private double kI = 0.0;     // DO NOT USE for flywheel
    private double kD = 12.0;
    private double baseKf = 15.5;

    // ================= MOTOR =================
    private static final double MOTOR_MAX_RPM = 6000.0;
    private static final double TICKS_PER_REV = 28.0;

    // ================= CONTROL =================
    private double targetRPM = 0.0;
    private double rpmTolerance = 100.0;
    private boolean usePercentageTolerance = true;

    private double minPower = 0.3;
    private double maxPower = 1.0;
    private double defaultPower = 0.7;

    public enum ShooterSpeed {
        LOW(0.5),
        MEDIUM(0.7),
        HIGH(0.9),
        MAX(1.0);

        private final double power;
        ShooterSpeed(double power) { this.power = power; }
        public double getPower() { return power; }
    }

    // ===================================================

    public ShooterSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        leftShooterMotor = hardwareMap.get(DcMotorEx.class, "shooterL");
        rightShooterMotor = hardwareMap.get(DcMotorEx.class, "shooterR");

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        configureMotors();
        applyVoltageCompensatedPIDF();
    }

    // ===================================================

    private void configureMotors() {
        leftShooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightShooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        rightShooterMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    // ===================================================
    // 🔥 FTC-CORRECT VOLTAGE COMPENSATION (kF ONLY)
    // ===================================================

    private void applyVoltageCompensatedPIDF() {
        double voltage = batteryVoltageSensor.getVoltage();
        double compensatedKf = baseKf * (NOMINAL_VOLTAGE / voltage);

        PIDFCoefficients pidf =
                new PIDFCoefficients(kP, kI, kD, compensatedKf);

        leftShooterMotor.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        rightShooterMotor.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER, pidf);

        if (telemetry != null) {
            telemetry.addData("Battery Voltage", "%.2f V", voltage);
            telemetry.addData("kF (base)", "%.2f", baseKf);
            telemetry.addData("kF (comp)", "%.2f", compensatedKf);
        }
    }

    // Call this periodically (or every loop)
    public void updateVoltageCompensation() {
        applyVoltageCompensatedPIDF();
    }

    // ===================================================
    // VELOCITY CONTROL
    // ===================================================

    public void setPower(double power) {
        power = Range.clip(power, minPower, maxPower);
        setTargetRPM(MOTOR_MAX_RPM * power);
    }

    public void setTargetRPM(double rpm) {
        targetRPM = Range.clip(rpm, 0.0, MOTOR_MAX_RPM);

        double ticksPerSecond =
                (targetRPM * TICKS_PER_REV) / 60.0;

        leftShooterMotor.setVelocity(ticksPerSecond);
        rightShooterMotor.setVelocity(ticksPerSecond);

        if (telemetry != null) {
            telemetry.addData("Target RPM", "%.0f", targetRPM);
        }
    }

    public void setSpeed(ShooterSpeed speed) {
        setPower(speed.getPower());
    }

    public void start() {
        setPower(defaultPower);
    }

    public void stop() {
        leftShooterMotor.setVelocity(0);
        rightShooterMotor.setVelocity(0);
        targetRPM = 0;
    }

    // ===================================================
    // STATE
    // ===================================================

    public double getCurrentRPM() {
        double leftRPM = leftShooterMotor.getVelocity() * 60.0 / TICKS_PER_REV;
        double rightRPM = rightShooterMotor.getVelocity() * 60.0 / TICKS_PER_REV;
        return (leftRPM + rightRPM) / 2.0;
    }

    public double getTargetRPM() {
        return targetRPM;
    }

    public boolean isAtTargetRPM() {
        double error = Math.abs(getCurrentRPM() - targetRPM);
        return usePercentageTolerance
                ? error <= targetRPM * 0.02
                : error <= rpmTolerance;
    }

    public double getRPMError() {
        return Math.abs(getCurrentRPM() - targetRPM);
    }

    // ===================================================
    // TELEMETRY
    // ===================================================

    public void updateTelemetry() {
        if (telemetry == null) return;

        telemetry.addData("Shooter", isAtTargetRPM() ? "READY ✓" : "SPINNING");
        telemetry.addData("Current RPM", "%.0f", getCurrentRPM());
        telemetry.addData("Target RPM", "%.0f", targetRPM);
        telemetry.addData("RPM Error", "%.0f", getRPMError());
        telemetry.update();
    }
}
