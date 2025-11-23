package org.firstinspires.ftc.teamcode.SubSystems.Shooter;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Shooter subsystem for GoBILDA 6000 RPM motors with 3-inch wheels in direct
 * connection.
 * Provides adjustable speeds and tuning capabilities with closed-loop velocity
 * control
 * to ensure accurate shooting at target RPM.
 */
public class ShooterSubsystem {

    private final DcMotorEx leftShooterMotor;
    private final DcMotorEx rightShooterMotor;
    private final Telemetry telemetry;

    // Motor specifications
    private static final double MOTOR_MAX_RPM = 6000.0; // GoBILDA 6000 RPM motor
    private static final double TICKS_PER_REVOLUTION = 28.0; // GoBILDA 6000 RPM motor encoder ticks
    private static final double WHEEL_DIAMETER = 3.0; // inches
    private static final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER; // inches

    // Velocity control
    private double targetRPM = 0.0; // Current target RPM
    private double rpmTolerance = 100.0; // RPM tolerance for "at target" check (default 100 RPM, ~1.7% at 6000 RPM)
    private boolean useVelocityControl = true; // Use velocity control by default
    private boolean usePercentageTolerance = true; // Use percentage-based tolerance (2% of target)

    // Tuning constants - adjustable for different shot distances
    private double minPower = 0.3; // Minimum power to start spinning (fallback mode)
    private double maxPower = 1.0; // Maximum power
    private double defaultPower = 0.7; // Default shooting power

    // Speed presets for different scenarios
    public enum ShooterSpeed {
        LOW(0.5),
        MEDIUM(0.7),
        HIGH(0.85),
        MAX(1.0);

        private final double power;

        ShooterSpeed(double power) {
            this.power = power;
        }

        public double getPower() {
            return power;
        }
    }

    public ShooterSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        // Initialize motors
        leftShooterMotor = hardwareMap.get(DcMotorEx.class, "shooterL");
        rightShooterMotor = hardwareMap.get(DcMotorEx.class, "shooterR");

        configureMotors();
    }

    private void configureMotors() {
        // Set motor modes for velocity control
        leftShooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightShooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set zero power behavior
        leftShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Reverse one motor if needed for opposite rotation
        // Adjust based on your physical setup
        rightShooterMotor.setDirection(DcMotor.Direction.REVERSE);

        // Configure velocity PIDF coefficients for accurate velocity control
        // These values may need tuning based on your specific motor and load
        // Increased P and F slightly for better response at high RPMs
        PIDFCoefficients velocityPIDF = new PIDFCoefficients(35.0, 0.15, 12.0, 15.0);
        leftShooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, velocityPIDF);
        rightShooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, velocityPIDF);
    }

    /**
     * Set shooter power directly (0.0 to 1.0).
     * 
     * Converts power percentage to target RPM for velocity control.
     * Equation: targetRPM = MOTOR_MAX_RPM × power
     * Example: 0.7 power × 6000 RPM = 4200 RPM target
     * Uses velocity control if enabled for precise RPM, otherwise direct power
     * control.
     * 
     * @param power Motor power (0.0 to 1.0, clipped to minPower-maxPower range)
     */
    public void setPower(double power) {
        power = Range.clip(power, minPower, maxPower);

        if (useVelocityControl) {
            // Convert power percentage to target RPM
            // Formula: targetRPM = MOTOR_MAX_RPM × power
            // Example: 0.7 × 6000 = 4200 RPM
            double targetRPM = MOTOR_MAX_RPM * power;
            setTargetRPM(targetRPM);
        } else {
            // Fallback to direct power control
            leftShooterMotor.setPower(power);
            rightShooterMotor.setPower(power);
            this.targetRPM = MOTOR_MAX_RPM * power;
        }

        if (telemetry != null) {
            telemetry.addData("Shooter Power", "%.2f", power);
            telemetry.addData("Shooter RPM", "%.0f", getCurrentRPM());
            telemetry.addData("Target RPM", "%.0f", targetRPM);
        }
    }

    /**
     * Set target RPM using velocity control for accurate shooting.
     * 
     * Converts RPM to encoder ticks per second for motor velocity control.
     * Equation: ticksPerSecond = (RPM × TICKS_PER_REVOLUTION) / 60
     * This allows precise velocity control using the motor's built-in PIDF
     * controller.
     * 
     * @param rpm Target RPM (0 to MOTOR_MAX_RPM)
     */
    public void setTargetRPM(double rpm) {
        targetRPM = Range.clip(rpm, 0.0, MOTOR_MAX_RPM);

        // Convert RPM to ticks per second
        // Formula: ticksPerSecond = (RPM × ticks/revolution) / 60 seconds
        // Example: 3000 RPM × 28 ticks/rev ÷ 60 = 1400 ticks/second
        double ticksPerSecond = (targetRPM * TICKS_PER_REVOLUTION) / 60.0;

        // Set velocity for both motors
        leftShooterMotor.setVelocity(ticksPerSecond);
        rightShooterMotor.setVelocity(ticksPerSecond);

        if (telemetry != null) {
            telemetry.addData("Target RPM Set", "%.0f", targetRPM);
        }
    }

    /**
     * Set shooter to a preset speed
     * 
     * @param speed ShooterSpeed preset
     */
    public void setSpeed(ShooterSpeed speed) {
        setPower(speed.getPower());
    }

    /**
     * Start shooter at default power
     */
    public void start() {
        setPower(defaultPower);
    }

    /**
     * Check if shooter is at target RPM (within tolerance).
     * 
     * Uses either fixed RPM tolerance or percentage-based tolerance (2% of target).
     * Percentage tolerance is more accurate for high RPMs.
     * 
     * @return True if current RPM is within tolerance of target RPM
     */
    public boolean isAtTargetRPM() {
        if (targetRPM == 0)
            return false;

        double currentRPM = getCurrentRPM();
        double error = Math.abs(currentRPM - targetRPM);

        if (usePercentageTolerance) {
            // Use percentage-based tolerance (2% of target RPM)
            // More accurate for high RPMs (e.g., 2% of 6000 RPM = 120 RPM tolerance)
            double percentageTolerance = targetRPM * 0.02; // 2% tolerance
            return error <= percentageTolerance;
        } else {
            // Use fixed RPM tolerance
            return error <= rpmTolerance;
        }
    }

    /**
     * Get the current RPM error (how far from target).
     * 
     * @return Absolute difference between current and target RPM
     */
    public double getRPMError() {
        return Math.abs(getCurrentRPM() - targetRPM);
    }

    /**
     * Get the current RPM error as a percentage of target.
     * 
     * @return Percentage error (0-100)
     */
    public double getRPMErrorPercentage() {
        if (targetRPM == 0)
            return 100.0;
        return (getRPMError() / targetRPM) * 100.0;
    }

    /**
     * Wait for shooter to reach target RPM
     * 
     * @param timeoutMs Maximum time to wait in milliseconds
     * @return True if target RPM was reached, false if timeout
     */
    public boolean waitForTargetRPM(long timeoutMs) {
        long startTime = System.currentTimeMillis();

        while (!isAtTargetRPM() && (System.currentTimeMillis() - startTime) < timeoutMs) {
            // Update velocity control
            if (useVelocityControl) {
                double ticksPerSecond = (targetRPM * TICKS_PER_REVOLUTION) / 60.0;
                leftShooterMotor.setVelocity(ticksPerSecond);
                rightShooterMotor.setVelocity(ticksPerSecond);
            }

            if (telemetry != null) {
                telemetry.addData("Waiting for RPM", "Current: %.0f, Target: %.0f",
                        getCurrentRPM(), targetRPM);
                telemetry.update();
            }

            try {
                Thread.sleep(10); // Small delay to avoid busy waiting
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                return false;
            }
        }

        return isAtTargetRPM();
    }

    /**
     * Get target RPM
     * 
     * @return Current target RPM
     */
    public double getTargetRPM() {
        return targetRPM;
    }

    /**
     * Set RPM tolerance for "at target" checking.
     * 
     * @param tolerance RPM tolerance (default 100 RPM)
     */
    public void setRPMTolerance(double tolerance) {
        this.rpmTolerance = Math.abs(tolerance);
    }

    /**
     * Enable or disable percentage-based tolerance.
     * When enabled, tolerance is 2% of target RPM (more accurate for high RPMs).
     * 
     * @param enabled True to use percentage tolerance, false for fixed tolerance
     */
    public void setPercentageTolerance(boolean enabled) {
        this.usePercentageTolerance = enabled;
    }

    /**
     * Get current RPM tolerance
     * 
     * @return RPM tolerance
     */
    public double getRPMTolerance() {
        return rpmTolerance;
    }

    /**
     * Enable or disable velocity control
     * 
     * @param enabled True to use velocity control, false for power control
     */
    public void setVelocityControlEnabled(boolean enabled) {
        this.useVelocityControl = enabled;

        if (!enabled) {
            // Switch to power mode if disabling velocity control
            leftShooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightShooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } else {
            // Switch back to encoder mode for velocity control
            leftShooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightShooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     * Check if velocity control is enabled
     * 
     * @return True if velocity control is enabled
     */
    public boolean isVelocityControlEnabled() {
        return useVelocityControl;
    }

    /**
     * Set velocity PIDF coefficients for tuning
     * 
     * @param p Proportional gain
     * @param i Integral gain
     * @param d Derivative gain
     * @param f Feedforward gain
     */
    public void setVelocityPIDF(double p, double i, double d, double f) {
        PIDFCoefficients velocityPIDF = new PIDFCoefficients(p, i, d, f);
        leftShooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, velocityPIDF);
        rightShooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, velocityPIDF);
    }

    /**
     * Stop shooter
     */
    public void stop() {
        if (useVelocityControl) {
            leftShooterMotor.setVelocity(0);
            rightShooterMotor.setVelocity(0);
        } else {
            leftShooterMotor.setPower(0);
            rightShooterMotor.setPower(0);
        }
        targetRPM = 0;
    }

    /**
     * Get current RPM based on motor velocity.
     * 
     * Converts motor velocity (ticks per second) back to RPM.
     * Equation: RPM = (ticksPerSecond × 60) / TICKS_PER_REVOLUTION
     * Averages both motors for more accurate reading.
     * 
     * @return Current RPM (average of left and right motors)
     */
    public double getCurrentRPM() {
        // Average of both motors for accuracy
        // getVelocity() returns ticks per second, convert to RPM
        // Formula: RPM = (ticks/second × 60) / (ticks/revolution)
        // Example: 1400 ticks/s × 60 ÷ 28 ticks/rev = 3000 RPM
        double leftVelocity = leftShooterMotor.getVelocity();
        double rightVelocity = rightShooterMotor.getVelocity();
        double leftRPM = (leftVelocity * 60.0) / TICKS_PER_REVOLUTION;
        double rightRPM = (rightVelocity * 60.0) / TICKS_PER_REVOLUTION;
        return (leftRPM + rightRPM) / 2.0;
    }

    /**
     * Get target RPM for a given power
     * 
     * @param power Motor power (0.0 to 1.0)
     * @return Target RPM
     */
    public double getTargetRPM(double power) {
        return MOTOR_MAX_RPM * power;
    }

    /**
     * Set minimum power threshold
     * 
     * @param minPower Minimum power (0.0 to 1.0)
     */
    public void setMinPower(double minPower) {
        this.minPower = Range.clip(minPower, 0.0, 1.0);
    }

    /**
     * Set maximum power threshold
     * 
     * @param maxPower Maximum power (0.0 to 1.0)
     */
    public void setMaxPower(double maxPower) {
        this.maxPower = Range.clip(maxPower, 0.0, 1.0);
    }

    /**
     * Set default power
     * 
     * @param defaultPower Default power (0.0 to 1.0)
     */
    public void setDefaultPower(double defaultPower) {
        this.defaultPower = Range.clip(defaultPower, 0.0, 1.0);
    }

    /**
     * Get current power setting
     * 
     * @return Current power
     */
    public double getCurrentPower() {
        return leftShooterMotor.getPower();
    }

    /**
     * Update telemetry with shooter information
     */
    public void updateTelemetry() {
        if (telemetry != null) {
            double currentRPM = getCurrentRPM();
            double rpmError = getRPMError();
            double errorPercentage = getRPMErrorPercentage();
            boolean atTarget = isAtTargetRPM();

            // Status with clear indication
            String status = atTarget ? "READY ✓" : "SPINNING UP...";
            telemetry.addData("Shooter Status", status);
            telemetry.addData("Velocity Control", useVelocityControl ? "ENABLED" : "DISABLED");

            // RPM information
            telemetry.addData("Current RPM", "%.0f", currentRPM);
            telemetry.addData("Target RPM", "%.0f", targetRPM);
            telemetry.addData("RPM Error", "%.0f RPM (%.1f%%)", rpmError, errorPercentage);

            // At target indicator
            if (atTarget) {
                telemetry.addData("At Target", "YES ✓");
            } else {
                // Show how close it is
                if (errorPercentage < 5.0) {
                    telemetry.addData("At Target", "CLOSE (%.1f%% off)", errorPercentage);
                } else if (errorPercentage < 10.0) {
                    telemetry.addData("At Target", "GETTING CLOSE (%.1f%% off)", errorPercentage);
                } else {
                    telemetry.addData("At Target", "NO (%.1f%% off)", errorPercentage);
                }
            }

            if (useVelocityControl) {
                telemetry.addData("Left Velocity", "%.1f ticks/s", leftShooterMotor.getVelocity());
                telemetry.addData("Right Velocity", "%.1f ticks/s", rightShooterMotor.getVelocity());
            } else {
                telemetry.addData("Left Motor Power", "%.2f", leftShooterMotor.getPower());
                telemetry.addData("Right Motor Power", "%.2f", rightShooterMotor.getPower());
            }

            telemetry.addData("Wheel Speed", "%.1f in/s", currentRPM * WHEEL_CIRCUMFERENCE / 60.0);

            // Show tolerance being used
            if (usePercentageTolerance && targetRPM > 0) {
                double tolerance = targetRPM * 0.02;
                telemetry.addData("Tolerance", "%.0f RPM (2%% of target)", tolerance);
            } else {
                telemetry.addData("Tolerance", "%.0f RPM (fixed)", rpmTolerance);
            }
        }
    }
}
