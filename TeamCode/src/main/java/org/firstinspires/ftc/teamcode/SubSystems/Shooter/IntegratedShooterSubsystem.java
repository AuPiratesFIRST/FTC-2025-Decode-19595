package org.firstinspires.ftc.teamcode.SubSystems.Shooter;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.SubSystems.Drive.TileCoordinate;
import org.firstinspires.ftc.teamcode.Constants.FieldConstants;

/**
 * Integrated shooter subsystem combining velocity control with distance-based
 * shot profile.
 * 
 * Features:
 * - Velocity control with RPM targeting (from ShooterSubsystem)
 * - Distance-based power calculation using shot profile (from
 * Scoring/ShooterSubsystem)
 * - Integration with AprilTag localization for automatic distance calculation
 * - Optional launch angle servo control
 * 
 * Shot profile entries: {distance_inches, motor_power_0to1, servo_angle_0to1}
 * Power is converted to RPM using: targetRPM = MOTOR_MAX_RPM × power
 */
public class IntegratedShooterSubsystem {

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
    private double rpmTolerance = 100.0; // RPM tolerance for "at target" check
    private boolean useVelocityControl = true; // Use velocity control by default
    private boolean usePercentageTolerance = true; // Use percentage-based tolerance (2% of target)

    // Shot profile entries: {distance_inches, motor_power_0to1}
    // Calibrated for DECODE field - adjust based on testing
    // Fixed-angle shooter: angle is approximately 45-65 degrees (less than 90°)
    // Tested: 88 inches at ~5000 RPM (0.83 power) scores successfully
    // Profile covers closer distances with lower power
    private static final double[][] SHOT_PROFILE = new double[][] {
            { 24, 0.45 }, // Close range
            { 36, 0.60 }, // Medium-close
            { 48, 0.70 }, // Medium
            { 60, 0.80 }, // Medium-far
            { 72, 0.85 }, // Far
            { 88, 0.83 }, // Very far (tested distance - ~5000 RPM)
            { 100, 1.0 } // Maximum range
    };

    // Fixed launch angle (approximately 45-65 degrees, less than 90°)
    // This is a physical constant of the shooter mechanism
    private static final double FIXED_LAUNCH_ANGLE_DEGREES = 55.0; // Estimated from 88" @ 5000 RPM

    // Physics constants for simulation
    private static final double GRAVITY_IN_PER_S2 = 386.088; // Gravity in inches per second squared (32.174 ft/s²)
    private static final double SHOOTER_HEIGHT_INCHES = 9.375; // Shooter height from ground (camera height)

    // Use centralized constants
    public static final double GOAL_HEIGHT_INCHES = FieldConstants.GOAL_HEIGHT_INCHES;

    public IntegratedShooterSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
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
        rightShooterMotor.setDirection(DcMotor.Direction.REVERSE);

        // Configure velocity PIDF coefficients for accurate velocity control
        PIDFCoefficients velocityPIDF = new PIDFCoefficients(35.0, 0.15, 12.0, 15.0);
        leftShooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, velocityPIDF);
        rightShooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, velocityPIDF);
    }

    /**
     * Shoot artifact using distance-based shot profile.
     * Calculates distance from robot to goal, then uses shot profile to determine
     * power.
     * Uses fixed launch angle (approximately 45-65 degrees).
     * 
     * @param robotPosition  Robot's current position (from AprilTag localization)
     * @param isBlueAlliance True for blue alliance, false for red
     */
    public void shootArtifact(TileCoordinate robotPosition, boolean isBlueAlliance) {
        TileCoordinate goal = getGoalPosition(isBlueAlliance);
        double horizontalDistance = robotPosition.distanceTo(goal);

        double motorPower = calculateMotorPower(horizontalDistance, GOAL_HEIGHT_INCHES);

        // Set power using velocity control
        setPower(motorPower);

        if (telemetry != null) {
            telemetry.addData("Shooter Distance", "%.1f in", horizontalDistance);
            telemetry.addData("Shooter Power", "%.2f", motorPower);
            telemetry.addData("Target RPM", "%.0f", targetRPM);
            telemetry.addData("Fixed Launch Angle", "%.1f°", FIXED_LAUNCH_ANGLE_DEGREES);

            // Optional: Uncomment to see physics-based comparison
            updateSimulationTelemetry(horizontalDistance, GOAL_HEIGHT_INCHES);
        }
    }

    /**
     * Get goal position based on alliance
     * 
     * @param isBlueAlliance True for blue alliance, false for red
     * @return Goal position as TileCoordinate
     */
    public static TileCoordinate getGoalPosition(boolean isBlueAlliance) {
        return isBlueAlliance
                ? new TileCoordinate(FieldConstants.GOAL_X_BLUE, FieldConstants.GOAL_Y_BLUE)
                : new TileCoordinate(FieldConstants.GOAL_X_RED, FieldConstants.GOAL_Y_RED);
    }

    /**
     * Calculate motor power based on distance to goal using shot profile
     * 
     * @param horizontalDistanceInches Horizontal distance to goal in inches
     * @param goalHeightInches         Goal height in inches (for reference, not
     *                                 used in calculation)
     * @return Motor power (0.0 to 1.0)
     */
    public double calculateMotorPower(double horizontalDistanceInches, double goalHeightInches) {
        return interpolateProfile(horizontalDistanceInches, /* columnIndex */1);
    }

    /**
     * Get the fixed launch angle of the shooter
     * 
     * @return Fixed launch angle in degrees (approximately 45-65 degrees)
     */
    public static double getFixedLaunchAngle() {
        return FIXED_LAUNCH_ANGLE_DEGREES;
    }

    /**
     * SIMULATION: Calculate required power using physics-based projectile motion.
     * 
     * Uses projectile motion equations to calculate the initial velocity needed
     * to hit the goal at a given distance, then converts to motor power.
     * 
     * Projectile motion equations:
     * - Horizontal: x = v₀ * cos(θ) * t
     * - Vertical: y = v₀ * sin(θ) * t - 0.5 * g * t²
     * 
     * Solving for v₀:
     * v₀ = sqrt(0.5 * g * x² / ((x * tan(θ) - Δy) * cos²(θ)))
     * 
     * Where:
     * - x = horizontal distance to goal
     * - Δy = goal height - shooter height
     * - θ = launch angle
     * - g = gravity
     * 
     * @param horizontalDistanceInches Horizontal distance to goal in inches
     * @param goalHeightInches         Goal height from floor in inches
     * @return Calculated motor power (0.0 to 1.0) based on physics, or -1 if
     *         calculation fails
     */
    public double calculatePowerFromPhysics(double horizontalDistanceInches, double goalHeightInches) {
        if (horizontalDistanceInches <= 0) {
            return -1.0; // Invalid distance
        }

        // Convert angle to radians
        double angleRad = Math.toRadians(FIXED_LAUNCH_ANGLE_DEGREES);
        double cosAngle = Math.cos(angleRad);
        double sinAngle = Math.sin(angleRad);
        double tanAngle = Math.tan(angleRad);

        // Calculate vertical displacement (goal height - shooter height)
        double deltaY = goalHeightInches - SHOOTER_HEIGHT_INCHES;

        // Calculate required initial velocity using projectile motion
        // v₀² = 0.5 * g * x² / ((x * tan(θ) - Δy) * cos²(θ))
        double denominator = (horizontalDistanceInches * tanAngle - deltaY) * cosAngle * cosAngle;

        if (denominator <= 0) {
            return -1.0; // Invalid trajectory (angle too low or distance too far)
        }

        double velocitySquared = 0.5 * GRAVITY_IN_PER_S2 * horizontalDistanceInches * horizontalDistanceInches
                / denominator;

        if (velocitySquared < 0) {
            return -1.0; // Invalid calculation
        }

        double velocityInchesPerSecond = Math.sqrt(velocitySquared);

        // Convert velocity to wheel RPM
        // Wheel circumference = π * diameter
        // RPM = (velocity in/s) / (circumference in) * 60 seconds/minute
        double wheelRPM = (velocityInchesPerSecond / WHEEL_CIRCUMFERENCE) * 60.0;

        // Convert RPM to motor power (assuming linear relationship)
        // Power = RPM / MAX_RPM
        double calculatedPower = wheelRPM / MOTOR_MAX_RPM;

        // Clamp to valid range
        return Range.clip(calculatedPower, 0.0, 1.0);
    }

    /**
     * SIMULATION: Compare distance-based vs physics-based power calculation.
     * 
     * This method compares the two approaches and returns comparison data.
     * The distance-based approach is still used for actual shooting.
     * 
     * @param horizontalDistanceInches Horizontal distance to goal in inches
     * @param goalHeightInches         Goal height from floor in inches
     * @return Array with [distanceBasedPower, physicsBasedPower, difference,
     *         physicsRPM]
     */
    public double[] compareShotMethods(double horizontalDistanceInches, double goalHeightInches) {
        double distanceBasedPower = calculateMotorPower(horizontalDistanceInches, goalHeightInches);
        double physicsBasedPower = calculatePowerFromPhysics(horizontalDistanceInches, goalHeightInches);

        double distanceBasedRPM = distanceBasedPower * MOTOR_MAX_RPM;
        double physicsBasedRPM = physicsBasedPower >= 0 ? physicsBasedPower * MOTOR_MAX_RPM : -1.0;

        double difference = physicsBasedPower >= 0 ? physicsBasedPower - distanceBasedPower : Double.NaN;

        return new double[] {
                distanceBasedPower,
                physicsBasedPower,
                difference,
                physicsBasedRPM
        };
    }

    /**
     * SIMULATION: Update telemetry with comparison between distance-based and
     * physics-based calculations.
     * 
     * Shows both approaches side-by-side for analysis.
     * 
     * @param horizontalDistanceInches Horizontal distance to goal in inches
     * @param goalHeightInches         Goal height from floor in inches
     */
    public void updateSimulationTelemetry(double horizontalDistanceInches, double goalHeightInches) {
        if (telemetry == null)
            return;

        double[] comparison = compareShotMethods(horizontalDistanceInches, goalHeightInches);
        double distancePower = comparison[0];
        double physicsPower = comparison[1];
        double difference = comparison[2];
        double physicsRPM = comparison[3];

        telemetry.addLine("=== SHOT PROFILE SIMULATION ===");
        telemetry.addData("Distance", "%.1f in", horizontalDistanceInches);
        telemetry.addLine();

        // Distance-based (current method)
        telemetry.addData("=== DISTANCE-BASED (ACTIVE) ===", "");
        telemetry.addData("Power", "%.3f", distancePower);
        telemetry.addData("RPM", "%.0f", distancePower * MOTOR_MAX_RPM);
        telemetry.addLine();

        // Physics-based (simulation)
        telemetry.addData("=== PHYSICS-BASED (SIMULATION) ===", "");
        if (physicsPower >= 0) {
            telemetry.addData("Power", "%.3f", physicsPower);
            telemetry.addData("RPM", "%.0f", physicsRPM);
            telemetry.addData("Difference", "%.3f (%.1f%%)", difference, difference * 100.0);

            // Show which is higher
            if (Math.abs(difference) < 0.01) {
                telemetry.addData("Comparison", "SIMILAR");
            } else if (difference > 0) {
                telemetry.addData("Comparison", "Physics needs MORE power");
            } else {
                telemetry.addData("Comparison", "Physics needs LESS power");
            }
        } else {
            telemetry.addData("Status", "INVALID (trajectory impossible)");
        }
        telemetry.addLine();

        // Physics constants used
        telemetry.addData("Launch Angle", "%.1f°", FIXED_LAUNCH_ANGLE_DEGREES);
        telemetry.addData("Goal Height", "%.1f in", goalHeightInches);
        telemetry.addData("Shooter Height", "%.1f in", SHOOTER_HEIGHT_INCHES);
        telemetry.addData("Vertical Δ", "%.1f in", goalHeightInches - SHOOTER_HEIGHT_INCHES);
    }

    /**
     * Linear interpolation between shot profile entries.
     * 
     * Interpolates motor power based on distance to goal.
     * Uses linear interpolation formula: value = v0 + t × (v1 - v0)
     * where t = (distance - d0) / (d1 - d0) is the interpolation factor [0, 1]
     * 
     * @param distanceInches Distance to goal in inches
     * @param columnIndex    Column index in SHOT_PROFILE (1 = power)
     * @return Interpolated motor power (0.0 to 1.0)
     */
    private double interpolateProfile(double distanceInches, int columnIndex) {
        if (SHOT_PROFILE.length == 0)
            return 0.0;

        // Clamp to bounds: use first or last entry if outside range
        if (distanceInches <= SHOT_PROFILE[0][0])
            return SHOT_PROFILE[0][columnIndex];
        if (distanceInches >= SHOT_PROFILE[SHOT_PROFILE.length - 1][0])
            return SHOT_PROFILE[SHOT_PROFILE.length - 1][columnIndex];

        // Find the two profile entries that bracket the distance
        for (int i = 0; i < SHOT_PROFILE.length - 1; i++) {
            double d0 = SHOT_PROFILE[i][0]; // Lower distance
            double d1 = SHOT_PROFILE[i + 1][0]; // Upper distance

            if (distanceInches >= d0 && distanceInches <= d1) {
                double v0 = SHOT_PROFILE[i][columnIndex]; // Lower value
                double v1 = SHOT_PROFILE[i + 1][columnIndex]; // Upper value

                // Linear interpolation: t = interpolation factor [0, 1]
                double t = (distanceInches - d0) / (d1 - d0);
                return v0 + t * (v1 - v0);
            }
        }
        return SHOT_PROFILE[SHOT_PROFILE.length - 1][columnIndex];
    }

    /**
     * Set shooter power directly (0.0 to 1.0).
     * 
     * Converts power percentage to target RPM for velocity control.
     * Equation: targetRPM = MOTOR_MAX_RPM × power
     * 
     * @param power Motor power (0.0 to 1.0)
     */
    public void setPower(double power) {
        power = Range.clip(power, 0.0, 1.0);

        if (useVelocityControl) {
            // Convert power percentage to target RPM
            double targetRPM = MOTOR_MAX_RPM * power;
            setTargetRPM(targetRPM);
        } else {
            // Fallback to direct power control
            leftShooterMotor.setPower(power);
            rightShooterMotor.setPower(power);
            this.targetRPM = MOTOR_MAX_RPM * power;
        }
    }

    /**
     * Set target RPM using velocity control for accurate shooting.
     * 
     * @param rpm Target RPM (0 to MOTOR_MAX_RPM)
     */
    public void setTargetRPM(double rpm) {
        targetRPM = Range.clip(rpm, 0.0, MOTOR_MAX_RPM);

        // Convert RPM to ticks per second
        double ticksPerSecond = (targetRPM * TICKS_PER_REVOLUTION) / 60.0;

        // Set velocity for both motors
        leftShooterMotor.setVelocity(ticksPerSecond);
        rightShooterMotor.setVelocity(ticksPerSecond);
    }

    /**
     * Check if shooter is at target RPM (within tolerance).
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
     * Get current RPM based on motor velocity.
     * 
     * @return Current RPM (average of left and right motors)
     */
    public double getCurrentRPM() {
        // Average of both motors for accuracy
        double leftVelocity = leftShooterMotor.getVelocity();
        double rightVelocity = rightShooterMotor.getVelocity();
        double leftRPM = (leftVelocity * 60.0) / TICKS_PER_REVOLUTION;
        double rightRPM = (rightVelocity * 60.0) / TICKS_PER_REVOLUTION;
        return (leftRPM + rightRPM) / 2.0;
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
        }
    }

}
