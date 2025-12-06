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
 * Integrated shooter subsystem using:
 * - Distance-based power interpolation
 * - Fixed 48° launch angle
 * - Velocity (RPM) control using RUN_USING_ENCODER
 */
public class IntegratedShooterSubsystem {

    private final DcMotorEx leftShooterMotor;
    private final DcMotorEx rightShooterMotor;
    private final Telemetry telemetry;

    // Motor constants
    private static final double MOTOR_MAX_RPM = 6000.0;
    private static final double TICKS_PER_REVOLUTION = 28.0;

    // Shooter fixed angle
    private static final double FIXED_LAUNCH_ANGLE_DEGREES = 48.0;

    // RPM control
    private double targetRPM = 0.0;
    private boolean useVelocityControl = true;

    /**
     * Shot profile: { distance_inches , motor_power }
     */
    private static final double[][] SHOT_PROFILE = new double[][]{
            {20, 0.45},
            {24, 0.60},
            {36, 0.70},
            {48, 0.80},
            {60, 0.85},
            {72, 0.90},
            {88, 1.00}
    };

    public IntegratedShooterSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        leftShooterMotor = hardwareMap.get(DcMotorEx.class, "shooterL");
        rightShooterMotor = hardwareMap.get(DcMotorEx.class, "shooterR");

        configureMotors();
    }

    private void configureMotors() {
        leftShooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightShooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Reverse one motor
        rightShooterMotor.setDirection(DcMotor.Direction.REVERSE);

        // Shooter velocity PIDF tuning
        PIDFCoefficients velocityPIDF = new PIDFCoefficients(35.0, 0.15, 12.0, 15.0);
        leftShooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, velocityPIDF);
        rightShooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, velocityPIDF);
    }

    /** Auto-shoot based on robot position + alliance. */
    public void shootArtifact(TileCoordinate robotPosition, boolean isBlueAlliance) {
        TileCoordinate goal = getGoalPosition(isBlueAlliance);
        double distance = robotPosition.distanceTo(goal);

        double power = calculateMotorPower(distance);
        setPower(power);

        if (telemetry != null) {
            telemetry.addData("Shooter Distance", "%.1f in", distance);
            telemetry.addData("Shooter Power", "%.2f", power);
            telemetry.addData("Target RPM", "%.0f", targetRPM);
            telemetry.addData("Launch Angle", "%.1f°", FIXED_LAUNCH_ANGLE_DEGREES);
        }
    }

    /** Returns goal position based on alliance. */
    public static TileCoordinate getGoalPosition(boolean isBlueAlliance) {
        return isBlueAlliance
                ? new TileCoordinate(FieldConstants.GOAL_X_BLUE, FieldConstants.GOAL_Y_BLUE)
                : new TileCoordinate(FieldConstants.GOAL_X_RED, FieldConstants.GOAL_Y_RED);
    }

    /** Returns interpolated power from distance. */
    public double calculateMotorPower(double distanceInches) {
        return interpolateProfile(distanceInches, 1);
    }

    public static double getFixedLaunchAngle() {
        return FIXED_LAUNCH_ANGLE_DEGREES;
    }

    /** Linear interpolation for shot profile. */
    private double interpolateProfile(double distance, int columnIndex) {
        if (distance <= SHOT_PROFILE[0][0])
            return SHOT_PROFILE[0][columnIndex];

        if (distance >= SHOT_PROFILE[SHOT_PROFILE.length - 1][0])
            return SHOT_PROFILE[SHOT_PROFILE.length - 1][columnIndex];

        for (int i = 0; i < SHOT_PROFILE.length - 1; i++) {
            double d0 = SHOT_PROFILE[i][0];
            double d1 = SHOT_PROFILE[i + 1][0];

            if (distance >= d0 && distance <= d1) {
                double v0 = SHOT_PROFILE[i][columnIndex];
                double v1 = SHOT_PROFILE[i + 1][columnIndex];

                double t = (distance - d0) / (d1 - d0);
                return v0 + t * (v1 - v0);
            }
        }
        return SHOT_PROFILE[SHOT_PROFILE.length - 1][columnIndex];
    }

    /** Sets power → converts to RPM if using velocity mode. */
    public void setPower(double power) {
        power = Range.clip(power, 0.0, 1.0);

        if (useVelocityControl) {
            double rpm = MOTOR_MAX_RPM * power;
            setTargetRPM(rpm);
        } else {
            leftShooterMotor.setPower(power);
            rightShooterMotor.setPower(power);
            targetRPM = MOTOR_MAX_RPM * power;
        }
    }

    /** Applies RPM velocity control. */
    public void setTargetRPM(double rpm) {
        targetRPM = Range.clip(rpm, 0, MOTOR_MAX_RPM);

        double ticksPerSecond = (targetRPM * TICKS_PER_REVOLUTION) / 60.0;

        leftShooterMotor.setVelocity(ticksPerSecond);
        rightShooterMotor.setVelocity(ticksPerSecond);
    }

    /** True when both motors reached target RPM. */
    public boolean atTargetRPM() {
        double left = leftShooterMotor.getVelocity() * 60.0 / TICKS_PER_REVOLUTION;
        double right = rightShooterMotor.getVelocity() * 60.0 / TICKS_PER_REVOLUTION;

        return Math.abs(left - targetRPM) < 150 &&
                Math.abs(right - targetRPM) < 150;
    }

    /** Stops motors safely. */
    public void stop() {
        leftShooterMotor.setPower(0);
        rightShooterMotor.setPower(0);
        targetRPM = 0;
    }

    /** For teleop loop or dashboard. */
    public void updateTelemetry() {
        if (telemetry != null) {
            double leftRPM = leftShooterMotor.getVelocity() * 60.0 / TICKS_PER_REVOLUTION;
            double rightRPM = rightShooterMotor.getVelocity() * 60.0 / TICKS_PER_REVOLUTION;

            telemetry.addData("Left Shooter RPM", (int) leftRPM);
            telemetry.addData("Right Shooter RPM", (int) rightRPM);
            telemetry.addData("Target RPM", (int) targetRPM);
        }
    }
}
