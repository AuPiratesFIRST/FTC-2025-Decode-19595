package org.firstinspires.ftc.teamcode.SubSystems.Scoring;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.SubSystems.Drive.TileCoordinate;
import org.firstinspires.ftc.teamcode.Constants.FieldConstants;

/**
 * Scoring shooter subsystem with distance-based power and angle calibration.
 * Calculates motor power and launch angle based on robot position to goal
 * distance.
 */
public class ShooterSubsystem {

	private final DcMotorEx shooterMotor;
	private final Servo launchAngleServo; // optional; can be null if fixed-angle
	private final Telemetry telemetry;

	// Use centralized constants
	public static final double GOAL_HEIGHT_INCHES = FieldConstants.GOAL_HEIGHT_INCHES;

	// Shot profile entries: {distance_inches, motor_power_0to1, servo_angle_0to1}
	// Populate via calibration; seed with rough placeholders
	private static final double[][] SHOT_PROFILE = new double[][] {
			{ 24, 0.45, 0.45 },
			{ 36, 0.60, 0.50 },
			{ 48, 0.70, 0.55 },
			{ 60, 0.80, 0.60 }
	};

	public ShooterSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
		this.telemetry = telemetry;
		this.shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter");
		Servo maybeServo;
		try {
			maybeServo = hardwareMap.get(Servo.class, "launchAngle");
		} catch (Exception e) {
			maybeServo = null; // allow operation without angle servo
		}
		this.launchAngleServo = maybeServo;
	}

	public void shootArtifact(TileCoordinate robotPosition, boolean isBlueAlliance) {
		TileCoordinate goal = getGoalPosition(isBlueAlliance);
		double horizontalDistance = robotPosition.distanceTo(goal);

		double motorPower = calculateMotorPower(horizontalDistance, GOAL_HEIGHT_INCHES);
		Double launchAngle = null;
		if (launchAngleServo != null) {
			launchAngle = calculateLaunchAngle(horizontalDistance, GOAL_HEIGHT_INCHES);
			launchAngleServo.setPosition(clamp01(launchAngle));
		}

		shooterMotor.setPower(clamp01(motorPower));

		if (telemetry != null) {
			telemetry.addData("Shooter", "distance=%.1f in, power=%.2f", horizontalDistance, motorPower);
			if (launchAngle != null)
				telemetry.addData("LaunchAngle", "%.2f", launchAngle);
		}

		// Note: actual firing mechanism (feeder/trigger) should be actuated by caller
	}

	public static TileCoordinate getGoalPosition(boolean isBlueAlliance) {
		return isBlueAlliance ? new TileCoordinate(FieldConstants.GOAL_X_BLUE, FieldConstants.GOAL_Y_BLUE)
				: new TileCoordinate(FieldConstants.GOAL_X_RED, FieldConstants.GOAL_Y_RED);
	}

	public double calculateMotorPower(double horizontalDistanceInches, double goalHeightInches) {
		// Basic: interpolate power by horizontal distance using SHOT_PROFILE
		return interpolateProfile(horizontalDistanceInches, /* columnIndex */1);
	}

	public double calculateLaunchAngle(double horizontalDistanceInches, double goalHeightInches) {
		// If angle servo is present, interpolate desired angle from profile
		return interpolateProfile(horizontalDistanceInches, /* columnIndex */2);
	}

	/**
	 * Linear interpolation between shot profile entries.
	 * 
	 * Interpolates motor power or launch angle based on distance to goal.
	 * Uses linear interpolation formula: value = v0 + t × (v1 - v0)
	 * where t = (distance - d0) / (d1 - d0) is the interpolation factor [0, 1]
	 * 
	 * Example: distance = 42 inches, between entries at 36" and 48"
	 * - t = (42 - 36) / (48 - 36) = 0.5
	 * - power = 0.60 + 0.5 × (0.70 - 0.60) = 0.65
	 * 
	 * @param distanceInches Distance to goal in inches
	 * @param columnIndex    Column index in SHOT_PROFILE (1 = power, 2 = angle)
	 * @return Interpolated value (motor power or servo angle)
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
				// value = v0 + t × (v1 - v0)
				double t = (distanceInches - d0) / (d1 - d0);
				return v0 + t * (v1 - v0);
			}
		}
		return SHOT_PROFILE[SHOT_PROFILE.length - 1][columnIndex];
	}

	private double clamp01(double v) {
		return Math.max(0.0, Math.min(1.0, v));
	}
}
