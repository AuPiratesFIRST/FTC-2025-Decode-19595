package org.firstinspires.ftc.teamcode.SubSystems.Drive;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Drive subsystem for mecanum wheel drivebase.
 * Provides low-level hardware control for motors and IMU.
 * 
 * Single Responsibility: Hardware control only.
 * For tile-based navigation, use TileNavigator class.
 */
public class DriveSubsystem {

    public final DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private IMU imu; // REV Hub IMU for absolute rotation measurement

    // Pose tracking for localization
    private TileCoordinate currentPosition;
    private double currentHeading; // in radians
    private double poseHeading; // in radians

    private static final double TICKS_PER_REVOLUTION = 560.0;
    private static final double WHEEL_DIAMETER = 4.0; // inches
    private static final double WHEEL_BASE = 16.0; // inches between wheels (left-right)
    private static final double STRAFE_CORRECTION = 1.15; // Mecanum strafe slip compensation

    private Telemetry telemetry;

    public DriveSubsystem(HardwareMap hardwareMap) {
        this(hardwareMap, null);
    }

    public DriveSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        // Initialize motors
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        configureMotors();
        initializeIMU(hardwareMap);
        
        // Initialize pose tracking
        currentPosition = new TileCoordinate(0, 0);
        currentHeading = 0.0;
        poseHeading = 0.0;
    }

    private void configureMotors() {
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);

        // Use RUN_USING_ENCODER for closed-loop control
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Initialize the IMU (Inertial Measurement Unit) for absolute rotation measurement.
     * 
     * The IMU provides accurate heading data that is immune to wheel slip.
     * Adjust the RevHubOrientationOnRobot parameters to match your physical hub mounting:
     * - LogoFacingDirection: Direction the REV logo is facing (UP, DOWN, LEFT, RIGHT, FORWARD, BACKWARD)
     * - UsbFacingDirection: Direction the USB port is facing
     * 
     * Default configuration assumes:
     * - Logo facing UP (toward ceiling)
     * - USB port facing FORWARD (toward front of robot)
     */
    private void initializeIMU(HardwareMap hardwareMap) {
        imu = hardwareMap.get(IMU.class, "imu");
        
        // Configure IMU orientation based on REV Control Hub mounting
        // ADJUST THESE VALUES to match your physical robot configuration!
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT);

        imu.initialize(new IMU.Parameters(orientationOnRobot));
        
        if (telemetry != null) {
            telemetry.addData("IMU", "Initialized successfully");
        }
    }

    // =================== Drive Methods ===================

    /**
     * Mecanum wheel drive kinematics with power normalization.
     * Calculates individual motor powers for mecanum wheel drivebase.
     * Automatically scales powers to prevent any motor from exceeding ±1.0.
     */
    public void drive(double forward, double strafe, double turn) {
        double fl = forward + strafe + turn; // Left Front
        double fr = forward - strafe - turn; // Right Front
        double bl = forward - strafe + turn; // Left Rear
        double br = forward + strafe - turn; // Right Rear

        // Normalize powers to prevent exceeding ±1.0
        double max = Math.max(1.0, Math.max(Math.abs(fl), Math.max(Math.abs(fr), 
                                     Math.max(Math.abs(bl), Math.abs(br)))));
        fl /= max;
        fr /= max;
        bl /= max;
        br /= max;

        // Set motor powers (already normalized, no clipping needed)
        leftFront.setPower(fl);
        rightFront.setPower(fr);
        leftRear.setPower(bl);
        rightRear.setPower(br);
    }

    public void stop() {
        drive(0, 0, 0);
    }

    // =================== Movement Methods ===================

    /**
     * Move the robot forward or backward by a specific distance in inches.
     * NOTE: This is a blocking call. Motors will run to position asynchronously.
     * Call isAtTarget() in your OpMode loop to check completion.
     */
    public void moveInches(double inches, double power) {
        double wheelCircumference = Math.PI * WHEEL_DIAMETER;
        double ticksRequired = (inches / wheelCircumference) * TICKS_PER_REVOLUTION;

        leftFront.setTargetPosition(leftFront.getCurrentPosition() + (int) ticksRequired);
        leftRear.setTargetPosition(leftRear.getCurrentPosition() + (int) ticksRequired);
        rightFront.setTargetPosition(rightFront.getCurrentPosition() + (int) ticksRequired);
        rightRear.setTargetPosition(rightRear.getCurrentPosition() + (int) ticksRequired);

        setMotorModes(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(Math.abs(power));
        leftRear.setPower(Math.abs(power));
        rightFront.setPower(Math.abs(power));
        rightRear.setPower(Math.abs(power));

        // Non-blocking: caller must check isAtTarget() in their loop
        if (telemetry != null) {
            telemetry.addData("Moving", "Target: %.1f inches", inches);
        }
    }

    /**
     * Strafe the robot left or right by a specific distance in inches.
     * Includes STRAFE_CORRECTION factor to compensate for mecanum wheel slip.
     * NOTE: This is a blocking call. Motors will run to position asynchronously.
     * Call isAtTarget() in your OpMode loop to check completion.
     */
    public void strafeInches(double inches, double power) {
        double wheelCircumference = Math.PI * WHEEL_DIAMETER;
        double ticksRequired = (inches / wheelCircumference) * TICKS_PER_REVOLUTION * STRAFE_CORRECTION;

        leftFront.setTargetPosition(leftFront.getCurrentPosition() + (int) -ticksRequired); // Left Front
        leftRear.setTargetPosition(leftRear.getCurrentPosition() + (int) ticksRequired); // Left Rear
        rightFront.setTargetPosition(rightFront.getCurrentPosition() + (int) ticksRequired); // Right Front
        rightRear.setTargetPosition(rightRear.getCurrentPosition() + (int) -ticksRequired); // Right Rear

        setMotorModes(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(Math.abs(power));
        leftRear.setPower(Math.abs(power));
        rightFront.setPower(Math.abs(power));
        rightRear.setPower(Math.abs(power));

        // Non-blocking: caller must check isAtTarget() in their loop
        if (telemetry != null) {
            telemetry.addData("Strafing", "Target: %.1f inches", inches);
        }
    }

    /**
     * Rotate the robot by a specific number of degrees using encoder-based control.
     * Uses the robot's WHEEL_BASE to calculate proper turning arc.
     * NOTE: This is a blocking call. Motors will run to position asynchronously.
     * Call isAtTarget() in your OpMode loop to check completion.
     * 
     * For more accurate rotation, consider using rotateToHeading() with IMU.
     */
    public void rotateDegrees(double degrees, double power) {
        // Calculate arc length for rotation based on wheelbase
        double turnCircumference = Math.PI * WHEEL_BASE;
        double turnDistance = (degrees / 360.0) * turnCircumference;
        
        // Convert arc distance to encoder ticks
        double wheelCircumference = Math.PI * WHEEL_DIAMETER;
        double ticksRequired = (turnDistance / wheelCircumference) * TICKS_PER_REVOLUTION;

        int ticks = (int) ticksRequired;

        // Left motors rotate backward (CCW), right motors forward (CCW rotation)
        leftFront.setTargetPosition(leftFront.getCurrentPosition() - ticks);
        leftRear.setTargetPosition(leftRear.getCurrentPosition() - ticks);
        rightFront.setTargetPosition(rightFront.getCurrentPosition() + ticks);
        rightRear.setTargetPosition(rightRear.getCurrentPosition() + ticks);

        setMotorModes(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(Math.abs(power));
        leftRear.setPower(Math.abs(power));
        rightFront.setPower(Math.abs(power));
        rightRear.setPower(Math.abs(power));

        // Non-blocking: caller must check isAtTarget() in their loop
        if (telemetry != null) {
            telemetry.addData("Rotating", "Target: %.1f degrees", degrees);
        }
    }

    /**
     * IMU-based rotation to absolute heading (more accurate than encoder-based).
     * Rotates to a target heading using closed-loop PID control with the IMU.
     * Call this method repeatedly in your OpMode loop until isAtTarget() returns true.
     * 
     * @param targetHeadingDegrees Target heading in degrees (0 = forward, +CCW)
     * @param kP Proportional gain for rotation control (try 0.02-0.05)
     * @param toleranceDegrees Acceptable heading error in degrees (try 1.0-2.0)
     * @return True if at target heading
     */
    public boolean rotateToHeading(double targetHeadingDegrees, double kP, double toleranceDegrees) {
        double currentHeadingDeg = Math.toDegrees(getHeading());
        double error = AngleUnit.normalizeDegrees(targetHeadingDegrees - currentHeadingDeg);
        
        if (Math.abs(error) < toleranceDegrees) {
            stop();
            return true;
        }
        
        double turnPower = Range.clip(error * kP, -1.0, 1.0);
        drive(0, 0, turnPower);
        
        if (telemetry != null) {
            telemetry.addData("IMU Rotation", "Error: %.1f°", error);
        }
        
        return false;
    }

    /**
     * Restore motors to velocity control mode.
     * Call this after using RUN_TO_POSITION movements to restore normal driving.
     */
    public void restoreNormalMode() {
        setMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);
    }



    /**
     * Set motor modes (RUN_TO_POSITION or RUN_USING_ENCODER)
     */
    private void setMotorModes(DcMotor.RunMode mode) {
        leftFront.setMode(mode);
        leftRear.setMode(mode);
        rightFront.setMode(mode);
        rightRear.setMode(mode);
    }

    // =================== IMU METHODS ===================

    /**
     * Get the robot's current heading from the IMU sensor.
     * 
     * This provides ABSOLUTE rotation measurement that is immune to wheel slip.
     * The IMU continuously tracks rotation even if wheels slip or the robot gets bumped.
     *
     * @return Current heading in radians (0 = forward, positive = CCW rotation)
     */
    public double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    /**
     * Reset the IMU heading to zero.
     * 
     * Call this at the start of autonomous or when the robot is in a known orientation.
     * In TeleOp, map this to a button (e.g., Gamepad1 Start) to recalibrate if IMU drifts.
     */
    public void resetHeading() {
        imu.resetYaw();
        
        if (telemetry != null) {
            telemetry.addData("IMU", "Heading reset to 0°");
        }
    }

    // =================== QUERY METHODS ===================

    /**
     * Check if all motors have reached their target positions.
     *
     * @return True if all motors are at target
     */
    public boolean isAtTarget() {
        return !leftFront.isBusy() && !leftRear.isBusy() && !rightFront.isBusy() && !rightRear.isBusy();
    }

    // =================== POSE TRACKING METHODS ===================

    /**
     * Set the pose heading (initial heading for pose estimation).
     * This is used to initialize the robot's orientation in the field coordinate system.
     * 
     * @param heading Heading in radians
     */
    public void setPoseHeading(double heading) {
        this.poseHeading = heading;
        this.currentHeading = heading;
        
        if (telemetry != null) {
            telemetry.addData("Pose Heading", "%.1f°", Math.toDegrees(heading));
        }
    }

    /**
     * Set the robot's current position on the field.
     * 
     * @param position TileCoordinate representing the robot's position
     */
    public void setPosition(TileCoordinate position) {
        this.currentPosition = position;
        
        if (telemetry != null) {
            telemetry.addData("Position Set", "X: %.1f, Y: %.1f", position.getX(), position.getY());
        }
    }

    /**
     * Set the robot's current heading.
     * 
     * @param heading Heading in radians
     */
    public void setHeading(double heading) {
        this.currentHeading = heading;
        
        if (telemetry != null) {
            telemetry.addData("Heading Set", "%.1f°", Math.toDegrees(heading));
        }
    }

    /**
     * Get the robot's current position on the field.
     * 
     * @return Current TileCoordinate position
     */
    public TileCoordinate getCurrentPosition() {
        return currentPosition;
    }

    /**
     * Get the robot's current heading.
     * 
     * @return Current heading in radians
     */
    public double getCurrentHeading() {
        return currentHeading;
    }
}
