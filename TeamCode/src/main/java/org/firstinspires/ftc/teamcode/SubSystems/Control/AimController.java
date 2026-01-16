package org.firstinspires.ftc.teamcode.SubSystems.Control;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import org.firstinspires.ftc.teamcode.SubSystems.Drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Vision.AprilTagNavigator;

/**
 * UNIFIED ALIGNMENT CONTROLLER
 * 
 * Handles robot alignment using sensor fusion:
 * - AprilTag vision for X/Y position
 * - IMU gyro for rotation control
 * - Automatic fallback to IMU-only when vision fails
 * - Alliance-aware goal targeting (Blue/Red with triangular goal geometry)
 * 
 * Used by both TeleOp and Autonomous to eliminate duplication.
 */
public class AimController {

    // === ALLIANCE COLOR SELECTOR ===
    public enum AllianceColor {
        BLUE,
        RED
    }

    // === DEPENDENCIES ===
    private final AprilTagNavigator aprilTag;
    private final DriveSubsystem drive;
    private final Telemetry telemetry;

    // === ALLIANCE & GOAL CONSTANTS ===
    private AllianceColor alliance = AllianceColor.BLUE; // default

    // Triangular goal scoring face centers (inches)
    // Goals are right triangles with two sides flush against field walls
    private static final double GOAL_X_BLUE = 12.0;
    private static final double GOAL_Y_BLUE = 132.0;

    private static final double GOAL_X_RED = 132.0;
    private static final double GOAL_Y_RED = 132.0;

    // Fixed aiming angles for each alliance (degrees)
    // Blue goal at (12, 132): hypotenuse angles down-right at 45°
    // Red goal at (132, 132): hypotenuse angles down-left at 135°
    private static final double ANGLE_BLUE = 45.0;
    private static final double ANGLE_RED = 135.0;

    // === CONFIGURATION (Tunable) ===
    private double desiredDistance = 134.0;
    private double desiredAngle = 45.0; // Will be set by setAlliance()
    private double kpStrafe = 0.03;
    private double kpForward = 0.03;
    private double kpRot = 0.015;
    private double maxPower = 0.40;
    private double posDeadband = 0.75;
    private double angleDeadband = 1.5;
    private double imuOnlyDeadband = 2.0;
    private int targetTagId = 24;
    private long visionLossTimeoutMs = 500;
    
    // === VISION SMOOTHING (Exponential Moving Average) ===
    private double visionAlpha = 0.3;  // 30% new measurement, 70% old
    private double lastStrafeCorrection = 0.0;
    private double lastForwardCorrection = 0.0;

    // === STATE TRACKING ===
    private ElapsedTime visionLostTimer = new ElapsedTime();
    private boolean visionWasLost = false;

    // === ALIGNMENT MODES ===
    public enum AlignmentMode {
        VISION_FUSION,   // AprilTag + IMU active
        IMU_HOLD,        // Vision lost briefly, holding angle
        IMU_ONLY,        // Vision lost completely, rotating to target
        ALIGNED          // Target reached
    }

    // === RESULT DATA ===
    public static class AlignmentResult {
        public double strafe;
        public double forward;
        public double turn;
        public boolean aligned;
        public AlignmentMode mode;
        
        public AlignmentResult(double strafe, double forward, double turn, boolean aligned, AlignmentMode mode) {
            this.strafe = strafe;
            this.forward = forward;
            this.turn = turn;
            this.aligned = aligned;
            this.mode = mode;
        }
    }

    // === CONSTRUCTOR ===
    public AimController(AprilTagNavigator aprilTag, DriveSubsystem drive, Telemetry telemetry) {
        this.aprilTag = aprilTag;
        this.drive = drive;
        this.telemetry = telemetry;
    }

    // === CONFIGURATION SETTERS ===
    /**
     * Set the alliance color, which automatically updates the desired aiming angle
     * and goal coordinates for the scoring triangle hypotenuse.
     *
     * @param alliance BLUE or RED
     */
    public void setAlliance(AllianceColor alliance) {
        this.alliance = alliance;
        // Update desiredAngle automatically based on alliance
        this.desiredAngle = (alliance == AllianceColor.BLUE) ? ANGLE_BLUE : ANGLE_RED;
    }

    /**
     * Get current alliance setting
     * @return current AllianceColor
     */
    public AllianceColor getAlliance() {
        return alliance;
    }

    /**
     * Get current goal X coordinate based on alliance
     * @return goal X in inches
     */
    public double getGoalX() {
        return (alliance == AllianceColor.BLUE) ? GOAL_X_BLUE : GOAL_X_RED;
    }

    /**
     * Get current goal Y coordinate based on alliance
     * @return goal Y in inches
     */
    public double getGoalY() {
        return (alliance == AllianceColor.BLUE) ? GOAL_Y_BLUE : GOAL_Y_RED;
    }

    public void setDesiredDistance(double distance) { this.desiredDistance = distance; }
    public void setDesiredAngle(double angle) { this.desiredAngle = angle; }
    public void setGains(double kpStrafe, double kpForward, double kpRot) {
        this.kpStrafe = kpStrafe;
        this.kpForward = kpForward;
        this.kpRot = kpRot;
    }
    public void setMaxPower(double maxPower) { this.maxPower = maxPower; }
    public void setDeadbands(double pos, double angle, double imuOnly) {
        this.posDeadband = pos;
        this.angleDeadband = angle;
        this.imuOnlyDeadband = imuOnly;
    }
    public void setTargetTagId(int tagId) { this.targetTagId = tagId; }
    public void setVisionLossTimeout(long timeoutMs) { this.visionLossTimeoutMs = timeoutMs; }
    public void setVisionSmoothing(double alpha) { this.visionAlpha = alpha; }

    // === MAIN UPDATE METHOD ===
    /**
     * Calculates alignment corrections for the robot.
     * Call this every loop when alignment is desired.
     * 
     * @return AlignmentResult containing drive commands and status
     */
    public AlignmentResult update() {
        
        // === PHASE 1: CHECK VISION ===
        boolean tagSeen = aprilTag.updateRobotPositionFromAllianceGoals();
        AprilTagDetection tag = aprilTag.getBestAllianceGoalDetection();

        if (tagSeen && tag != null && tag.id == targetTagId) {
            // Vision active - reset timer
            visionLostTimer.reset();
            visionWasLost = false;

            // === PHASE 2: VISION + IMU FUSION ===
            double[] corrections = aprilTag.calculateAlignmentCorrections(
                    tag,
                    desiredDistance,
                    desiredAngle,
                    posDeadband,
                    posDeadband,
                    0,              // dbAngle = 0 (IMU handles rotation)
                    kpStrafe,
                    kpForward,
                    0,              // kPR = 0 (IMU controls turn)
                    maxPower
            );

            if (corrections != null) {
                // Calculate IMU-based rotation
                double currentHeading = drive.getHeadingDegrees(); // Use getHeadingDegrees() when working with degrees
                double angleError = AngleUnit.normalizeDegrees(desiredAngle - currentHeading);
                double turnPower = Range.clip(angleError * kpRot, -maxPower, maxPower);

                // Apply exponential moving average to vision corrections (reduce jitter)
                double smoothedStrafe = visionAlpha * corrections[1] + (1.0 - visionAlpha) * lastStrafeCorrection;
                double smoothedForward = visionAlpha * corrections[0] + (1.0 - visionAlpha) * lastForwardCorrection;
                lastStrafeCorrection = smoothedStrafe;
                lastForwardCorrection = smoothedForward;

                // Check if aligned
                boolean aligned = Math.abs(angleError) < angleDeadband &&
                                Math.abs(smoothedStrafe) < 0.05 &&
                                Math.abs(smoothedForward) < 0.05;

                if (aligned) {
                    return new AlignmentResult(0, 0, 0, true, AlignmentMode.ALIGNED);
                }

                return new AlignmentResult(smoothedStrafe, smoothedForward, turnPower, false, AlignmentMode.VISION_FUSION);
            }
        }

        // === PHASE 3: VISION LOST - IMU HOLD OR IMU-ONLY ===
        if (!visionWasLost) {
            visionLostTimer.reset();
            visionWasLost = true;
        }

        double currentHeading = drive.getHeadingDegrees(); // Use getHeadingDegrees() when working with degrees
        double angleError = AngleUnit.normalizeDegrees(desiredAngle - currentHeading);
        double turnPower = Range.clip(angleError * kpRot, -maxPower, maxPower);

        // Still within timeout window? Hold angle (IMU_HOLD mode)
        if (visionLostTimer.milliseconds() < visionLossTimeoutMs) {
            return new AlignmentResult(0, 0, turnPower, false, AlignmentMode.IMU_HOLD);
        }

        // === PHASE 4: IMU-ONLY FALLBACK ===
        // Vision completely lost - actively rotate to target angle
        if (Math.abs(angleError) <= imuOnlyDeadband) {
            return new AlignmentResult(0, 0, 0, true, AlignmentMode.ALIGNED);
        }

        return new AlignmentResult(0, 0, turnPower, false, AlignmentMode.IMU_ONLY);
    }

    // === UTILITY: Reset smoothing filters ===
    public void resetSmoothing() {
        lastStrafeCorrection = 0.0;
        lastForwardCorrection = 0.0;
    }
}
