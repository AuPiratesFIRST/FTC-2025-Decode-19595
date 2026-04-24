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
 * Uses the same logic as TagChaserOp (tested and working):
 * - Forward/back: tag.ftcPose.y vs desired distance, P control with deadband
 * - Strafe: tag.ftcPose.x, PD control with deadband (D term reduces jitter)
 * - Turn: tag.ftcPose.bearing, P control with deadband
 * - Alliance-aware via targetTagId (e.g. 20 Blue, 24 Red)
 * - When vision lost: IMU hold then IMU-only fallback
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
    private static final double ANGLE_BLUE = 25.0;
    private static final double ANGLE_RED = -25.5;

    // === CONFIGURATION (TagChaserOp-tuned defaults) ===
    private double desiredDistance = 134.0;
    private double desiredAngle = 25.0;
    private double kpForward = 0.079;   // TagChaserOp Kp
    private double kpStrafe = 0.075;    // TagChaserOp StrafeKp
    private double kdStrafe = 0.030;    // TagChaserOp StrafeKd (PD for strafe)
    private double kpTurn = 0.020;      // TagChaserOp TurnKp
    private double maxPower = 1.0;     // TagChaserOp SPEED_LIMIT
    private double forwardDeadband = 1.0;   // TagChaserOp FORWARD_DEADBAND inches
    private double strafeDeadband = 0.75;   // TagChaserOp STRAFE_DEADBAND inches
    private double turnDeadband = 1.5;      // TagChaserOp TURN_DEADBAND degrees
    private double imuOnlyDeadband = 2.0;
    private int targetTagId = 24;
    private long visionLossTimeoutMs = 500;

    // === STATE TRACKING (for Strafe D-term, same as TagChaserOp) ===
    private double lastErrorX = 0;
    private double lastTime = 0;
    private ElapsedTime runtime = new ElapsedTime();
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
    /** Sets P gains: forward, strafe, turn (TagChaserOp: 0.079, 0.075, 0.020) */
    public void setGains(double kpForward, double kpStrafe, double kpTurn) {
        this.kpForward = kpForward;
        this.kpStrafe = kpStrafe;
        this.kpTurn = kpTurn;
    }
    /** Sets strafe D gain (TagChaserOp: 0.030) for PD control. */
    public void setStrafeKd(double kdStrafe) { this.kdStrafe = kdStrafe; }
    public void setMaxPower(double maxPower) { this.maxPower = maxPower; }
    /** Deadbands: forward (in), strafe (in), turn (deg). TagChaserOp: 1.0, 0.75, 1.5 */
    public void setDeadbands(double forwardDb, double strafeDb, double turnDb) {
        this.forwardDeadband = forwardDb;
        this.strafeDeadband = strafeDb;
        this.turnDeadband = turnDb;
    }
    public void setTargetTagId(int tagId) { this.targetTagId = tagId; }
    public void setVisionLossTimeout(long timeoutMs) { this.visionLossTimeoutMs = timeoutMs; }

    // === MAIN UPDATE METHOD (TagChaserOp logic) ===
    /**
     * Calculates alignment corrections using the same math as TagChaserOp.
     * Returns (forward, strafe, turn) for drive(forward, strafe, turn).
     */
    public AlignmentResult update() {
        AprilTagDetection tag = aprilTag.getBestDetectionForTag(targetTagId);

        if (tag != null && tag.ftcPose != null) {
            visionLostTimer.reset();
            visionWasLost = false;

            double currentTime = runtime.seconds();
            double deltaTime = currentTime - lastTime;

            // --- 1. FORWARD/BACK (TagChaserOp: errory, Kp, deadband) ---
            double errory = tag.ftcPose.y - desiredDistance;
            double driveForward = 0;
            if (Math.abs(errory) > forwardDeadband) {
                driveForward = errory * kpForward;
            }

            // --- 2. STRAFE PD (TagChaserOp: errorx, StrafeKp, StrafeKd, deadband) ---
            double errorx = tag.ftcPose.x;
            double driveStrafe = 0;
            if (Math.abs(errorx) > strafeDeadband) {
                double pTermX = errorx * kpStrafe;
                double dTermX = (deltaTime > 0) ? ((errorx - lastErrorX) / deltaTime) * kdStrafe : 0;
                driveStrafe = pTermX + dTermX;
            }

            // --- 3. ROTATION (TagChaserOp: bearing, TurnKp, deadband) ---
            double errorRot = tag.ftcPose.bearing;
            double driveTurn = 0;
            if (Math.abs(errorRot) > turnDeadband) {
                driveTurn = errorRot * kpTurn;
            }

            lastErrorX = errorx;
            lastTime = currentTime;

            double forward = Range.clip(driveForward, -maxPower, maxPower);
            double strafe = Range.clip(driveStrafe, -maxPower, maxPower);
            double turn = Range.clip(driveTurn, -maxPower, maxPower);

            boolean aligned = Math.abs(errory) <= forwardDeadband
                    && Math.abs(errorx) <= strafeDeadband
                    && Math.abs(errorRot) <= turnDeadband;

            if (aligned) {
                return new AlignmentResult(0, 0, 0, true, AlignmentMode.ALIGNED);
            }
            return new AlignmentResult(strafe, forward, turn, false, AlignmentMode.VISION_FUSION);
        }

        // === VISION LOST: IMU hold then IMU-only ===
        if (!visionWasLost) {
            visionLostTimer.reset();
            visionWasLost = true;
        }
        lastErrorX = 0;
        lastTime = runtime.seconds();

        double angleError = AngleUnit.normalizeDegrees(desiredAngle - drive.getHeadingDegrees());
        double turnPower = Range.clip(angleError * kpTurn, -maxPower, maxPower);

        if (visionLostTimer.milliseconds() < visionLossTimeoutMs) {
            return new AlignmentResult(0, 0, turnPower, false, AlignmentMode.IMU_HOLD);
        }
        if (Math.abs(angleError) <= imuOnlyDeadband) {
            return new AlignmentResult(0, 0, 0, true, AlignmentMode.ALIGNED);
        }
        return new AlignmentResult(0, 0, turnPower, false, AlignmentMode.IMU_ONLY);
    }

    public void resetSmoothing() {
        lastErrorX = 0;
        lastTime = runtime.seconds();
    }
}
