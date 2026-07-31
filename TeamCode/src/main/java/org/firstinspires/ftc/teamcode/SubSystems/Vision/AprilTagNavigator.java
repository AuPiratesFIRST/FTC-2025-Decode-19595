package org.firstinspires.ftc.teamcode.SubSystems.Vision;

import android.util.Size;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.concurrent.TimeUnit;
import java.util.List;
import java.util.Comparator;
import java.util.stream.Collectors;

import org.firstinspires.ftc.teamcode.SubSystems.Drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Drive.TileCoordinate;
import org.firstinspires.ftc.teamcode.Constants.FieldConstants;

import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;

/**
 * Optimized AprilTag Navigator for DECODE.
 * Fixes "Flickering" with 320x240 @ 3ms exposure.
 * Restored ALL legacy methods to fix "cannot find symbol" errors.
 */
public class AprilTagNavigator {

    private DriveSubsystem driveSubsystem;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private Telemetry telemetry;

    private final double MIN_DETECTION_DISTANCE = 1.0;
    private final double MAX_DETECTION_DISTANCE = 250;
    private final double MIN_DECISION_MARGIN = 0.15;

    private static final double CAMERA_HEIGHT = 9.375;
    private static final double CAMERA_Y_OFFSET = 9.0; // inches forward of center

    // Trusted ranges for Global Positioning
    private final double MAX_TRUST_DISTANCE = 100.0; // Trust tags within 4 tiles
    private final double MIN_TRUST_DISTANCE = 6.0;   // Too close = distortion
    private final double TRUST_WEIGHT_BASE = 0.1;   // How much to "nudge" pose per detection (0.1 = 10%)
    private int cameraExposureMs = 14;
    private int cameraGain = 35;
    private boolean cameraControlsSet = false;

    // Alignment logic state (from TagChaserOp)
    private double lastErrorX = 0;
    private double lastTime = 0;
    private final ElapsedTime alignmentTimer = new ElapsedTime();

    public AprilTagNavigator(DriveSubsystem driveSubsystem, HardwareMap hardwareMap, Telemetry telemetry) {
        this.driveSubsystem = driveSubsystem;
        this.telemetry = telemetry;

        Position cameraPosition = new Position(DistanceUnit.INCH, 0, 9.0, CAMERA_HEIGHT, 0);
        YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0, 0, 0, 0);

        aprilTag = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setCameraPose(cameraPosition, cameraOrientation)
                // Calibrated Intrinsics for 320x240
                .setLensIntrinsics(270.376, 270.376, 153.340, 120.421)
                .build();

        // FIX: Documentation shows setDecimation is on the Processor, not the Builder
        aprilTag.setDecimation(2.0f);

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(320, 240))
                .addProcessor(aprilTag)
                .build();

        FtcDashboard.getInstance().startCameraStream(visionPortal, 15);
    }

    // ==================== RESTORED METHODS FOR COMPATIBILITY ====================

    /**
     * Get the last calculated global position as a Pedro Pathing Pose.
     * Automatically converts from FTC coordinates to Pedro coordinates.
     * 
     * @return Pose in Pedro coordinate system
     */
    public Pose getPedroPose() {
        if (driveSubsystem == null) return new Pose(0, 0, 0);
        
        TileCoordinate pos = driveSubsystem.getCurrentPosition();
        double heading = driveSubsystem.getHeading();
        
        // Create Pedro Pose using the FTC coordinate system, then convert to Pedro internal system
        return new Pose(pos.getX(), pos.getY(), heading, FTCCoordinates.INSTANCE)
                .getAsCoordinateSystem(PedroCoordinates.INSTANCE);
    }

    /**
     * GLOBAL POSE ESTIMATOR (FRC-Style)
     * Fuses all visible tags to update the robot's field position and heading.
     * Use this to correct odometry drift and fix "wrong starting position" errors.
     *
     * @return true if a confident update was performed
     */
    public boolean updateGlobalPoseEstimate() {
        if (driveSubsystem != null) {
            // Safety: Don't trust vision if rotating too fast (prevents motion blur errors)
            if (Math.abs(driveSubsystem.getAngularVelocity()) > 2.0) return false;
        }

        List<AprilTagDetection> detections = aprilTag.getDetections();
        if (detections.isEmpty()) return false;

        double weightedX = 0, weightedY = 0, weightedSin = 0, weightedCos = 0;
        double totalWeight = 0;

        for (AprilTagDetection det : detections) {
            if (det.ftcPose == null) continue;

            // 1. Calculate Confidence (Weight)
            // Weight decreases with distance and extreme angles
            double dist = det.ftcPose.range;
            if (dist > MAX_TRUST_DISTANCE || dist < MIN_TRUST_DISTANCE) continue;

            double weight = (1.0 - (dist / MAX_TRUST_DISTANCE)); // 1.0 at 0in, 0.0 at 100in
            weight *= (det.decisionMargin / 30.0); // Higher margin = more trust
            weight = Range.clip(weight, 0.0, 1.0);

            // 2. Calculate Robot Pose from this specific tag
            double[] pose = calculateRobotPoseWithoutUpdating(det);
            if (pose == null) continue;

            weightedX += pose[0] * weight;
            weightedY += pose[1] * weight;
            weightedSin += Math.sin(pose[2]) * weight;
            weightedCos += Math.cos(pose[2]) * weight;
            totalWeight += weight;
        }

        if (totalWeight < 0.1) return false; // Not enough confident data

        // 3. Average the detections
        double finalX = weightedX / totalWeight;
        double finalY = weightedY / totalWeight;
        double finalHeading = Math.atan2(weightedSin / totalWeight, weightedCos / totalWeight);

        // 4. Apply to DriveSubsystem (Global Position Update)
        if (driveSubsystem != null) {
            TileCoordinate currentPos = driveSubsystem.getCurrentPosition();
            
            // If the error is MASSIVE (> 12 inches), we assume the robot started in the wrong spot 
            // or skipped, and we "Snap" to the vision pose.
            double error = currentPos.distanceTo(new TileCoordinate(finalX, finalY));
            
            if (error > 12.0) {
                driveSubsystem.setPosition(new TileCoordinate(finalX, finalY));
                driveSubsystem.setHeading(finalHeading);
                telemetry.addData("Vision", "POS SNAP: Correction %.1f in", error);
            } else {
                // Smoothly "nudge" the current odometry towards the vision pose (Kalman-lite)
                double newX = currentPos.getX() + (finalX - currentPos.getX()) * TRUST_WEIGHT_BASE;
                double newY = currentPos.getY() + (finalY - currentPos.getY()) * TRUST_WEIGHT_BASE;
                
                driveSubsystem.setPosition(new TileCoordinate(newX, newY));
                // We trust the IMU for heading mostly, but nudge it if vision is confident
                driveSubsystem.setHeading(finalHeading); 
            }
        }
        return true;
    }

    /**
     * RESTORED: Used by RedAllianceAuto/EncoderAuto for complex telemetry.
     */
    public void updateDECODELocalizationTelemetry() {
        if (telemetry == null) return;
        List<AprilTagDetection> detections = aprilTag.getDetections();
        telemetry.addData("Vision FPS", visionPortal.getFps());
        telemetry.addData("Tags Seen", detections.size());

        AprilTagDetection best = getBestDetection();
        if (best != null) {
            telemetry.addData("Best Tag", "ID %d Range %.1f", best.id, best.ftcPose.range);
        }
    }

    public boolean isObeliskTag(AprilTagDetection detection) {
        return detection != null && (detection.id == 21 || detection.id == 22 || detection.id == 23);
    }

    // ==================== CORE VISION LOGIC ====================

    public void initializeCameraControls() {
        if (!cameraControlsSet && visionPortal != null) {
            if (setCameraControls(cameraExposureMs, cameraGain)) {
                cameraControlsSet = true;
            }
        }
    }

    public boolean setCameraControls(int exposureMs, int gain) {
        if (visionPortal == null || visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) return false;
        try {
            ExposureControl exp = visionPortal.getCameraControl(ExposureControl.class);
            if (exp.getMode() != ExposureControl.Mode.Manual) {
                exp.setMode(ExposureControl.Mode.Manual);
                Thread.sleep(50);
            }
            exp.setExposure((long) exposureMs, TimeUnit.MILLISECONDS);
            GainControl g = visionPortal.getCameraControl(GainControl.class);
            g.setGain(Range.clip(gain, g.getMinGain(), g.getMaxGain()));
            return true;
        } catch (Exception e) { return false; }
    }

    public AprilTagDetection getBestDetection() {
        return getBestDetectionForTag(-1); // -1 signifies "any alliance tag"
    }

    public TileCoordinate calculateRobotPosition(AprilTagDetection detection) {
        double[] pose = calculateRobotPoseWithoutUpdating(detection);
        if (pose == null) return null;
        
        TileCoordinate robotPos = new TileCoordinate(pose[0], pose[1]);
        if (driveSubsystem != null) {
            driveSubsystem.setPosition(robotPos);
            driveSubsystem.setHeading(pose[2]);
        }
        return robotPos;
    }

    private double[] calculateRobotPoseWithoutUpdating(AprilTagDetection detection) {
        if (detection == null) return null;
        double[] tagPos = null;
        for (double[] p : FieldConstants.APRILTAG_POSITIONS) {
            if (p != null && (int) p[0] == detection.id) { tagPos = p; break; }
        }
        if (tagPos == null) return null;

        double relX = detection.ftcPose.x;
        double relY = detection.ftcPose.y;
        double cameraHeading = Math.toRadians(tagPos[3]) + Math.PI - Math.toRadians(detection.ftcPose.yaw);

        double cosH = Math.cos(cameraHeading);
        double sinH = Math.sin(cameraHeading);

        // Camera position in field coordinates
        double cameraX = tagPos[1] - (relX * cosH - relY * sinH);
        double cameraY = tagPos[2] - (relX * sinH + relY * cosH);

        // Robot center is 9.0 inches behind the camera (offset in robot Y direction)
        double robotX = cameraX - 9.0 * cosH;
        double robotY = cameraY - 9.0 * sinH;

        return new double[]{robotX, robotY, cameraHeading};
    }

    public TileCoordinate getCurrentTilePosition() {
        AprilTagDetection best = getBestDetection();
        return (best == null) ? null : calculateRobotPosition(best);
    }

    public AprilTagDetection getBestAllianceGoalDetection() { return getBestDetection(); }

    /**
     * Returns the best (closest) detection for a specific tag ID.
     * If tagId is -1, returns the best detection from tags 20 or 24.
     * Used by AimController for alliance-specific goal alignment.
     */
    public AprilTagDetection getBestDetectionForTag(int tagId) {
        return aprilTag.getDetections().stream()
                .filter(tag -> tag.ftcPose != null && tag.ftcPose.range <= MAX_DETECTION_DISTANCE)
                .filter(tag -> (tagId == -1) ? (tag.id == 20 || tag.id == 24) : (tag.id == tagId))
                .min(Comparator.comparingDouble(tag -> tag.ftcPose.range))
                .orElse(null);
    }

    public List<AprilTagDetection> getRawDetections() { return aprilTag.getDetections(); }
    public void closeVision() { if (visionPortal != null) visionPortal.close(); }

    /**
     * REFINED ALIGNMENT LOGIC (from TagChaserOp)
     * Calculates powers to align with a tag using PD control for strafe and deadbands for stability.
     * Integrates IMU heading if robotHeading is provided (not null).
     *
     * @param tag The tag to align to
     * @param desiredDist Target distance from tag (inches)
     * @param targetHeadingRad Optional: Target field heading in radians. If null, uses tag bearing.
     * @param currentHeadingRad Robot's current field heading in radians.
     * @return double[] {strafe, forward, turn, isAligned(1.0 or 0.0)}
     */
    public double[] calculateAlignmentCorrections(AprilTagDetection tag, double desiredDist, 
                                                  Double targetHeadingRad, double currentHeadingRad,
                                                  double dbX, double dbY, double dbAngleDeg,
                                                  double kPS, double kPF, double kPR, double kDS, double maxP) {
        if (tag == null || tag.ftcPose == null) {
            lastErrorX = 0;
            lastTime = 0;
            return new double[]{0, 0, 0, 0.0};
        }

        double currentTime = alignmentTimer.seconds();
        double deltaTime = (lastTime == 0) ? 0.02 : (currentTime - lastTime);
        
        // 1. FORWARD/BACK (Y error)
        double fErr = tag.ftcPose.y - desiredDist;
        double fP = (Math.abs(fErr) > dbY) ? fErr * kPF : 0;

        // 2. STRAFE PD (X error)
        double errorX = tag.ftcPose.x;
        double dTermX = (deltaTime > 0) ? ((errorX - lastErrorX) / deltaTime) * kDS : 0;
        double sP = (Math.abs(errorX) > dbX) ? (errorX * kPS + dTermX) : 0;

        // 3. ROTATION
        double tP = 0;
        double angleErrDeg;
        
        if (targetHeadingRad != null) {
            // Use IMU for heading alignment (Field relative)
            double headingErrRad = AngleUnit.normalizeRadians(targetHeadingRad - currentHeadingRad);
            angleErrDeg = Math.toDegrees(headingErrRad);
            tP = headingErrRad * kPR * 57.29; // Scale P to match degree-based gains
        } else {
            // Use tag bearing for alignment (Robot relative)
            angleErrDeg = tag.ftcPose.bearing;
            tP = (Math.abs(angleErrDeg) > dbAngleDeg) ? angleErrDeg * kPR : 0;
        }

        // Update history
        lastErrorX = errorX;
        lastTime = currentTime;

        // Check if fully aligned
        boolean aligned = Math.abs(fErr) <= dbY 
                       && Math.abs(errorX) <= dbX 
                       && Math.abs(angleErrDeg) <= dbAngleDeg;

        return new double[]{
            Range.clip(sP, -maxP, maxP), 
            Range.clip(fP, -maxP, maxP), 
            Range.clip(tP, -maxP, maxP),
            aligned ? 1.0 : 0.0
        };
    }

    /**
     * Legacy compatibility method.
     */
    public double[] calculateAlignmentCorrections(AprilTagDetection tag, double desiredDist, double desiredAngle,
                                                  double dbX, double dbY, double dbAngle,
                                                  double kPS, double kPF, double kPR, double maxP) {
        return calculateAlignmentCorrections(tag, desiredDist, null, 0, dbX, dbY, dbAngle, kPS, kPF, kPR, 0.03, maxP);
    }
}