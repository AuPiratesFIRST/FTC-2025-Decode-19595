package org.firstinspires.ftc.teamcode.SubSystems.Vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;//
// import com.acmerobotics.dashboard.FtcDashboard;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import com.qualcomm.robotcore.util.Range;

import java.util.concurrent.TimeUnit;

import java.util.List;
import java.util.Comparator;

import org.firstinspires.ftc.teamcode.SubSystems.Drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Drive.TileCoordinate;
import org.firstinspires.ftc.teamcode.Constants.FieldConstants;

/**
 * AprilTag-based localization system for tile-based navigation.
 * Uses AprilTag detection to determine robot's current tile position on the
 * field.
 * 
 * @author Pedro Pathing Team
 * @version 2.0
 */
public class AprilTagNavigator {

    private DriveSubsystem driveSubsystem;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private Telemetry telemetry;

    // AprilTag field positions and specs are centralized in FieldConstants

    // Detection parameters optimized for DECODE field
    // Based on FTC SDK recommendations and field testing
    // Reduced minimum distance for redundancy - allows detection closer to tags
    private final double MIN_DETECTION_DISTANCE = 1.0; // Minimum distance for reliable detection (inches) - reduced from 3.0 for redundancy
    private final double MAX_DETECTION_DISTANCE = 200; // Maximum distance for detection (increased for goal tags at
                                                       // field edges)
    // Note: decisionMargin is NOT a confidence score (0-1). It's a measure of how
    // close the tag
    // classification was between two families. Values can be > 1.0 or < 0.2
    // inconsistently.
    // We use a low threshold to avoid filtering valid detections.
    private final double MIN_DECISION_MARGIN = 30.0; // Minimum decision margin (not a confidence score)

    // Camera position on robot
    // Camera is in the middle of the robot (x=0, y=0) and 9 3/8 inches off the
    // ground
    // Position: (x, y, z) where x=left/right, y=forward/back, z=up/down from robot
    // center
    // Since camera is in middle: x=0, y=0, z=9.375 inches
    private static final double CAMERA_HEIGHT = 9.375; // 9 3/8 inches = 9.375 inches

    // Camera control settings (optimized for DECODE field lighting)
    private int cameraExposureMs = 11; // Low exposure to reduce motion blur (milliseconds)
    private int cameraGain = 20; // Higher gain for low-light conditions (0-255)
    private boolean cameraControlsSet = false;

    public AprilTagNavigator(DriveSubsystem driveSubsystem, HardwareMap hardwareMap, Telemetry telemetry) {
        this.driveSubsystem = driveSubsystem;
        this.telemetry = telemetry;

        // Initialize AprilTag vision for DECODE field
        // Using FTC SDK 8.2+ AprilTag processor with 36h11 family

        // Configure camera pose on robot
        // Camera is in the middle of the robot (x=0, y=0) and 9 3/8 inches off the
        // ground
        // Position: (x, y, z) where x=left/right, y=forward/back, z=up/down from robot
        // center
        // Orientation: (yaw, pitch, roll) where yaw=rotation around z, pitch=rotation
        // around x, roll=rotation around y
        //
        // Camera orientation assumptions (DECODE Field Standard):
        // Per FTC SDK v11.0 and DECODE Competition Manual:
        // - pitch=0 means camera optical axis is horizontal (pointing forward at horizon level)
        // - This is correct for a standard front-facing webcam mount
        // - pitch=-90 would mean camera pointing straight down at the floor (INCORRECT for navigation)
        // - yaw=0 means camera pointing forward relative to robot (0° in robot frame)
        // - roll=0 means camera is not rotated around its optical axis
        // IMPORTANT: Using pitch=0 ensures ftcPose X/Y are correctly aligned:
        //   - ftcPose.x = sideways offset (right is positive, matches field X axis)
        //   - ftcPose.y = forward distance (away is positive, matches field Y axis)
        Position cameraPosition = new Position(DistanceUnit.INCH, 0, 0, CAMERA_HEIGHT, 0);
        YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0, 0, 0, 0);
        //
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawTagID(true) // Show tag ID numbers
                .setDrawTagOutline(true) // Draw colored border around detected tags
                .setDrawAxes(true) // Show RGB axes at tag center
                .setDrawCubeProjection(true) // Show 3D cube projection
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11) // DECODE uses 36h11 tag family
                .setCameraPose(cameraPosition, cameraOrientation) // Set camera position and orientation
                .setLensIntrinsics(540.752,540.752,306.681, 240.842)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

        // Stream the VisionPortal feed to FTC Dashboard (VisionPortal implements
        // CameraStreamSource)
        // Reduced to 15 FPS to avoid DS lag on weaker hardware
         FtcDashboard.getInstance().startCameraStream(visionPortal, 15);

        // Set camera controls after vision portal is initialized
        // This will be done asynchronously when camera is ready
    }

    /**
     * Set camera exposure and gain for optimal AprilTag detection.
     * Lower exposure reduces motion blur, higher gain compensates for low light.
     * 
     * @param exposureMs Exposure time in milliseconds (typically 1-10ms)
     * @param gain       Camera gain (0-255, typically 50-200)
     * @return True if controls were successfully set
     */
    public boolean setCameraControls(int exposureMs, int gain) {
        if (visionPortal == null) {
            return false;
        }

        // Wait for camera to be ready
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            return false; // Camera not ready yet
        }

        try {
            // Set exposure control
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                try {
                    Thread.sleep(50); // Allow mode change to take effect
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    return false;
                }
            }
            exposureControl.setExposure((long) exposureMs, TimeUnit.MILLISECONDS);

            // Set gain control
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(Range.clip(gain, gainControl.getMinGain(), gainControl.getMaxGain()));

            this.cameraExposureMs = exposureMs;
            this.cameraGain = gain;
            this.cameraControlsSet = true;

            if (telemetry != null) {
                telemetry.addData("Camera Controls", "Exposure: %d ms, Gain: %d", exposureMs, gain);
            }

            return true;
        } catch (Exception e) {
            if (telemetry != null) {
                telemetry.addData("Camera Control Error", e.getMessage());
            }
            return false;
        }
    }

    /**
     * Initialize camera controls with default optimized settings.
     * Call this after vision portal is initialized and streaming.
     */
    public void initializeCameraControls() {
        if (!cameraControlsSet && visionPortal != null) {
            // Try to set controls, will retry if camera not ready
            if (setCameraControls(cameraExposureMs, cameraGain)) {
                cameraControlsSet = true;
            }
        }
    }

    /**
     * Get current camera exposure setting
     */
    public int getCameraExposure() {
        return cameraExposureMs;
    }

    /**
     * Get current camera gain setting
     */
    public int getCameraGain() {
        return cameraGain;
    }

    /**
     * Get the best AprilTag detection for localization.
     * 
     * DECODE-Compliant: Only returns alliance goal tags (IDs 20 and 24).
     * Ignores obelisk tags (21, 22, 23) as they are not recommended for
     * navigation per DECODE Competition Manual.
     * 
     * Filters detections by distance and confidence, returns closest reliable
     * detection. Uses FTC SDK pose data: range = direct distance to tag center.
     * 
     * @return Best alliance goal detection or null if none found
     */
    public AprilTagDetection getBestDetection() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        if (detections.isEmpty())
            return null;

        // Filter detections by:
        // 1. DECODE alliance goal tags only (IDs 20 = Blue, 24 = Red)
        // 2. Distance thresholds for reliable detection
        // 3. Decision margin (tag classification confidence)
        // tag.ftcPose.range = direct distance to tag center (inches)
        // tag.decisionMargin = measure of tag classification confidence (NOT 0-1 scale,
        // can be > 1.0)
        return detections.stream()
                .filter(tag -> tag.id == 20 || tag.id == 24) // DECODE: alliance goals only
                .filter(tag -> tag.ftcPose.range >= MIN_DETECTION_DISTANCE)
                .filter(tag -> tag.ftcPose.range <= MAX_DETECTION_DISTANCE)
                .filter(tag -> tag.decisionMargin >= MIN_DECISION_MARGIN)
                .min(Comparator.comparingDouble(tag -> tag.ftcPose.range)) // Closest reliable detection
                .orElse(null);
    }

     /**
    * Returns all detections without ID filtering.
    * Used for Motif detection where we need IDs 21, 22, 23.
    */
    public List<AprilTagDetection> getRawDetections() {
    return aprilTag.getDetections();
    }

    /**
     * Get all valid AprilTag detections.
     * 
     * DECODE-Compliant: Only returns alliance goal tags (IDs 20 and 24).
     * 
     * @return List of valid alliance goal detections
     */
    public List<AprilTagDetection> getAllDetections() {
        return aprilTag.getDetections().stream()
                .filter(tag -> tag.id == 20 || tag.id == 24) // DECODE: alliance goals only
                .filter(tag -> tag.ftcPose.range >= MIN_DETECTION_DISTANCE)
                .filter(tag -> tag.ftcPose.range <= MAX_DETECTION_DISTANCE)
                .filter(tag -> tag.decisionMargin >= MIN_DECISION_MARGIN)
                .collect(java.util.stream.Collectors.toList());
    }

    /**
     * Get field position of an AprilTag by its ID
     * 
     * Array structure: {tagId, x_inches, y_inches, heading_degrees}
     * This method is robust to array structure changes as long as index 0 is tagId.
     * 
     * @param tagId AprilTag ID
     * @return Field position as {x, y, heading} or null if not found
     */
    private double[] getAprilTagFieldPosition(int tagId) {
        for (double[] position : FieldConstants.APRILTAG_POSITIONS) {
            if (position == null || position.length < 4) {
                continue; // Skip invalid entries
            }
            if ((int) position[0] == tagId) {
                return new double[] { position[1], position[2], position[3] };
            }
        }
        return null;
    }

    /**
     * Calculate robot's field position based on AprilTag detection WITHOUT updating
     * drive subsystem.
     * 
     * DECODE-Compliant Localization (FTC SDK v11.0 + DECODE Manual Page 72):
     * 
     * Coordinate Systems:
     * - Camera Frame: X=right, Y=forward (from camera lens perspective)
     * - Field Frame: X=toward Blue wall, Y=toward Audience (standard FTC)
     * - ftcPose reports the vector FROM camera TO tag in camera's local frame
     * - ftcPose.yaw = tag's rotation relative to camera (how much tag is "twisted")
     * 
     * Key Insight: The camera and tag face each other:
     * - Tag heading (field) is the direction the tag's normal vector points
     * - Camera heading is OPPOSITE: camera heading = tag heading + 180° - tag's relative yaw
     * - This accounts for the tag being on the wall and camera viewing it from the robot
     * 
     * Transformation Steps:
     * 1. Extract camera-relative measurements (already in camera frame)
     * 2. Calculate camera's absolute heading in field frame
     * 3. Rotate camera-relative vector to field frame using camera's heading
     * 4. Compute robot position = tag position - rotated vector
     * 
     * @param detection AprilTag detection (must be verified as valid alliance goal tag)
     * @return Robot pose as {x, y, heading} or null if calculation fails
     */
    private double[] calculateRobotPoseWithoutUpdating(AprilTagDetection detection) {
        if (detection == null)
            return null;

        // Only process DECODE alliance goal tags (IDs 20 and 24)
        // Obelisk tags (21, 22, 23) are NOT recommended for navigation per DECODE Manual
        if (detection.id != 20 && detection.id != 24) {
            if (telemetry != null) {
                telemetry.addData("AprilTag Warning", "Tag ID %d is not a DECODE alliance goal", detection.id);
            }
            return null;
        }

        double[] tagFieldPos = getAprilTagFieldPosition(detection.id);
        if (tagFieldPos == null) {
            if (telemetry != null) {
                telemetry.addData("AprilTag Error", "Unknown tag ID: %d", detection.id);
            }
            return null;
        }

        // ===== STEP 1: Extract camera-relative measurements =====
        // These vectors are in the camera's local coordinate frame
        double relX = detection.ftcPose.x;   // Sideways offset (right = positive)
        double relY = detection.ftcPose.y;   // Forward distance (away = positive)
        double relYaw = Math.toRadians(detection.ftcPose.yaw); // Tag rotation relative to camera

        // ===== STEP 2: Get tag's absolute field position and orientation =====
        double tagX = tagFieldPos[0];        // Tag X position in field (inches)
        double tagY = tagFieldPos[1];        // Tag Y position in field (inches)
        double tagHeading = Math.toRadians(tagFieldPos[2]); // Tag facing direction (radians)

        // ===== STEP 3: Calculate camera's absolute heading in field frame =====
        // Key relationship: Camera and tag face each other (opposite directions)
        // Camera heading = Tag heading + π (180°) - relative yaw
        // The subtraction of relYaw accounts for the tag's rotation relative to the camera
        double cameraHeading = tagHeading + Math.PI - relYaw;

        // ===== STEP 4: Transform camera-to-tag vector to field frame =====
        // Standard 2D rotation transformation:
        // [fieldX]   [cos(θ)  -sin(θ)] [relX]
        // [fieldY] = [sin(θ)   cos(θ)] [relY]
        // where θ = cameraHeading
        double cos_heading = Math.cos(cameraHeading);
        double sin_heading = Math.sin(cameraHeading);
        
        double fieldDX = relX * cos_heading - relY * sin_heading;
        double fieldDY = relX * sin_heading + relY * cos_heading;

        // ===== STEP 5: Calculate robot's field position =====
        // The rotated vector points FROM camera TO tag
        // Therefore: robot position = tag position - vector
        double robotX = tagX - fieldDX;
        double robotY = tagY - fieldDY;

        // Camera's heading is the robot's heading (camera is centered on robot)
        double robotHeading = cameraHeading;

        return new double[] { robotX, robotY, robotHeading };
    }

    /**
     * Calculate robot's field position based on AprilTag detection
     * 
     * This method updates the drive subsystem. For triangulation, use
     * calculateRobotPoseWithoutUpdating().
     * 
     * @param detection AprilTag detection
     * @return Robot's field position as TileCoordinate, or null if calculation
     *         fails
     */
    public TileCoordinate calculateRobotPosition(AprilTagDetection detection) {
        double[] pose = calculateRobotPoseWithoutUpdating(detection);
        if (pose == null) {
            return null;
        }

        // Create and return tile coordinate
        TileCoordinate robotPos = new TileCoordinate(pose[0], pose[1]);

        // Update drive subsystem with calculated position and heading
        if (driveSubsystem != null) {
            driveSubsystem.setPosition(robotPos);
            driveSubsystem.setHeading(pose[2]);
        }

        return robotPos;
    }

    /**
     * Update robot's position using AprilTag localization
     * 
     * @return True if position was successfully updated, false otherwise
     */
    public boolean updateRobotPosition() {
        AprilTagDetection bestDetection = getBestDetection();
        if (bestDetection == null) {
            if (telemetry != null) {
                telemetry.addData("AprilTag Localization", "No valid detections");
            }
            return false;
        }

        TileCoordinate robotPos = calculateRobotPosition(bestDetection);
        if (robotPos == null) {
            if (telemetry != null) {
                telemetry.addData("AprilTag Localization", "Position calculation failed");
            }
            return false;
        }

        if (telemetry != null) {
            telemetry.addData("AprilTag Localization", "Success");
            telemetry.addData("Detected Tag", "ID: %d, Range: %.1f", bestDetection.id, bestDetection.ftcPose.range);
            telemetry.addData("Robot Position", robotPos.getTilePosition());
            telemetry.addData("Robot Tab", robotPos.getTabPosition());
            telemetry.addData("Field Position", "X: %.1f, Y: %.1f", robotPos.getX(), robotPos.getY());
        }

        return true;
    }

    /**
     * Get robot's current tile position based on AprilTag localization
     * 
     * @return Current tile position or null if not localized
     */
    public TileCoordinate getCurrentTilePosition() {
        if (updateRobotPosition()) {
            return driveSubsystem.getCurrentPosition();
        }
        return null;
    }

    /**
     * Check if robot is currently localized using AprilTags
     * 
     * @return True if localized, false otherwise
     */
    public boolean isLocalized() {
        return getBestDetection() != null;
    }

    /**
     * Get detailed localization information
     * 
     * @return Localization info string
     */
    public String getLocalizationInfo() {
        List<AprilTagDetection> detections = getAllDetections();
        if (detections.isEmpty()) {
            return "No AprilTag detections";
        }

        StringBuilder info = new StringBuilder();
        info.append(String.format("Detected %d tags: ", detections.size()));

        for (AprilTagDetection detection : detections) {
            info.append(String.format("ID%d(%.1f) ", detection.id, detection.ftcPose.range));
        }

        TileCoordinate currentPos = getCurrentTilePosition();
        if (currentPos != null) {
            info.append(String.format("- Position: %s", currentPos.getTilePosition()));
        }

        return info.toString();
    }

    /**
     * Update telemetry with AprilTag localization information
     */
    public void updateLocalizationTelemetry() {
        if (telemetry == null)
            return;

        List<AprilTagDetection> detections = aprilTag.getDetections();
        telemetry.addData("AprilTag Detections", detections.size());

        if (detections.isEmpty()) {
            telemetry.addData("Localization Status", "No detections");
            return;
        }

        AprilTagDetection best = getBestDetection();
        if (best != null) {
            telemetry.addData("Best Detection", "ID: %d, Range: %.1f, Confidence: %.2f",
                    best.id, best.ftcPose.range, best.decisionMargin);
        }

        TileCoordinate currentPos = getCurrentTilePosition();
        if (currentPos != null) {
            telemetry.addData("Robot Tile", currentPos.getTilePosition());
            telemetry.addData("Robot Tab", currentPos.getTabPosition());
            telemetry.addData("Field Position", "X: %.1f, Y: %.1f", currentPos.getX(), currentPos.getY());
            telemetry.addData("Robot Heading", "%.1f°", Math.toDegrees(driveSubsystem.getCurrentHeading()));
        } else {
            telemetry.addData("Localization Status", "Failed to calculate position");
        }
    }

    /**
     * Close the vision portal
     */
    public void closeVision() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }

    /**
     * Get the AprilTag processor for advanced usage
     * 
     * @return AprilTag processor instance
     */
    public AprilTagProcessor getAprilTagProcessor() {
        return aprilTag;
    }

    /**
     * Get the vision portal for advanced usage
     * 
     * @return Vision portal instance
     */
    public VisionPortal getVisionPortal() {
        return visionPortal;
    }

    // ==================== DECODE FIELD SPECIFIC METHODS ====================

    /**
     * Check if the detected AprilTag is from the blue alliance goal
     * 
     * @param detection AprilTag detection
     * @return True if blue alliance goal (Tag ID 20)
     */
    public boolean isBlueAllianceGoal(AprilTagDetection detection) {
        return detection != null && detection.id == 20;
    }

    /**
     * Check if the detected AprilTag is from the red alliance goal
     * 
     * @param detection AprilTag detection
     * @return True if red alliance goal (Tag ID 24)
     */
    public boolean isRedAllianceGoal(AprilTagDetection detection) {
        return detection != null && detection.id == 24;
    }

    /**
     * Check if the detected AprilTag is from the obelisk (not recommended for
     * navigation)
     * 
     * @param detection AprilTag detection
     * @return True if obelisk tag (IDs 21, 22, 23)
     */
    public boolean isObeliskTag(AprilTagDetection detection) {
        return detection != null && (detection.id == 21 || detection.id == 22 || detection.id == 23);
    }

    /**
     * Get the alliance color of the detected AprilTag
     * 
     * @param detection AprilTag detection
     * @return "Blue", "Red", "Obelisk", or "Unknown"
     */
    public String getAllianceColor(AprilTagDetection detection) {
        if (detection == null)
            return "Unknown";

        if (isBlueAllianceGoal(detection))
            return "Blue";
        if (isRedAllianceGoal(detection))
            return "Red";
        if (isObeliskTag(detection))
            return "Obelisk";

        return "Unknown";
    }

    /**
     * Get the best detection from alliance goals only (excludes obelisk tags)
     * 
     * @return Best alliance goal detection or null if none found
     */
    public AprilTagDetection getBestAllianceGoalDetection() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        if (detections.isEmpty())
            return null;

        // Filter for alliance goals only (exclude obelisk tags)
        return detections.stream()
                .filter(tag -> tag.id == 20 || tag.id == 24) // Only blue and red alliance goals
                .filter(tag -> tag.ftcPose.range >= MIN_DETECTION_DISTANCE)
                .filter(tag -> tag.ftcPose.range <= MAX_DETECTION_DISTANCE)
                .filter(tag -> tag.decisionMargin >= MIN_DECISION_MARGIN)
                .min(Comparator.comparingDouble(tag -> tag.ftcPose.range)) // Closest reliable detection
                .orElse(null);
    }

    /**
     * Update robot position using only alliance goal detections (recommended for
     * DECODE)
     * 
     * @return True if position was successfully updated, false otherwise
     */
    public boolean updateRobotPositionFromAllianceGoals() {
        AprilTagDetection bestDetection = getBestAllianceGoalDetection();
        if (bestDetection == null) {
            if (telemetry != null) {
                telemetry.addData("AprilTag Localization", "No alliance goal detections");
            }
            return false;
        }

        TileCoordinate robotPos = calculateRobotPosition(bestDetection);
        if (robotPos == null) {
            if (telemetry != null) {
                telemetry.addData("AprilTag Localization", "Position calculation failed");
            }
            return false;
        }

        if (telemetry != null) {
            String alliance = getAllianceColor(bestDetection);
            telemetry.addData("AprilTag Localization", "Success - %s Alliance Goal", alliance);
            telemetry.addData("Detected Tag", "ID: %d, Range: %.1f", bestDetection.id, bestDetection.ftcPose.range);
            telemetry.addData("Robot Position", robotPos.getTilePosition());
            telemetry.addData("Robot Tab", robotPos.getTabPosition());
            telemetry.addData("Field Position", "X: %.1f, Y: %.1f", robotPos.getX(), robotPos.getY());
            telemetry.addData("Expected Start", "Red=D1, Blue=C1");
        }

        return true;
    }

    /**
     * Update robot position using triangulation from both alliance goal tags
     * when available. Falls back to best single alliance goal detection when
     * only one is visible.
     *
     * Triangulation approach:
     * - Compute independent robot poses from each valid detection (IDs 20, 24)
     * - Fuse by simple averaging of positions and headings
     * - If only one detection available, use that single pose
     *
     * @return True if position was successfully updated, false otherwise
     */
    public boolean updateRobotPositionFromTriangulation() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        if (detections.isEmpty()) {
            if (telemetry != null)
                telemetry.addData("AprilTag Localization", "No detections");
            return false;
        }

        // Filter to valid alliance goal detections only (IDs 20 and 24) and within
        // thresholds
        List<AprilTagDetection> goalDetections = detections.stream()
                .filter(tag -> tag.id == 20 || tag.id == 24)
                .filter(tag -> tag.ftcPose.range >= MIN_DETECTION_DISTANCE)
                .filter(tag -> tag.ftcPose.range <= MAX_DETECTION_DISTANCE)
                .filter(tag -> tag.decisionMargin >= MIN_DECISION_MARGIN)
                .collect(java.util.stream.Collectors.toList());

        if (goalDetections.isEmpty()) {
            if (telemetry != null)
                telemetry.addData("AprilTag Localization", "No alliance goal detections");
            return false;
        }

        TileCoordinate poseFrom20 = null;
        TileCoordinate poseFrom24 = null;
        double headingFrom20 = Double.NaN;
        double headingFrom24 = Double.NaN;

        for (AprilTagDetection det : goalDetections) {
            // Use calculateRobotPoseWithoutUpdating to avoid mutating driveSubsystem
            // multiple times
            double[] poseData = calculateRobotPoseWithoutUpdating(det);
            if (poseData == null)
                continue;

            TileCoordinate pose = new TileCoordinate(poseData[0], poseData[1]);
            double computedHeading = poseData[2];

            if (det.id == 20) {
                poseFrom20 = pose;
                headingFrom20 = computedHeading;
            } else if (det.id == 24) {
                poseFrom24 = pose;
                headingFrom24 = computedHeading;
            }
        }

        TileCoordinate fused;
        double fusedHeading;

        if (poseFrom20 != null && poseFrom24 != null) {
            // Simple average fusion
            double x = (poseFrom20.getX() + poseFrom24.getX()) / 2.0;
            double y = (poseFrom20.getY() + poseFrom24.getY()) / 2.0;
            fused = new TileCoordinate(x, y);
            // Average headings taking wrap-around into account
            double s = Math.sin(headingFrom20) + Math.sin(headingFrom24);
            double c = Math.cos(headingFrom20) + Math.cos(headingFrom24);
            fusedHeading = Math.atan2(s, c);
        } else if (poseFrom20 != null) {
            fused = poseFrom20;
            fusedHeading = headingFrom20;
        } else if (poseFrom24 != null) {
            fused = poseFrom24;
            fusedHeading = headingFrom24;
        } else {
            if (telemetry != null)
                telemetry.addData("AprilTag Localization", "No valid triangulation poses");
            return false;
        }

        if (driveSubsystem != null) {
            driveSubsystem.setPosition(fused);
            driveSubsystem.setHeading(fusedHeading);
        }

        if (telemetry != null) {
            telemetry.addData("AprilTag Triangulation", "Success");
            telemetry.addData("Field Position", "X: %.1f, Y: %.1f", fused.getX(), fused.getY());
            telemetry.addData("Heading", "%.1f°", Math.toDegrees(fusedHeading));
        }

        return true;
    }

    /**
     * Calculate alignment corrections for shooting with reduced oscillation.
     * Uses deadband and rate limiting to prevent jittery movements.
     * 
     * @param tag             AprilTag detection (must be alliance goal tag)
     * @param desiredDistance Desired forward distance to tag (inches)
     * @param desiredAngle    Desired angle offset (degrees, positive =
     *                        counterclockwise)
     * @param deadbandX       Deadband for X offset (inches)
     * @param deadbandY       Deadband for Y distance (inches)
     * @param deadbandAngle   Deadband for angle (degrees)
     * @param kPStrafe        Proportional gain for strafe correction
     * @param kPForward       Proportional gain for forward correction
     * @param kPRot           Proportional gain for rotation correction
     * @param maxPower        Maximum power output (0-1)
     * @return Array [strafePower, forwardPower, turnPower, isAligned] or null if
     *         tag not found
     */
    public double[] calculateAlignmentCorrections(AprilTagDetection tag,
            double desiredDistance,
            double desiredAngle,
            double deadbandX,
            double deadbandY,
            double deadbandAngle,
            double kPStrafe,
            double kPForward,
            double kPRot,
            double maxPower) {
        if (tag == null || (tag.id != 20 && tag.id != 24)) {
            return null; // Not an alliance goal tag
        }

        double xOffset = tag.ftcPose.x; // Left/right offset
        double yDistance = tag.ftcPose.y; // Forward distance
        double yaw = tag.ftcPose.yaw; // Tag rotation relative to camera

        double forwardError = yDistance - desiredDistance;
        double angleError = yaw + desiredAngle; // Add desired angle offset

        // Apply deadbands
        double strafePower = 0;
        double forwardPower = 0;
        double turnPower = 0;

        if (Math.abs(xOffset) > deadbandX) {
            strafePower = kPStrafe * xOffset;
        }

        if (Math.abs(forwardError) > deadbandY) {
            forwardPower = kPForward * forwardError;
        }

        // Prioritize angle correction first
        if (Math.abs(angleError) > deadbandAngle) {
            turnPower = kPRot * angleError;
            // Don't move forward/strafe while correcting angle
            strafePower = 0;
            forwardPower = 0;
        }

        // Clip to max power
        strafePower = Range.clip(strafePower, -maxPower, maxPower);
        forwardPower = Range.clip(forwardPower, -maxPower, maxPower);
        turnPower = Range.clip(turnPower, -maxPower, maxPower);

        // Check if aligned (all corrections are zero)
        boolean isAligned = (strafePower == 0 && forwardPower == 0 && turnPower == 0);

        return new double[] { strafePower, forwardPower, turnPower, isAligned ? 1.0 : 0.0 };
    }

    /**
     * Update telemetry with DECODE-specific localization information
     */
    public void updateDECODELocalizationTelemetry() {
        if (telemetry == null)
            return;

        List<AprilTagDetection> detections = aprilTag.getDetections();
        telemetry.addData("AprilTag Detections", detections.size());

        if (detections.isEmpty()) {
            telemetry.addData("Localization Status", "No detections");
            return;
        }

        // Count different types of detections
        long blueGoals = detections.stream().filter(this::isBlueAllianceGoal).count();
        long redGoals = detections.stream().filter(this::isRedAllianceGoal).count();
        long obeliskTags = detections.stream().filter(this::isObeliskTag).count();

        telemetry.addData("Blue Alliance Goals", blueGoals);
        telemetry.addData("Red Alliance Goals", redGoals);
        telemetry.addData("Obelisk Tags", obeliskTags);

        AprilTagDetection best = getBestAllianceGoalDetection();
        if (best != null) {
            String alliance = getAllianceColor(best);
            telemetry.addData("Best Detection", "%s Alliance Goal - ID: %d, Range: %.1f, Confidence: %.2f",
                    alliance, best.id, best.ftcPose.range, best.decisionMargin);
        } else {
            telemetry.addData("Best Detection", "No alliance goal detected");
        }

        TileCoordinate currentPos = getCurrentTilePosition();
        if (currentPos != null) {
            telemetry.addData("Robot Tile", currentPos.getTilePosition());
            telemetry.addData("Robot Tab", currentPos.getTabPosition());
            telemetry.addData("Field Position", "X: %.1f, Y: %.1f", currentPos.getX(), currentPos.getY());
            telemetry.addData("Robot Heading", "%.1f°", Math.toDegrees(driveSubsystem.getCurrentHeading()));
        } else {
            telemetry.addData("Localization Status", "Failed to calculate position");
        }
    }
}
