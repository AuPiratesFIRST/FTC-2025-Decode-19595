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
import com.qualcomm.robotcore.util.Range;

import java.util.concurrent.TimeUnit;
import java.util.List;
import java.util.Comparator;
import java.util.stream.Collectors;

import org.firstinspires.ftc.teamcode.SubSystems.Drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Drive.TileCoordinate;
import org.firstinspires.ftc.teamcode.Constants.FieldConstants;

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

    // Fast settings for long-range stability
    private int cameraExposureMs = 3;
    private int cameraGain = 120;
    private boolean cameraControlsSet = false;

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
     * RESTORED: Updates robot position based on the best alliance goal tag (IDs 20/24).
     */
    public boolean updateRobotPositionFromAllianceGoals() {
        return updateRobotPosition();
    }

    /**
     * RESTORED: General position update method used by MoveToTile and other tests.
     */
    public boolean updateRobotPosition() {
        AprilTagDetection best = getBestDetection();
        if (best == null) return false;
        return calculateRobotPosition(best) != null;
    }

    /**
     * RESTORED: Fuses multiple tag detections into one position.
     */
    public boolean updateRobotPositionFromTriangulation() {
        List<AprilTagDetection> goalDetections = aprilTag.getDetections().stream()
                .filter(tag -> (tag.id == 20 || tag.id == 24) && tag.decisionMargin >= MIN_DECISION_MARGIN)
                .collect(Collectors.toList());

        if (goalDetections.isEmpty()) return false;

        double sumX = 0, sumY = 0, sumSin = 0, sumCos = 0;
        int count = 0;

        for (AprilTagDetection det : goalDetections) {
            double[] pose = calculateRobotPoseWithoutUpdating(det);
            if (pose != null) {
                sumX += pose[0];
                sumY += pose[1];
                sumSin += Math.sin(pose[2]);
                sumCos += Math.cos(pose[2]);
                count++;
            }
        }

        if (count == 0) return false;

        TileCoordinate fusedPos = new TileCoordinate(sumX / count, sumY / count);
        double fusedHeading = Math.atan2(sumSin / count, sumCos / count);

        if (driveSubsystem != null) {
            driveSubsystem.setPosition(fusedPos);
            driveSubsystem.setHeading(fusedHeading);
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
            // Clamp exposure to hardware limits
            long clampedExposure = Range.clip(
                exposureMs,
                (int) exp.getMinExposure(TimeUnit.MILLISECONDS),
                (int) exp.getMaxExposure(TimeUnit.MILLISECONDS)
            );
            exp.setExposure(clampedExposure, TimeUnit.MILLISECONDS);
            
            GainControl g = visionPortal.getCameraControl(GainControl.class);
            g.setGain(Range.clip(gain, g.getMinGain(), g.getMaxGain()));
            return true;
        } catch (Exception e) { 
            if (telemetry != null) telemetry.addData("Camera Controls", "Failed to set");
            return false;
        }
    }

    public AprilTagDetection getBestDetection() {
        return aprilTag.getDetections().stream()
                .filter(tag -> tag.id == 20 || tag.id == 24)
                .filter(tag -> tag.ftcPose.range >= MIN_DETECTION_DISTANCE)  // Prevent near-field explosions
                .filter(tag -> tag.ftcPose.range <= MAX_DETECTION_DISTANCE)
                .filter(tag -> tag.decisionMargin >= MIN_DECISION_MARGIN)
                .min(Comparator.comparingDouble(tag -> tag.ftcPose.range / tag.decisionMargin))  // Favor close + confident
                .orElse(null);
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

        // Extract tag position components correctly: {id, x, y, headingDeg}
        double tagX = tagPos[1];
        double tagY = tagPos[2];
        double tagHeadingDeg = tagPos[3];

        double relX = detection.ftcPose.x;
        double relY = detection.ftcPose.y;
        
        // Camera heading in field frame
        double cameraHeading = Math.toRadians(tagHeadingDeg) + Math.PI - Math.toRadians(detection.ftcPose.yaw);

        double cosH = Math.cos(cameraHeading);
        double sinH = Math.sin(cameraHeading);

        // Transform camera-relative position to field frame
        double robotX = tagX - (relX * cosH - relY * sinH);
        double robotY = tagY - (relX * sinH + relY * cosH);

        return new double[]{robotX, robotY, cameraHeading};
    }

    public TileCoordinate getCurrentTilePosition() {
        AprilTagDetection best = getBestDetection();
        return (best == null) ? null : calculateRobotPosition(best);
    }

    public AprilTagDetection getBestAllianceGoalDetection() { return getBestDetection(); }
    public List<AprilTagDetection> getRawDetections() { return aprilTag.getDetections(); }
    public void closeVision() { if (visionPortal != null) visionPortal.close(); }

    public double[] calculateAlignmentCorrections(AprilTagDetection tag, double desiredDist, double desiredAngle,
                                                  double dbX, double dbY, double dbAngle,
                                                  double kPS, double kPF, double kPR, double maxP) {
        if (tag == null) return null;
        double fErr = tag.ftcPose.y - desiredDist;
        // Use normalized angle difference to prevent wraparound oscillation at ±180°
        double aErr = AngleUnit.normalizeDegrees(tag.ftcPose.yaw - desiredAngle);
        double sP = (Math.abs(tag.ftcPose.x) > dbX) ? tag.ftcPose.x * kPS : 0;
        double fP = (Math.abs(fErr) > dbY) ? fErr * kPF : 0;
        double tP = (Math.abs(aErr) > dbAngle) ? aErr * kPR : 0;
        return new double[]{Range.clip(sP, -maxP, maxP), Range.clip(fP, -maxP, maxP), Range.clip(tP, -maxP, maxP)};
    }
}