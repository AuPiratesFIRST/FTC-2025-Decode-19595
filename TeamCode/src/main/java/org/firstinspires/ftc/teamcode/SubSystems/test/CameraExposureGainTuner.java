package org.firstinspires.ftc.teamcode.SubSystems.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.teamcode.SubSystems.Drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Vision.AprilTagNavigator;

import java.util.ArrayList;
import java.util.List;

/**
 * Camera Exposure and Gain Auto-Tuning Test
 * 
 * Automatically tests different exposure and gain combinations to find optimal settings
 * for AprilTag detection. Tests detection rate, range, and reliability.
 * 
 * Controls:
 * - Gamepad 1 A: Start auto-tuning
 * - Gamepad 1 B: Stop tuning
 * - Gamepad 1 X: Test current settings manually
 * - Gamepad 1 Y: Reset to default settings
 * - D-Pad: Manual exposure/gain adjustment
 * 
 * The tuner will:
 * 1. Test exposure values from 1ms to 20ms
 * 2. Test gain values from 10 to 255
 * 3. Measure detection rate and range for each combination
 * 4. Recommend best settings based on detection performance
 */
@TeleOp(name = "Camera Exposure/Gain Tuner", group = "Test")
public class CameraExposureGainTuner extends LinearOpMode {

    private DriveSubsystem drive;
    private AprilTagNavigator aprilTag;

    private enum TuningMode {
        MANUAL,
        AUTO_EXPOSURE,
        AUTO_GAIN,
        AUTO_BOTH
    }

    private TuningMode mode = TuningMode.MANUAL;
    private boolean tuningActive = false;
    private ElapsedTime tuningTimer = new ElapsedTime();
    private ElapsedTime testTimer = new ElapsedTime();

    // Exposure range: 1ms to 20ms
    private int currentExposure = 14; // Default
    private int minExposure = 1;
    private int maxExposure = 20;

    // Gain range: 10 to 255
    private int currentGain = 25; // Default
    private int minGain = 10;
    private int maxGain = 255;

    // Test results
    private List<TestResult> testResults = new ArrayList<>();
    private TestResult bestResult = null;
    private int testExposure = 14;
    private int testGain = 25;
    private int testIndex = 0;

    // Button state
    private boolean aPressedLast = false;
    private boolean bPressedLast = false;
    private boolean xPressedLast = false;
    private boolean yPressedLast = false;
    private boolean dpadUpLast = false;
    private boolean dpadDownLast = false;
    private boolean dpadLeftLast = false;
    private boolean dpadRightLast = false;

    // Detection statistics
    private int detectionCount = 0;
    private int totalFrames = 0;
    private double maxRange = 0;
    private double avgRange = 0;
    private double rangeSum = 0;

    private static class TestResult {
        int exposure;
        int gain;
        double detectionRate;
        double maxRange;
        double avgRange;
        int totalDetections;
        int totalFrames;

        double score() {
            // Score based on detection rate (70%) and range (30%)
            return (detectionRate * 0.7) + ((maxRange / 200.0) * 0.3);
        }
    }

    @Override
    public void runOpMode() {
        drive = new DriveSubsystem(hardwareMap, telemetry);
        aprilTag = new AprilTagNavigator(drive, hardwareMap, telemetry);

        telemetry.addData("Status", "Initialized - Camera Tuner");
        telemetry.addData("Current Exposure", "%d ms", currentExposure);
        telemetry.addData("Current Gain", "%d", currentGain);
        telemetry.update();

        waitForStart();

        // Initialize camera controls
        aprilTag.initializeCameraControls();

        while (opModeIsActive()) {
            // Update AprilTag detection
            aprilTag.updateRobotPositionFromTriangulation();

            // Handle controls
            handleControls();

            // Run tuning if active
            if (tuningActive) {
                runAutoTuning();
            } else {
                // Manual mode - test current settings
                testCurrentSettings();
            }

            // Update telemetry
            updateTelemetry();
        }

        aprilTag.closeVision();
    }

    private void handleControls() {
        boolean a = gamepad1.a;
        boolean b = gamepad1.b;
        boolean x = gamepad1.x;
        boolean y = gamepad1.y;

        if (a && !aPressedLast) {
            // Start auto-tuning
            if (!tuningActive) {
                tuningActive = true;
                testResults.clear();
                testIndex = 0;
                tuningTimer.reset();
                testTimer.reset();
                detectionCount = 0;
                totalFrames = 0;
                maxRange = 0;
                rangeSum = 0;
                mode = TuningMode.AUTO_BOTH;
            }
        }
        if (b && !bPressedLast) {
            // Stop tuning
            tuningActive = false;
            if (bestResult != null) {
                // Apply best settings
                aprilTag.setCameraControls(bestResult.exposure, bestResult.gain);
                currentExposure = bestResult.exposure;
                currentGain = bestResult.gain;
            }
        }
        if (x && !xPressedLast) {
            // Test current settings manually
            testTimer.reset();
            detectionCount = 0;
            totalFrames = 0;
            maxRange = 0;
            rangeSum = 0;
        }
        if (y && !yPressedLast) {
            // Reset to defaults
            currentExposure = 14;
            currentGain = 25;
            aprilTag.setCameraControls(currentExposure, currentGain);
        }

        // Manual adjustment
        boolean dpadUp = gamepad1.dpad_up;
        boolean dpadDown = gamepad1.dpad_down;
        boolean dpadLeft = gamepad1.dpad_left;
        boolean dpadRight = gamepad1.dpad_right;

        if (dpadUp && !dpadUpLast && !tuningActive) {
            currentExposure = Math.min(maxExposure, currentExposure + 1);
            aprilTag.setCameraControls(currentExposure, currentGain);
        }
        if (dpadDown && !dpadDownLast && !tuningActive) {
            currentExposure = Math.max(minExposure, currentExposure - 1);
            aprilTag.setCameraControls(currentExposure, currentGain);
        }
        if (dpadRight && !dpadRightLast && !tuningActive) {
            currentGain = Math.min(maxGain, currentGain + 5);
            aprilTag.setCameraControls(currentExposure, currentGain);
        }
        if (dpadLeft && !dpadLeftLast && !tuningActive) {
            currentGain = Math.max(minGain, currentGain - 5);
            aprilTag.setCameraControls(currentExposure, currentGain);
        }

        aPressedLast = a;
        bPressedLast = b;
        xPressedLast = x;
        yPressedLast = y;
        dpadUpLast = dpadUp;
        dpadDownLast = dpadDown;
        dpadLeftLast = dpadLeft;
        dpadRightLast = dpadRight;
    }

    private void runAutoTuning() {
        // Test each exposure/gain combination for 2 seconds
        if (testTimer.seconds() < 2.0) {
            // Collect detection data
            aprilTag.updateRobotPositionFromAllianceGoals();
            AprilTagDetection tag = aprilTag.getBestAllianceGoalDetection();
            
            totalFrames++;
            if (tag != null) {
                detectionCount++;
                double range = tag.ftcPose.range;
                if (range > maxRange) maxRange = range;
                rangeSum += range;
            }
        } else {
            // Test period complete - save results
            double detectionRate = totalFrames > 0 ? (double) detectionCount / totalFrames : 0;
            double avgRange = detectionCount > 0 ? rangeSum / detectionCount : 0;

            TestResult result = new TestResult();
            result.exposure = testExposure;
            result.gain = testGain;
            result.detectionRate = detectionRate;
            result.maxRange = maxRange;
            result.avgRange = avgRange;
            result.totalDetections = detectionCount;
            result.totalFrames = totalFrames;
            testResults.add(result);

            // Update best result
            if (bestResult == null || result.score() > bestResult.score()) {
                bestResult = result;
            }

            // Move to next test combination
            testIndex++;
            testExposure = minExposure + (testIndex % (maxExposure - minExposure + 1));
            testGain = minGain + ((testIndex / (maxExposure - minExposure + 1)) % ((maxGain - minGain) / 5 + 1)) * 5;

            // Apply test settings
            aprilTag.setCameraControls(testExposure, testGain);

            // Reset counters
            testTimer.reset();
            detectionCount = 0;
            totalFrames = 0;
            maxRange = 0;
            rangeSum = 0;

            // Stop if we've tested enough combinations
            if (testIndex >= 100) { // Limit to 100 combinations
                tuningActive = false;
            }
        }
    }

    private void testCurrentSettings() {
        if (testTimer.seconds() < 5.0) {
            aprilTag.updateRobotPositionFromAllianceGoals();
            AprilTagDetection tag = aprilTag.getBestAllianceGoalDetection();
            
            totalFrames++;
            if (tag != null) {
                detectionCount++;
                double range = tag.ftcPose.range;
                if (range > maxRange) maxRange = range;
                rangeSum += range;
            }
        } else {
            // Reset test
            testTimer.reset();
            detectionCount = 0;
            totalFrames = 0;
            maxRange = 0;
            rangeSum = 0;
        }
    }

    private void updateTelemetry() {
        telemetry.addLine("=== CAMERA EXPOSURE/GAIN TUNER ===");
        telemetry.addData("Mode", tuningActive ? "AUTO-TUNING" : "MANUAL");
        telemetry.addData("Current Exposure", "%d ms", aprilTag.getCameraExposure());
        telemetry.addData("Current Gain", "%d", aprilTag.getCameraGain());
        telemetry.addLine();

        if (tuningActive) {
            telemetry.addData("Testing", "Exposure: %d ms, Gain: %d", testExposure, testGain);
            telemetry.addData("Progress", "%d/%d combinations tested", testIndex, 100);
            telemetry.addData("Current Test", "Detections: %d/%d frames (%.1f%%)",
                detectionCount, totalFrames, totalFrames > 0 ? (detectionCount * 100.0 / totalFrames) : 0);
            telemetry.addData("  Max Range", "%.1f\"", maxRange);
        } else {
            // Manual test results
            double detectionRate = totalFrames > 0 ? (double) detectionCount / totalFrames : 0;
            telemetry.addData("Test Results", "Detections: %d/%d frames (%.1f%%)",
                detectionCount, totalFrames, detectionRate * 100);
            telemetry.addData("  Max Range", "%.1f\"", maxRange);
            if (detectionCount > 0) {
                telemetry.addData("  Avg Range", "%.1f\"", rangeSum / detectionCount);
            }
        }

        if (bestResult != null) {
            telemetry.addLine();
            telemetry.addData("BEST SETTINGS", "Exposure: %d ms, Gain: %d", bestResult.exposure, bestResult.gain);
            telemetry.addData("  Detection Rate", "%.1f%%", bestResult.detectionRate * 100);
            telemetry.addData("  Max Range", "%.1f\"", bestResult.maxRange);
            telemetry.addData("  Score", "%.3f", bestResult.score());
        }

        telemetry.addLine();
        telemetry.addData("Controls", "A: Start Auto-Tune, B: Stop & Apply Best, X: Test Current, Y: Reset Defaults");
        telemetry.addData("  D-Pad", "Up/Down: Exposure, Left/Right: Gain");

        // AprilTag detection info
        aprilTag.updateDECODELocalizationTelemetry();
        telemetry.update();
    }
}

