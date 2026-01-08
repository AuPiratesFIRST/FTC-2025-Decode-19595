package org.firstinspires.ftc.teamcode.SubSystems.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.teamcode.SubSystems.Drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Vision.AprilTagNavigator;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Camera Exposure/Gain Tuner", group = "Test")
public class CameraExposureGainTuner extends LinearOpMode {

    private DriveSubsystem drive;
    private AprilTagNavigator aprilTag;

    private boolean tuningActive = false;
    private ElapsedTime stateTimer = new ElapsedTime();

    // === OPTIMIZED RANGES FOR ~2 MINUTE TEST ===
    // Test Exposure: 2, 5, 8, 11, 14, 17, 20 (7 steps)
    // Test Gain: 10, 35, 60, 85, 110, 135, ..., 255 (Max 10 steps)
    // Total: ~70 combinations * 1.5s = ~105 seconds
    private int[] exposureSteps = {2, 5, 8, 11, 14, 17, 20};
    private int gainStepSize = 25;
    private int minGain = 10;
    private int maxGain = 250;

    // Test State Machine
    private enum TestState {
        INIT_NEW_SETTING,
        WAIT_FOR_HARDWARE_SETTLE, // Critical: Wait for camera to adjust
        COLLECT_DATA,
        FINALIZE
    }
    private TestState currentTestState = TestState.INIT_NEW_SETTING;

    // Iterators
    private int exposureIndex = 0;
    private int currentTestGain = minGain;

    // Stats
    private List<TestResult> testResults = new ArrayList<>();
    private TestResult bestResult = null;
    private int detectionCount = 0;
    private int loopCount = 0;
    private double maxRange = 0;

    private static class TestResult {
        int exposure, gain;
        double score;
        
        // Simple string for telemetry
        public String toString() {
            return String.format("E:%d G:%d -> Score: %.1f", exposure, gain, score);
        }
    }

    // Controls
    private int manualExposure = 14;
    private int manualGain = 25;
    private boolean lastA = false, lastB = false, lastY = false;
    private boolean lastUp = false, lastDown = false, lastLeft = false, lastRight = false;

    @Override
    public void runOpMode() {
        drive = new DriveSubsystem(hardwareMap, telemetry);
        aprilTag = new AprilTagNavigator(drive, hardwareMap, telemetry);

        telemetry.addLine("Camera Tuner Ready");
        telemetry.addLine("Press A to Auto-Tune (approx 2 mins)");
        telemetry.addLine("Press D-Pad for Manual Tuning");
        telemetry.update();

        waitForStart();
        aprilTag.initializeCameraControls();

        while (opModeIsActive()) {
            handleInput();

            if (tuningActive) {
                runAutoTuningLogic();
            } else {
                runManualModeLogic();
            }

            telemetry.update();
        }
        aprilTag.closeVision();
    }

    private void runAutoTuningLogic() {
        switch (currentTestState) {
            case INIT_NEW_SETTING:
                // Apply new settings
                int exp = exposureSteps[exposureIndex];
                int gn = currentTestGain;
                
                boolean success = aprilTag.setCameraControls(exp, gn);
                if (success) {
                    stateTimer.reset();
                    currentTestState = TestState.WAIT_FOR_HARDWARE_SETTLE;
                }
                telemetry.addData("Status", "Setting E:%d G:%d", exp, gn);
                break;

            case WAIT_FOR_HARDWARE_SETTLE:
                // Wait 400ms for the camera sensor to actually apply the gain/exposure
                // Otherwise we capture dark/blurry frames from the previous setting
                if (stateTimer.milliseconds() > 400) {
                    // Reset counters for actual data collection
                    detectionCount = 0;
                    loopCount = 0;
                    maxRange = 0;
                    stateTimer.reset();
                    currentTestState = TestState.COLLECT_DATA;
                }
                telemetry.addData("Status", "Settling Hardware...");
                break;

            case COLLECT_DATA:
                // Run for 1.0 second (sufficient for checking detection)
                if (stateTimer.seconds() < 1.0) {
                    aprilTag.updateRobotPositionFromAllianceGoals();
                    AprilTagDetection tag = aprilTag.getBestAllianceGoalDetection();
                    loopCount++;
                    
                    if (tag != null) {
                        detectionCount++;
                        if (tag.ftcPose.range > maxRange) maxRange = tag.ftcPose.range;
                    }
                } else {
                    recordResult();
                    advanceToNextSetting();
                }
                telemetry.addData("Status", "Collecting Data...");
                break;

            case FINALIZE:
                tuningActive = false;
                if (bestResult != null) {
                    manualExposure = bestResult.exposure;
                    manualGain = bestResult.gain;
                    aprilTag.setCameraControls(manualExposure, manualGain);
                }
                break;
        }

        // Progress Telemetry
        telemetry.addData("Progress", "Exp Idx: %d/%d | Gain: %d", 
            exposureIndex, exposureSteps.length, currentTestGain);
    }

    private void recordResult() {
        TestResult res = new TestResult();
        res.exposure = exposureSteps[exposureIndex];
        res.gain = currentTestGain;
        
        // Score Algorithm:
        // 1. Consistency: (detections / loops) * 100
        // 2. Range Bonus: maxRange * 2
        // 3. Penalty for high exposure (motion blur risk): -exposure * 2
        double consistency = (loopCount > 0) ? ((double) detectionCount / loopCount) * 100 : 0;
        res.score = consistency + (maxRange * 2.0) - (res.exposure * 2.0);

        testResults.add(res);

        if (bestResult == null || res.score > bestResult.score) {
            bestResult = res;
        }
    }

    private void advanceToNextSetting() {
        currentTestGain += gainStepSize;
        if (currentTestGain > maxGain) {
            currentTestGain = minGain;
            exposureIndex++;
        }

        if (exposureIndex >= exposureSteps.length) {
            currentTestState = TestState.FINALIZE;
        } else {
            currentTestState = TestState.INIT_NEW_SETTING;
        }
    }

    private void runManualModeLogic() {
        // Just show current performance
        aprilTag.updateRobotPositionFromAllianceGoals();
        AprilTagDetection tag = aprilTag.getBestAllianceGoalDetection();
        
        telemetry.addLine("=== MANUAL MODE ===");
        telemetry.addData("Current Exp", manualExposure);
        telemetry.addData("Current Gain", manualGain);
        telemetry.addData("Tag Detected", tag != null ? "YES" : "NO");
        if(tag != null) {
            telemetry.addData("Range", "%.1f", tag.ftcPose.range);
        }

        if (bestResult != null) {
            telemetry.addLine("\n=== BEST FOUND ===");
            telemetry.addLine(bestResult.toString());
        }
    }

    private void handleInput() {
        if (gamepad1.a && !lastA) {
            tuningActive = true;
            exposureIndex = 0;
            currentTestGain = minGain;
            currentTestState = TestState.INIT_NEW_SETTING;
            testResults.clear();
            bestResult = null;
        }
        if (gamepad1.b && !lastB) {
            tuningActive = false; // Stop early
        }
        
        // Manual Adjustments
        boolean changed = false;
        if (!tuningActive) {
            if (gamepad1.dpad_up && !lastUp) { manualExposure++; changed = true; }
            if (gamepad1.dpad_down && !lastDown) { manualExposure--; changed = true; }
            if (gamepad1.dpad_right && !lastRight) { manualGain += 5; changed = true; }
            if (gamepad1.dpad_left && !lastLeft) { manualGain -= 5; changed = true; }
            
            if (changed) {
                aprilTag.setCameraControls(manualExposure, manualGain);
            }
        }

        lastA = gamepad1.a; lastB = gamepad1.b; lastY = gamepad1.y;
        lastUp = gamepad1.dpad_up; lastDown = gamepad1.dpad_down;
        lastLeft = gamepad1.dpad_left; lastRight = gamepad1.dpad_right;
    }
}