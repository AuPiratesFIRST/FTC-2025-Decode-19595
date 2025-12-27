package org.firstinspires.ftc.teamcode.SubSystems.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.teamcode.SubSystems.Drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Vision.AprilTagNavigator;

/**
 * AprilTag Alignment Auto-Tuner
 * 
 * Helps tune PID values for auto alignment to reduce oscillation and jerking.
 * Displays real-time April tag detection data in inches.
 * 
 * Controls:
 * - Gamepad 1 Y: Toggle auto alignment (hold to test)
 * - Gamepad 1 A: Reset to default values
 * - Gamepad 1 B: Save current values as best
 * - Gamepad 1 X: Toggle between tuning modes (KP, Deadbands, Targets)
 * 
 * Tuning Mode 1 - Proportional Gains:
 * - D-Pad Up/Down: Adjust KP_STRAFE
 * - D-Pad Left/Right: Adjust KP_FORWARD
 * - Left Stick Y: Adjust KP_ROT
 * - Right Stick Y: Adjust MAX_AUTO_POWER
 * 
 * Tuning Mode 2 - Deadbands:
 * - D-Pad Up/Down: Adjust POSITION_DEADBAND
 * - D-Pad Left/Right: Adjust ANGLE_DEADBAND_DEG
 * 
 * Tuning Mode 3 - Target Values:
 * - D-Pad Up/Down: Adjust DESIRED_SHOOTING_DISTANCE
 * - D-Pad Left/Right: Adjust DESIRED_SHOOTING_ANGLE
 * 
 * Right Bumper: Fine adjustment mode (smaller steps)
 * Left Bumper: Coarse adjustment mode (larger steps)
 */
@TeleOp(name = "AprilTag Alignment Tuner", group = "Test")
public class AprilTagAlignmentTuner extends LinearOpMode {

    private DriveSubsystem drive;
    private AprilTagNavigator aprilTag;

    // === TUNING PARAMETERS ===
    // Current values being tested
    private double currentKPStrafe = 0.03;
    private double currentKPForward = 0.03;
    private double currentKPRot = 0.015;
    private double currentMaxPower = 0.40;
    private double currentPositionDeadband = 0.75;
    private double currentAngleDeadband = 1.5;
    private double currentDesiredDistance = 134.0; // inches
    private double currentDesiredAngle = 21.0; // degrees

    // Best values (saved)
    private double bestKPStrafe = 0.03;
    private double bestKPForward = 0.03;
    private double bestKPRot = 0.015;
    private double bestMaxPower = 0.40;
    private double bestPositionDeadband = 0.75;
    private double bestAngleDeadband = 1.5;
    private double bestDesiredDistance = 134.0;
    private double bestDesiredAngle = 21.0;

    // Default values
    private static final double DEFAULT_KP_STRAFE = 0.03;
    private static final double DEFAULT_KP_FORWARD = 0.03;
    private static final double DEFAULT_KP_ROT = 0.015;
    private static final double DEFAULT_MAX_POWER = 0.40;
    private static final double DEFAULT_POSITION_DEADBAND = 0.75;
    private static final double DEFAULT_ANGLE_DEADBAND = 1.5;
    private static final double DEFAULT_DESIRED_DISTANCE = 134.0;
    private static final double DEFAULT_DESIRED_ANGLE = 21.0;

    // Target tag ID (24 for red, 20 for blue)
    private int targetTagId = 24;

    // Tuning mode
    private enum TuningMode {
        PROPORTIONAL_GAINS,
        DEADBANDS,
        TARGET_VALUES
    }
    private TuningMode currentMode = TuningMode.PROPORTIONAL_GAINS;

    // State
    private boolean alignmentActive = false;
    private boolean fineMode = false;
    private ElapsedTime alignmentTimer = new ElapsedTime();
    private ElapsedTime oscillationTimer = new ElapsedTime();
    
    // Performance tracking
    private double lastXOffset = 0;
    private double lastYDistance = 0;
    private double lastYaw = 0;
    private int oscillationCount = 0;
    private double maxOscillation = 0;
    private boolean wasOscillating = false;
    private double alignmentStability = 0; // 0-1, higher is better

    // Button state tracking
    private boolean yPressedLast = false;
    private boolean aPressedLast = false;
    private boolean bPressedLast = false;
    private boolean xPressedLast = false;
    private boolean rbPressedLast = false;
    private boolean lbPressedLast = false;
    private boolean dpadUpLast = false;
    private boolean dpadDownLast = false;
    private boolean dpadLeftLast = false;
    private boolean dpadRightLast = false;

    @Override
    public void runOpMode() {
        drive = new DriveSubsystem(hardwareMap, telemetry);
        aprilTag = new AprilTagNavigator(drive, hardwareMap, telemetry);

        telemetry.addLine("=== APRIL TAG ALIGNMENT TUNER ===");
        telemetry.addLine("Initializing...");
        telemetry.update();

        waitForStart();

        // Initialize camera controls
        aprilTag.initializeCameraControls();

        while (opModeIsActive()) {
            // Update AprilTag detection
            aprilTag.updateRobotPositionFromAllianceGoals();

            // Handle controls
            handleControls();

            // Run alignment if active
            if (alignmentActive) {
                runAlignment();
                trackOscillation();
            } else {
                drive.stop();
            }

            // Update telemetry
            updateTelemetry();
        }

        drive.stop();
        aprilTag.closeVision();
    }

    private void handleControls() {
        // Toggle alignment
        boolean y = gamepad1.y;
        if (y && !yPressedLast) {
            alignmentActive = !alignmentActive;
            if (alignmentActive) {
                alignmentTimer.reset();
                oscillationTimer.reset();
                oscillationCount = 0;
                maxOscillation = 0;
                wasOscillating = false;
            }
        }
        yPressedLast = y;

        // Reset to defaults
        boolean a = gamepad1.a;
        if (a && !aPressedLast) {
            resetToDefaults();
        }
        aPressedLast = a;

        // Save as best
        boolean b = gamepad1.b;
        if (b && !bPressedLast) {
            saveAsBest();
        }
        bPressedLast = b;

        // Toggle tuning mode
        boolean x = gamepad1.x;
        if (x && !xPressedLast) {
            cycleTuningMode();
        }
        xPressedLast = x;

        // Fine/Coarse mode
        boolean rb = gamepad1.right_bumper;
        boolean lb = gamepad1.left_bumper;
        fineMode = rb;
        if (rb && !rbPressedLast) {
            // Fine mode activated
        }
        if (lb && !lbPressedLast) {
            // Coarse mode activated
        }
        rbPressedLast = rb;
        lbPressedLast = lb;

        // Adjust values based on current mode
        if (!alignmentActive) {
            adjustValues();
        }
    }

    private void adjustValues() {
        double step = fineMode ? 0.001 : 0.01; // Fine: 0.001, Coarse: 0.01
        double angleStep = fineMode ? 0.1 : 1.0;
        double distanceStep = fineMode ? 0.5 : 2.0;
        double powerStep = fineMode ? 0.01 : 0.05;

        boolean dpadUp = gamepad1.dpad_up;
        boolean dpadDown = gamepad1.dpad_down;
        boolean dpadLeft = gamepad1.dpad_left;
        boolean dpadRight = gamepad1.dpad_right;

        switch (currentMode) {
            case PROPORTIONAL_GAINS:
                if (dpadUp && !dpadUpLast) {
                    currentKPStrafe = Range.clip(currentKPStrafe + step, 0.0, 0.2);
                }
                if (dpadDown && !dpadDownLast) {
                    currentKPStrafe = Range.clip(currentKPStrafe - step, 0.0, 0.2);
                }
                if (dpadRight && !dpadRightLast) {
                    currentKPForward = Range.clip(currentKPForward + step, 0.0, 0.2);
                }
                if (dpadLeft && !dpadLeftLast) {
                    currentKPForward = Range.clip(currentKPForward - step, 0.0, 0.2);
                }
                
                // Left stick Y for KP_ROT
                double leftStickY = -gamepad1.left_stick_y;
                if (Math.abs(leftStickY) > 0.1) {
                    currentKPRot = Range.clip(currentKPRot + leftStickY * step * 0.1, 0.0, 0.1);
                }
                
                // Right stick Y for MAX_POWER
                double rightStickY = -gamepad1.right_stick_y;
                if (Math.abs(rightStickY) > 0.1) {
                    currentMaxPower = Range.clip(currentMaxPower + rightStickY * powerStep * 0.1, 0.1, 1.0);
                }
                break;

            case DEADBANDS:
                if (dpadUp && !dpadUpLast) {
                    currentPositionDeadband = Range.clip(currentPositionDeadband + step, 0.0, 5.0);
                }
                if (dpadDown && !dpadDownLast) {
                    currentPositionDeadband = Range.clip(currentPositionDeadband - step, 0.0, 5.0);
                }
                if (dpadRight && !dpadRightLast) {
                    currentAngleDeadband = Range.clip(currentAngleDeadband + angleStep, 0.0, 10.0);
                }
                if (dpadLeft && !dpadLeftLast) {
                    currentAngleDeadband = Range.clip(currentAngleDeadband - angleStep, 0.0, 10.0);
                }
                break;

            case TARGET_VALUES:
                if (dpadUp && !dpadUpLast) {
                    currentDesiredDistance = Range.clip(currentDesiredDistance + distanceStep, 50.0, 200.0);
                }
                if (dpadDown && !dpadDownLast) {
                    currentDesiredDistance = Range.clip(currentDesiredDistance - distanceStep, 50.0, 200.0);
                }
                if (dpadRight && !dpadRightLast) {
                    currentDesiredAngle = Range.clip(currentDesiredAngle + angleStep, -45.0, 45.0);
                }
                if (dpadLeft && !dpadLeftLast) {
                    currentDesiredAngle = Range.clip(currentDesiredAngle - angleStep, -45.0, 45.0);
                }
                break;
        }

        dpadUpLast = dpadUp;
        dpadDownLast = dpadDown;
        dpadLeftLast = dpadLeft;
        dpadRightLast = dpadRight;
    }

    private void cycleTuningMode() {
        switch (currentMode) {
            case PROPORTIONAL_GAINS:
                currentMode = TuningMode.DEADBANDS;
                break;
            case DEADBANDS:
                currentMode = TuningMode.TARGET_VALUES;
                break;
            case TARGET_VALUES:
                currentMode = TuningMode.PROPORTIONAL_GAINS;
                break;
        }
    }

    private void resetToDefaults() {
        currentKPStrafe = DEFAULT_KP_STRAFE;
        currentKPForward = DEFAULT_KP_FORWARD;
        currentKPRot = DEFAULT_KP_ROT;
        currentMaxPower = DEFAULT_MAX_POWER;
        currentPositionDeadband = DEFAULT_POSITION_DEADBAND;
        currentAngleDeadband = DEFAULT_ANGLE_DEADBAND;
        currentDesiredDistance = DEFAULT_DESIRED_DISTANCE;
        currentDesiredAngle = DEFAULT_DESIRED_ANGLE;
    }

    private void saveAsBest() {
        bestKPStrafe = currentKPStrafe;
        bestKPForward = currentKPForward;
        bestKPRot = currentKPRot;
        bestMaxPower = currentMaxPower;
        bestPositionDeadband = currentPositionDeadband;
        bestAngleDeadband = currentAngleDeadband;
        bestDesiredDistance = currentDesiredDistance;
        bestDesiredAngle = currentDesiredAngle;
    }

    private void runAlignment() {
        AprilTagDetection tag = aprilTag.getBestAllianceGoalDetection();

        if (tag == null || tag.id != targetTagId) {
            drive.drive(0, 0, 0);
            return;
        }

        double[] corrections = aprilTag.calculateAlignmentCorrections(
                tag,
                currentDesiredDistance,
                currentDesiredAngle,
                currentPositionDeadband,
                currentPositionDeadband,
                currentAngleDeadband,
                currentKPStrafe,
                currentKPForward,
                currentKPRot,
                currentMaxPower
        );

        if (corrections != null) {
            // Store current values for oscillation tracking
            lastXOffset = tag.ftcPose.x;
            lastYDistance = tag.ftcPose.y;
            lastYaw = tag.ftcPose.yaw;
            
            drive.drive(corrections[1], corrections[0], corrections[2]);
        } else {
            drive.drive(0, 0, 0);
        }
    }

    private void trackOscillation() {
        AprilTagDetection tag = aprilTag.getBestAllianceGoalDetection();
        if (tag == null || tag.id != targetTagId) {
            return;
        }

        double currentXOffset = tag.ftcPose.x;
        double currentYDistance = tag.ftcPose.y;
        double currentYaw = tag.ftcPose.yaw;

        // Detect oscillations (rapid sign changes in error)
        double xChange = Math.abs(currentXOffset - lastXOffset);
        double yChange = Math.abs(currentYDistance - lastYDistance);
        double yawChange = Math.abs(currentYaw - lastYaw);

        // If changes are large and frequent, we're oscillating
        if (xChange > 0.5 || yChange > 0.5 || yawChange > 1.0) {
            if (!wasOscillating) {
                oscillationCount++;
                wasOscillating = true;
            }
            double totalChange = xChange + yChange + yawChange;
            if (totalChange > maxOscillation) {
                maxOscillation = totalChange;
            }
        } else {
            wasOscillating = false;
        }

        // Calculate stability (inverse of oscillation)
        double timeSinceStart = alignmentTimer.milliseconds();
        if (timeSinceStart > 0) {
            alignmentStability = Math.max(0, 1.0 - (oscillationCount * 0.1) - (maxOscillation * 0.01));
            alignmentStability = Range.clip(alignmentStability, 0.0, 1.0);
        }
    }

    private void updateTelemetry() {
        telemetry.addLine("=== APRIL TAG ALIGNMENT TUNER ===");
        telemetry.addData("Alignment", alignmentActive ? "ACTIVE (Hold Y)" : "INACTIVE");
        telemetry.addData("Tuning Mode", currentMode);
        telemetry.addData("Adjust Mode", fineMode ? "FINE" : "COARSE");
        telemetry.addLine();

        // Current AprilTag detection info (in inches)
        AprilTagDetection tag = aprilTag.getBestAllianceGoalDetection();
        if (tag != null && tag.id == targetTagId) {
            telemetry.addLine("=== APRIL TAG DETECTION (INCHES) ===");
            telemetry.addData("Tag ID", "%d", tag.id);
            telemetry.addData("Range", "%.2f\"", tag.ftcPose.range);
            telemetry.addData("X Offset", "%.2f\"", tag.ftcPose.x);
            telemetry.addData("Y Distance", "%.2f\"", tag.ftcPose.y);
            telemetry.addData("Z Height", "%.2f\"", tag.ftcPose.z);
            telemetry.addData("Yaw", "%.2f°", tag.ftcPose.yaw);
            telemetry.addData("Pitch", "%.2f°", tag.ftcPose.pitch);
            telemetry.addData("Roll", "%.2f°", tag.ftcPose.roll);
            telemetry.addData("Bearing", "%.2f°", tag.ftcPose.bearing);
            telemetry.addData("Elevation", "%.2f°", tag.ftcPose.elevation);
            telemetry.addData("Decision Margin", "%.2f", tag.decisionMargin);
            telemetry.addLine();
        } else {
            telemetry.addLine("=== NO TAG DETECTED ===");
            telemetry.addData("Looking for Tag ID", "%d", targetTagId);
            telemetry.addLine();
        }

        // Current tuning values
        telemetry.addLine("=== CURRENT VALUES ===");
        telemetry.addData("KP_STRAFE", "%.4f", currentKPStrafe);
        telemetry.addData("KP_FORWARD", "%.4f", currentKPForward);
        telemetry.addData("KP_ROT", "%.4f", currentKPRot);
        telemetry.addData("MAX_POWER", "%.2f", currentMaxPower);
        telemetry.addData("POS_DEADBAND", "%.2f\"", currentPositionDeadband);
        telemetry.addData("ANGLE_DEADBAND", "%.2f°", currentAngleDeadband);
        telemetry.addData("DESIRED_DISTANCE", "%.1f\"", currentDesiredDistance);
        telemetry.addData("DESIRED_ANGLE", "%.1f°", currentDesiredAngle);
        telemetry.addLine();

        // Best values
        telemetry.addLine("=== BEST VALUES (SAVED) ===");
        telemetry.addData("KP_STRAFE", "%.4f", bestKPStrafe);
        telemetry.addData("KP_FORWARD", "%.4f", bestKPForward);
        telemetry.addData("KP_ROT", "%.4f", bestKPRot);
        telemetry.addData("MAX_POWER", "%.2f", bestMaxPower);
        telemetry.addData("POS_DEADBAND", "%.2f\"", bestPositionDeadband);
        telemetry.addData("ANGLE_DEADBAND", "%.2f°", bestAngleDeadband);
        telemetry.addData("DESIRED_DISTANCE", "%.1f\"", bestDesiredDistance);
        telemetry.addData("DESIRED_ANGLE", "%.1f°", bestDesiredAngle);
        telemetry.addLine();

        // Performance metrics
        if (alignmentActive) {
            telemetry.addLine("=== PERFORMANCE METRICS ===");
            telemetry.addData("Oscillation Count", "%d", oscillationCount);
            telemetry.addData("Max Oscillation", "%.2f", maxOscillation);
            telemetry.addData("Stability", "%.2f%%", alignmentStability * 100);
            telemetry.addData("Time Active", "%.1fs", alignmentTimer.seconds());
            telemetry.addLine();
        }

        // Alignment errors (if tag detected)
        if (tag != null && tag.id == targetTagId) {
            double forwardError = tag.ftcPose.y - currentDesiredDistance;
            double angleError = tag.ftcPose.yaw + currentDesiredAngle;
            telemetry.addLine("=== ALIGNMENT ERRORS ===");
            telemetry.addData("X Error", "%.2f\"", tag.ftcPose.x);
            telemetry.addData("Y Error", "%.2f\"", forwardError);
            telemetry.addData("Angle Error", "%.2f°", angleError);
            
            // Calculate correction powers
            double[] corrections = aprilTag.calculateAlignmentCorrections(
                    tag,
                    currentDesiredDistance,
                    currentDesiredAngle,
                    currentPositionDeadband,
                    currentPositionDeadband,
                    currentAngleDeadband,
                    currentKPStrafe,
                    currentKPForward,
                    currentKPRot,
                    currentMaxPower
            );
            if (corrections != null) {
                telemetry.addData("Strafe Power", "%.3f", corrections[0]);
                telemetry.addData("Forward Power", "%.3f", corrections[1]);
                telemetry.addData("Turn Power", "%.3f", corrections[2]);
                telemetry.addData("Aligned", corrections[3] == 1.0 ? "YES" : "NO");
            }
            telemetry.addLine();
        }

        // Controls help
        telemetry.addLine("=== CONTROLS ===");
        telemetry.addData("Y", "Toggle Alignment");
        telemetry.addData("A", "Reset to Defaults");
        telemetry.addData("B", "Save as Best");
        telemetry.addData("X", "Cycle Tuning Mode");
        telemetry.addData("RB", "Fine Mode");
        telemetry.addData("LB", "Coarse Mode");
        telemetry.addData("D-Pad", "Adjust Values (mode dependent)");
        telemetry.addData("Sticks", "Adjust KP_ROT (L) / MAX_POWER (R)");

        telemetry.update();
    }
}

