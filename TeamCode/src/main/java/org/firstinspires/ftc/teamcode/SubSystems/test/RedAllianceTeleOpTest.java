package org.firstinspires.ftc.teamcode.SubSystems.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import org.firstinspires.ftc.teamcode.SubSystems.Drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Shooter.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Spindexer.OldSpindexerSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Funnel.FunnelSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Vision.AprilTagNavigator;

/**
 * FINAL COMPETITION TELEOP - AUDITED VERSION
 * Logic: Circular 1 -> 2 -> 3 -> Intake flow (Clockwise Only).
 */
@TeleOp(name = "Red Alliance TeleOp - FINAL", group = "TeleOp")
public class RedAllianceTeleOpTest extends LinearOpMode {

    private enum RobotMode { INTAKE, SHOOTING_SETUP, SHOOTING_READY, MANUAL_OVERRIDE }

    // Subsystems
    private DriveSubsystem drive;
    private ShooterSubsystem shooter;
    private OldSpindexerSubsystem spindexer;
    private FunnelSubsystem funnel;
    private AprilTagNavigator aprilTag;

    // State Variables
    private RobotMode currentMode = RobotMode.INTAKE;
    private int spindexerIndex = 0; 
    private boolean intakeMode = true; 
    private double driveSpeed = 1.0;

    // Input Latches
    private boolean lastInputA = false;
    private boolean lastInputB = false;
    private boolean lastInputX = false;
    private boolean lastInputDpadDown = false;

    // === DRIVE INPUT CONSTANTS ===
    private static final double STICK_DEADBAND = 0.05;     // Prevent drift from stick noise
    private static final double CUBIC_INPUT_POWER = 3.0;   // Smooth low-speed control

    // === SPINDEXER STATE TRACKING ===
    private long spindexerReadyTime = 0;                    // Debounce timer for mode transitions
    private static final long SPINDEXER_DEBOUNCE_MS = 100;  // 100ms debounce
    private long visionLostTime = 0;                        // Track how long tag has been lost
    private static final long VISION_LOSS_TIMEOUT_MS = 500; // Timeout after 0.5s

    // === FUNNEL STATE MACHINE ===
    private enum FunnelState { RETRACTED, EXTENDING, EXTENDED, RETRACTING }
    private FunnelState funnelState = FunnelState.RETRACTED;
    private long funnelTimer = 0;
    private static final long FUNNEL_EXTEND_TIME_MS = 400;  // Time to fully extend/retract
    private static final long FUNNEL_HOLD_TIME_MS = 300;    // Hold time before retracting

    // === ALIGNMENT CONSTANTS ===
    private static final double DESIRED_SHOOTING_DISTANCE = 134;
    private static final double DESIRED_SHOOTING_ANGLE = 21;
    private static final double KP_STRAFE = 0.03, KP_FORWARD = 0.03, KP_ROT = 0.015;
    private static final double MAX_AUTO_POWER = 0.40;
    private static final double POS_DEADBAND = 0.75, ANGLE_DEADBAND = 1.5;
    private static final int TARGET_TAG_ID = 24;
    
    // === SHOOTER CONFIG ===
    private static final double SHOOTER_RPM = 5225;
    
    // === VISION SMOOTHING (Exponential Moving Average) ===
    private static final double VISION_ALPHA = 0.3;  // Weight for new measurement (0.3 = 30% new, 70% old)
    private double lastStrafeCorrection = 0.0;
    private double lastForwardCorrection = 0.0; 

    @Override
    public void runOpMode() {
        // Initialize Subsystems
        drive = new DriveSubsystem(hardwareMap, telemetry);
        shooter = new ShooterSubsystem(hardwareMap, telemetry);
        spindexer = new OldSpindexerSubsystem(hardwareMap, telemetry);
        funnel = new FunnelSubsystem(hardwareMap, telemetry);
        aprilTag = new AprilTagNavigator(drive, hardwareMap, telemetry);

        // ✅ FIX: Initialize camera controls ONCE before the loop to prevent lag
        aprilTag.initializeCameraControls();

        waitForStart();

        while (opModeIsActive()) {
            shooter.updateVoltageCompensation();

            // 1. MANUAL OVERRIDE TOGGLE
            if (gamepad2.x && !lastInputX) {
                if (currentMode == RobotMode.MANUAL_OVERRIDE) currentMode = RobotMode.INTAKE;
                else {
                    currentMode = RobotMode.MANUAL_OVERRIDE;
                    spindexer.lockCurrentPosition();
                }
            }
            lastInputX = gamepad2.x;

            // 2. STATE MACHINE TRANSITIONS
            if (currentMode != RobotMode.MANUAL_OVERRIDE) {
                handleCircularTransitions();
            }

            // 3. DRIVE & AUTO-ALIGNMENT
            handleDriveAndAlignment();

            // 4. HARDWARE COMMANDS
            executeHardwareActions();

            // ✅ ALWAYS RUN PID: Keeps "Active Hold" working to resist camera-cam force
            spindexer.update();

            updateTelemetry();
        }
    }

    private void handleCircularTransitions() {
        // ✅ COLLISION PREVENTION: Only allow spindexer movement when funnel is fully retracted
        if (funnelState != FunnelState.RETRACTED) {
            return; // Block all spindexer transitions while funnel is moving
        }

        // Press B: Start Shooting Sequence (Ball 1)
        if (gamepad2.b && !lastInputB && currentMode == RobotMode.INTAKE) {
            intakeMode = false;
            spindexerIndex = 0;
            spindexer.setIntakeMode(false);
            spindexer.goToPositionForCurrentMode(0);
            currentMode = RobotMode.SHOOTING_SETUP;
        }
        lastInputB = gamepad2.b;

        // Press A: Move to next Ball (1 -> 2 -> 3 -> Intake)
        if (gamepad2.a && !lastInputA) {
            spindexerIndex++;
            if (spindexerIndex > 2) {
                // Return to Intake configuration
                intakeMode = true;
                spindexerIndex = 0;
                spindexer.setIntakeMode(true);
                spindexer.goToPositionForCurrentMode(0);
                currentMode = RobotMode.INTAKE;
            } else {
                spindexer.goToPositionForCurrentMode(spindexerIndex);
                currentMode = RobotMode.SHOOTING_SETUP;
            }
        }
        lastInputA = gamepad2.a;

        // Reset to Intake via D-Pad
        if (gamepad2.dpad_down && !lastInputDpadDown) {
            intakeMode = true;
            spindexerIndex = 0;
            spindexer.setIntakeMode(true);
            spindexer.goToPositionForCurrentMode(0);
            currentMode = RobotMode.INTAKE;
        }
        lastInputDpadDown = gamepad2.dpad_down;

        // Auto-Ready Transition (with debounce to prevent flickering)
        if (currentMode == RobotMode.SHOOTING_SETUP && spindexer.isAtPosition()) {
            if (spindexerReadyTime == 0) {
                spindexerReadyTime = System.currentTimeMillis();
            } else if (System.currentTimeMillis() - spindexerReadyTime >= SPINDEXER_DEBOUNCE_MS) {
                currentMode = RobotMode.SHOOTING_READY;
                spindexerReadyTime = 0; // Reset debounce
            }
        } else {
            spindexerReadyTime = 0; // Reset if position check fails
        }
    }

    private void handleDriveAndAlignment() {
        // --- SENSOR FUSION ALIGNMENT: Vision + IMU ---
        // Hold Y to activate auto-alignment using AprilTag (vision truth) + IMU (gyro stability)
        if (gamepad2.y) {
            // 1. ASK VISION: Find the tag and update robot's position/heading in DriveSubsystem
            boolean tagSeen = aprilTag.updateRobotPositionFromAllianceGoals();
            AprilTagDetection tag = aprilTag.getBestAllianceGoalDetection();

            if (tag != null && tag.id == TARGET_TAG_ID) {
                visionLostTime = 0; // Reset timeout
                
                // 2. GET X/Y CORRECTIONS FROM VISION (strafe & forward)
                // dbAngle = 0, kPR = 0: We skip vision-based rotation and use IMU instead
                double[] corrections = aprilTag.calculateAlignmentCorrections(
                        tag, DESIRED_SHOOTING_DISTANCE, DESIRED_SHOOTING_ANGLE,
                        POS_DEADBAND, POS_DEADBAND, 0,    // dbAngle set to 0 (handled by IMU)
                        KP_STRAFE, KP_FORWARD, 0, MAX_AUTO_POWER); // kPR set to 0 (IMU controls turn)
                
                if (corrections != null) {
                    // 3. CALCULATE TURN POWER USING IMU (gyro-based PID)
                    double currentHeadingDeg = Math.toDegrees(drive.getHeading());
                    double angleError = AngleUnit.normalizeDegrees(DESIRED_SHOOTING_ANGLE - currentHeadingDeg);
                    double turnPower = Range.clip(angleError * KP_ROT, -MAX_AUTO_POWER, MAX_AUTO_POWER);

                    // 4. APPLY EXPONENTIAL MOVING AVERAGE TO VISION CORRECTIONS (reduce jitter)
                    double smoothedStrafe = VISION_ALPHA * corrections[1] + (1.0 - VISION_ALPHA) * lastStrafeCorrection;
                    double smoothedForward = VISION_ALPHA * corrections[0] + (1.0 - VISION_ALPHA) * lastForwardCorrection;
                    lastStrafeCorrection = smoothedStrafe;
                    lastForwardCorrection = smoothedForward;

                    // 5. COMBINE VISION (STRAFE + FORWARD) + IMU (TURN)
                    drive.drive(smoothedStrafe, smoothedForward, turnPower);
                    return;
                }
            } else {
                // TAG LOST: Track timeout and hold angle
                if (visionLostTime == 0) {
                    visionLostTime = System.currentTimeMillis();
                }
                
                // Still searching within timeout window? Use IMU to hold angle
                if (System.currentTimeMillis() - visionLostTime < VISION_LOSS_TIMEOUT_MS) {
                    double currentHeadingDeg = Math.toDegrees(drive.getHeading());
                    double angleError = AngleUnit.normalizeDegrees(DESIRED_SHOOTING_ANGLE - currentHeadingDeg);
                    double turnPower = Range.clip(angleError * KP_ROT, -MAX_AUTO_POWER, MAX_AUTO_POWER);
                    
                    drive.drive(0, 0, turnPower);
                    return;
                } else {
                    // Timeout exceeded: disable alignment to allow manual control
                    // Don't drive, let driver take over
                    return;
                }
            }
        }

        // --- MANUAL DRIVER CONTROLS ---
        // Standard teleop driving (not vision-assisted)
        if (gamepad1.a) driveSpeed = 1.0;
        if (gamepad1.b) driveSpeed = 0.5;

        // Apply deadband to prevent stick drift
        float forward = applyDeadband(gamepad1.left_stick_y + gamepad1.right_stick_y, STICK_DEADBAND);
        float strafe = applyDeadband(-gamepad1.left_stick_x, STICK_DEADBAND);
        float turn = applyDeadband(-gamepad1.right_stick_x, STICK_DEADBAND);

        // Apply cubic scaling for smoother low-speed control
        forward = (float) Math.copySign(Math.pow(Math.abs(forward), CUBIC_INPUT_POWER), forward);
        strafe = (float) Math.copySign(Math.pow(Math.abs(strafe), CUBIC_INPUT_POWER), strafe);
        turn = (float) Math.copySign(Math.pow(Math.abs(turn), CUBIC_INPUT_POWER), turn);

        // Normalize powers to prevent exceeding 1.0 (improves on 0.5 + |turn| heuristic)
        double max = Math.max(1.0, Math.abs(forward) + Math.abs(strafe) + Math.abs(turn));
        forward /= max;
        strafe /= max;
        turn /= max;

        drive.drive(forward * driveSpeed, strafe * driveSpeed, turn * driveSpeed);
        
        if (gamepad1.start) drive.resetHeading();
    }

    /**
     * Apply deadband to joystick input to prevent drift from stick noise.
     * If input is below deadband threshold, return 0. Otherwise, scale linearly.
     */
    private float applyDeadband(double value, double deadband) {
        if (Math.abs(value) < deadband) {
            return 0.0f;
        }
        // Scale from [deadband, 1.0] -> [0, 1.0]
        double scaled = (Math.abs(value) - deadband) / (1.0 - deadband);
        return (float) Math.copySign(scaled, value);
    }

    private void executeHardwareActions() {
        // Shooter logic: Spin up during setup/ready or manual override
        if (currentMode == RobotMode.SHOOTING_READY || currentMode == RobotMode.SHOOTING_SETUP || gamepad2.right_bumper) {
            shooter.setTargetRPM(SHOOTER_RPM);
        } else {
            shooter.stop();
        }

        // === FUNNEL STATE MACHINE ===
        // Ensures funnel fully extends/retracts before spindexer can move (collision prevention)
        switch (funnelState) {
            case RETRACTED:
                // Trigger pressed and we're in shooting mode? Start extending
                if (gamepad2.right_trigger > 0.5 && (currentMode == RobotMode.SHOOTING_READY || currentMode == RobotMode.MANUAL_OVERRIDE)) {
                    funnel.extend();
                    funnelState = FunnelState.EXTENDING;
                    funnelTimer = System.currentTimeMillis();
                }
                break;

            case EXTENDING:
                // Wait for funnel to fully extend
                if (System.currentTimeMillis() - funnelTimer >= FUNNEL_EXTEND_TIME_MS) {
                    funnelState = FunnelState.EXTENDED;
                    funnelTimer = System.currentTimeMillis();
                }
                break;

            case EXTENDED:
                // Hold extended state for a moment before retracting
                if (System.currentTimeMillis() - funnelTimer >= FUNNEL_HOLD_TIME_MS) {
                    funnelState = FunnelState.RETRACTING;
                    funnel.retract();
                    funnelTimer = System.currentTimeMillis();
                }
                break;

            case RETRACTING:
                // Wait for funnel to fully retract
                if (System.currentTimeMillis() - funnelTimer >= FUNNEL_EXTEND_TIME_MS) {
                    funnelState = FunnelState.RETRACTED;
                }
                break;
        }

        // Manual Spindexer (Gamepad 2 Right Stick)
        if (currentMode == RobotMode.MANUAL_OVERRIDE) {
            float stick = -gamepad2.right_stick_y;
            if (Math.abs(stick) > STICK_DEADBAND) {
                spindexer.setManualPower(stick * 0.75);
            } else {
                // ✅ FIX: Call stopManual() to truly disengage PID and motor
                spindexer.stopManual();
            }
        }
    }

    private void updateTelemetry() {
        telemetry.addData("PHASE", currentMode);
        telemetry.addData("TARGET SLOT", spindexerIndex + 1);
        telemetry.addData("INTAKE MODE", intakeMode);
        telemetry.addData("FUNNEL STATE", funnelState);
        
        AprilTagDetection bestTag = aprilTag.getBestAllianceGoalDetection();
        telemetry.addData("TAG ID", bestTag != null ? bestTag.id : "NONE");
        telemetry.addData("TAG RANGE", bestTag != null ? String.format("%.2f in", bestTag.ftcPose.range) : "N/A");
        
        telemetry.addData("SHOOTER RPM", "%.0f / %.0f", shooter.getCurrentRPM(), shooter.getTargetRPM());
        telemetry.addData("ALIGNMENT", gamepad2.y ? "ACTIVE" : "OFF");
        telemetry.update();
    }
}