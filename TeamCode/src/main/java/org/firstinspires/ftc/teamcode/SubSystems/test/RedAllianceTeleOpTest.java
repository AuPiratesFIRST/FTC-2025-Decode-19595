package org.firstinspires.ftc.teamcode.SubSystems.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import org.firstinspires.ftc.teamcode.SubSystems.Drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Shooter.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Spindexer.OldSpindexerSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Funnel.FunnelSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Vision.AprilTagNavigator;
import org.firstinspires.ftc.teamcode.SubSystems.Control.AimController;

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
    private IntakeSubsystem intake;
    private OldSpindexerSubsystem spindexer;
    private FunnelSubsystem funnel;
    private AprilTagNavigator aprilTag;
    private AimController aimController;

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
    private boolean lastInputDpadLeft = false;

    // === DRIVE INPUT CONSTANTS ===
    private static final double STICK_DEADBAND = 0.05;     // Prevent drift from stick noise
    private static final double CUBIC_INPUT_POWER = 3.0;   // Smooth low-speed control

    // === SPINDEXER STATE TRACKING ===
    private long spindexerReadyTime = 0;                    // Debounce timer for mode transitions
    private static final long SPINDEXER_DEBOUNCE_MS = 100;  // 100ms debounce

    // === FUNNEL STATE MACHINE ===
    private enum FunnelState { RETRACTED, EXTENDING, EXTENDED, RETRACTING }
    private FunnelState funnelState = FunnelState.RETRACTED;
    private long funnelTimer = 0;
    private static final long FUNNEL_EXTEND_TIME_MS = 400;  // Time to fully extend/retract
    private static final long FUNNEL_HOLD_TIME_MS = 300;    // Hold time before retracting
    
    // === SHOOTER CONFIG ===
    // Uses centralized value from ShooterSubsystem.TARGET_RPM
    
    // === INTAKE POWER CONFIG (Keeper Wheel Pattern) ===
    private static final double INTAKE_HOLD_POWER = 0.5;      // Always-on background power
    private static final double INTAKE_FULL_POWER = 1.0;       // Full intake (trigger pressed)
    private static final double INTAKE_REVERSE_POWER = -1.0;   // Reverse (bumper pressed)
    
    // === MANUAL SPINDEXER TUNING ===
    private static final double SPINDEXER_MANUAL_SCALE = 0.6;   // Overall power limit
    private static final double SPINDEXER_CUBIC_POWER = 3.0;    // Higher = less sensitive near center

    @Override
    public void runOpMode() {
        // Initialize Subsystems
        drive = new DriveSubsystem(hardwareMap, telemetry);
        shooter = new ShooterSubsystem(hardwareMap, telemetry);
        intake = new IntakeSubsystem(hardwareMap, telemetry);
        spindexer = new OldSpindexerSubsystem(hardwareMap, telemetry);
        funnel = new FunnelSubsystem(hardwareMap, telemetry);
        aprilTag = new AprilTagNavigator(drive, hardwareMap, telemetry);

        // ✅ FIX: Initialize camera controls ONCE before the loop to prevent lag
        aprilTag.initializeCameraControls();

        // Initialize AimController with competition settings
        aimController = new AimController(aprilTag, drive, telemetry);
        aimController.setDesiredDistance(134);
        aimController.setDesiredAngle(21);
        aimController.setGains(0.03, 0.03, 0.015);
        aimController.setMaxPower(0.40);
        aimController.setDeadbands(0.75, 1.5, 2.0);
        aimController.setTargetTagId(24);
        aimController.setVisionLossTimeout(500);
        aimController.setVisionSmoothing(0.3);

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

            // 1.5 INTAKE/OUTTAKE MODE TOGGLE (D-Pad Left)
            if (gamepad2.dpad_left && !lastInputDpadLeft) {
                boolean wasIntakeMode = intakeMode;
                intakeMode = !intakeMode;
                spindexer.setIntakeMode(intakeMode);
                // Reset to position 0 in the new mode
                spindexerIndex = 0;
                
                // If switching from intake to outtake, use backward movement
                if (wasIntakeMode && !intakeMode) {
                    int targetTicks = OldSpindexerSubsystem.OUTTAKE_POSITIONS[0];
                    spindexer.goToPositionBackwardOnly(targetTicks);
                } else {
                    // Otherwise use normal shortest path
                    spindexer.goToPositionForCurrentMode(0);
                }
                
                if (intakeMode) {
                    currentMode = RobotMode.INTAKE;
                }
            }
            lastInputDpadLeft = gamepad2.dpad_left;

            // 2. STATE MACHINE TRANSITIONS
            if (currentMode != RobotMode.MANUAL_OVERRIDE) {
                handleCircularTransitions();
            }

            // 3. DRIVE & AUTO-ALIGNMENT
            handleDriveAndAlignment();

            // 3.5 INTAKE CONTROL (LT forward, LB reverse)
            handleIntake();

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
            // ✅ FORWARD-ONLY: Guarantee clockwise movement from intake to outtake
            spindexer.goToPositionForwardOnly(OldSpindexerSubsystem.OUTTAKE_POSITIONS[0]);
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
        // --- UNIFIED ALIGNMENT: Vision + IMU Fusion with Fallback ---
        // Hold Y to activate auto-alignment using AimController
        if (gamepad2.y) {
            AimController.AlignmentResult result = aimController.update();
            drive.drive(result.strafe, result.forward, result.turn);
            return;
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
        // Shooter logic: Spin up during setup/ready or manual override (but NOT in intake mode)
        if ((currentMode == RobotMode.SHOOTING_READY || currentMode == RobotMode.SHOOTING_SETUP || gamepad2.right_bumper) 
                && !intakeMode) {
            shooter.setTargetRPM(ShooterSubsystem.TARGET_RPM);
        } else {
            shooter.stop();
        }

        // === FUNNEL STATE MACHINE ===
        // Ensures funnel fully extends/retracts before spindexer can move (collision prevention)
        switch (funnelState) {
            case RETRACTED:
                // Trigger pressed and we're in shooting mode? Start extending
                // ✅ CRITICAL: Only extend if spindexer is at position (prevents collision)
                if (gamepad2.right_trigger > 0.5 
                        && (currentMode == RobotMode.SHOOTING_READY || currentMode == RobotMode.MANUAL_OVERRIDE)
                        && spindexer.isAtPosition()) {
                    funnel.extend();
                    funnelState = FunnelState.EXTENDING;
                    funnelTimer = System.currentTimeMillis();
                }
                break;

            case EXTENDING:
                // Wait forfun ne lto fully extend
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

        // Manual Spindexer (Gamepad 2 Right Stick) - Cubic Scaling for Smooth Control
        if (currentMode == RobotMode.MANUAL_OVERRIDE) {
            double raw = -gamepad2.right_stick_y;

            // Deadband
            if (Math.abs(raw) < STICK_DEADBAND) {
                spindexer.stopManual();
            } else {
                // Remove deadband and normalize to [0, 1]
                double normalized =
                        (Math.abs(raw) - STICK_DEADBAND) / (1.0 - STICK_DEADBAND);
                normalized = Math.copySign(normalized, raw);

                // Cubic scaling (smooth near center, full power at extremes)
                double scaled =
                        Math.copySign(Math.pow(Math.abs(normalized), SPINDEXER_CUBIC_POWER),
                                      normalized);

                // Final power with safety scale
                spindexer.setManualPower(scaled * SPINDEXER_MANUAL_SCALE);
            }
        }
    }

    /**
     * KEEPER WHEEL PATTERN: Intake always runs at low hold power, overridable by driver.
     * Priority: Reverse > Full Intake > Hold Power
     */
    private void handleIntake() {
        // Safety: block intake while funnel is not retracted
        if (funnelState != FunnelState.RETRACTED) {
            intake.setPower(0);
            return;
        }

        // Priority 1: Reverse (clears jams, overrides everything)
        if (gamepad2.left_bumper) {
            intake.setPower(INTAKE_REVERSE_POWER);
            return;
        }

        // Priority 2: Full intake when trigger pressed AND in intake mode
        if (gamepad2.left_trigger > 0.1 && intakeMode) {
            intake.setPower(INTAKE_FULL_POWER);
            return;
        }

        // Priority 3: Default background holding power (keeps balls seated)
        intake.setPower(INTAKE_HOLD_POWER);
    }

    private void updateTelemetry() {
        telemetry.addLine("=== GAMEPAD 2 CONTROLS ===");
        telemetry.addLine("B = Start Shooting | A = Next Ball");
        telemetry.addLine("Y = Auto-Align | X = Manual Override");
        telemetry.addLine("D↓ = Reset | D→ = Toggle Mode | RB = Spin");
        telemetry.addLine("RT = Shoot | LT = Intake | LB = Reverse");
        telemetry.addLine();
        
        telemetry.addData("PHASE", currentMode);
        telemetry.addData("TARGET SLOT", spindexerIndex + 1);
        telemetry.addData("SPINDEXER MODE", intakeMode ? "INTAKE" : "OUTTAKE");
        telemetry.addData("FUNNEL STATE", funnelState);
        
        AprilTagDetection bestTag = aprilTag.getBestAllianceGoalDetection();
        telemetry.addData("TAG ID", bestTag != null ? bestTag.id : "NONE");
        telemetry.addData("TAG RANGE", bestTag != null ? String.format("%.2f in", bestTag.ftcPose.range) : "N/A");
        
        telemetry.addData("SHOOTER RPM", "%.0f / %.0f", shooter.getCurrentRPM(), shooter.getTargetRPM());
        
        if (gamepad2.y) {
            telemetry.addData("ALIGNMENT", "ACTIVE");
        } else {
            telemetry.addData("ALIGNMENT", "OFF");
        }
        
        telemetry.update();
    }
}