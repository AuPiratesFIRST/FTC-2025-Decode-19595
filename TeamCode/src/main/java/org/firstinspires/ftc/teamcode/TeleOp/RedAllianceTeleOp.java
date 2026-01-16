package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import org.firstinspires.ftc.teamcode.SubSystems.Drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Shooter.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Spindexer.OldSpindexerSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Funnel.FunnelSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Vision.AprilTagNavigator;
import org.firstinspires.ftc.teamcode.SubSystems.Vision.ObeliskMotifDetector;
import org.firstinspires.ftc.teamcode.SubSystems.Control.AimController;
import org.firstinspires.ftc.teamcode.SubSystems.Scoring.ArtifactColor;

/**
 * Red Alliance TeleOp with AprilTag alignment and automated scoring sequence.
 * 
 * Gamepad 1 (Driver):
 * - Left Stick Y: Forward/backward
 * - Left Stick X: Strafe left/right
 * - Right Stick X: Turn
 * - Right Stick Y: Diagonal forward/backward (adds to forward movement)
 * - A/B: Drive speed toggle (1.0 / 0.5)
 * - D-Pad Up: Toggle inverse direction (for when controller is at back of robot)
 * 
 * Gamepad 2 (Operator):
 * - Left Trigger: Intake forward
 * - Left Bumper: Intake reverse
 * - Right Bumper: Toggle shooter on/off (manual override)
 * - Right Trigger: Extend funnel to shoot (when ready)
 * - A Button: Advance spindexer position (automated sequence)
 * - X Button: Toggle manual/automated spindexer control
 * - D-Pad Down: Reset to intake mode
 * - D-Pad Left: Toggle intake/outtake mode
 * - D-Pad Right: Manual spindexer position increment (when in manual mode)
 * - Right Stick Y: Manual spindexer power control (when in manual mode)
 * - Y Button: Align to Red Alliance goal (Tag 24)
 * 
 * Automated Sequence:
 * - INTAKE: Move spindexer → Wait for position → Ball settling → Ready
 * - OUTTAKE: Move spindexer → Wait for position → Spin up shooter → Wait for RPM → Ready
 */
@TeleOp(name = "Red Alliance TeleOp", group = "TeleOp")
public class RedAllianceTeleOp extends LinearOpMode {

    private enum RobotMode { INTAKE, SHOOTING_SETUP, SHOOTING_READY, MANUAL_OVERRIDE }
    private enum FunnelState { RETRACTED, EXTENDING, EXTENDED, RETRACTING }

    // Subsystems
    private DriveSubsystem drive;
    private IntakeSubsystem intake;
    private ShooterSubsystem shooter;
    private OldSpindexerSubsystem spindexer;
    private FunnelSubsystem funnel;
    private AprilTagNavigator aprilTag;
    private AimController aimController;

    // State Variables
    private RobotMode currentMode = RobotMode.INTAKE;
    private FunnelState funnelState = FunnelState.RETRACTED;
    private int spindexerPositionIndex = 0;
    private boolean intakeMode = true; // true = Intake, false = Outtake
    private double driveSpeed = 1.0;
    private boolean inverseDirection = false;

    // Input Latches
    private boolean spindexerPressLast = false;
    private boolean dpadDownLast = false;
    private boolean dpadUpLast = false;
    private boolean rbPressedLast = false;
    private boolean yPressedLast = false;
    private boolean xPressedLast = false; // Manual control toggle
    private boolean dpadLeftLast = false;
    private boolean dpadRightLast = false;
    private boolean aPressedLast = false;
    private boolean bPressedLast = false;

    // === DRIVE INPUT CONSTANTS ===
    private static final double STICK_DEADBAND = 0.05;     // Prevent drift from stick noise
    private static final double CUBIC_INPUT_POWER = 3.0;   // Smooth low-speed control

    // === SPINDEXER STATE TRACKING ===
    private long spindexerReadyTime = 0;                    // Debounce timer for mode transitions
    private static final long SPINDEXER_DEBOUNCE_MS = 100;  // 100ms debounce

    // === FUNNEL STATE MACHINE ===
    private long funnelTimer = 0;
    private static final long FUNNEL_EXTEND_TIME_MS = 400;  // Time to fully extend/retract
    private static final long FUNNEL_HOLD_TIME_MS = 300;    // Hold time before retracting

    // === AUTO-ALIGNMENT CONSTANTS ===
    private static final double DESIRED_SHOOTING_DISTANCE = 134;
    private static final double DESIRED_SHOOTING_Angle = 21;
    private static final double KP_STRAFE = 0.03;
    private static final double KP_FORWARD = 0.03;
    private static final double KP_ROT = 0.015;
    private static final double MAX_AUTO_POWER = 0.40;
    private static final double POSITION_DEADBAND = 0.75;
    private static final double ANGLE_DEADBAND_DEG = 1.5;

    // Shooter RPM target
    private static final double SHOOTER_TARGET_RPM = 5220.0;

    // Alliance configuration
    private static final boolean IS_BLUE_ALLIANCE = false;
    private static final int TARGET_TAG_ID = 24; // Red alliance goal tag

    // === INTAKE POWER CONFIG (Keeper Wheel Pattern) ===
    private static final double INTAKE_HOLD_POWER = 0.5;      // Always-on background power
    private static final double INTAKE_FULL_POWER = 1.0;       // Full intake (trigger pressed)
    private static final double INTAKE_REVERSE_POWER = -1.0;   // Reverse (bumper pressed)

    // === MANUAL SPINDEXER TUNING ===
    private static final double SPINDEXER_MANUAL_SCALE = 0.6;   // Overall power limit
    private static final double SPINDEXER_CUBIC_POWER = 3.0;    // Higher = less sensitive near center

    // === OBELISK MOTIF TRACKING ===
    private ArtifactColor[] lockedMotif = null; // Locked at start, holds motif for entire match
    private int lockedMotifTagId = -1; // Locked obelisk tag ID

    @Override
    public void runOpMode() {
        // Initialize subsystems
        drive = new DriveSubsystem(hardwareMap, telemetry);
        intake = new IntakeSubsystem(hardwareMap, telemetry);
        shooter = new ShooterSubsystem(hardwareMap, telemetry);
        spindexer = new OldSpindexerSubsystem(hardwareMap, telemetry);
        funnel = new FunnelSubsystem(hardwareMap, telemetry);
        aprilTag = new AprilTagNavigator(drive, hardwareMap, telemetry);

        // Initialize camera controls ONCE before the loop to prevent lag
        aprilTag.initializeCameraControls();

        // Initialize AimController with competition settings
        aimController = new AimController(aprilTag, drive, telemetry);
        aimController.setDesiredDistance(DESIRED_SHOOTING_DISTANCE);
        aimController.setDesiredAngle(DESIRED_SHOOTING_Angle);
        aimController.setGains(KP_STRAFE, KP_FORWARD, KP_ROT);
        aimController.setMaxPower(MAX_AUTO_POWER);
        aimController.setDeadbands(POSITION_DEADBAND, ANGLE_DEADBAND_DEG, 2.0);
        aimController.setTargetTagId(TARGET_TAG_ID);
        aimController.setVisionLossTimeout(500);
        aimController.setVisionSmoothing(0.3);

        // Initialize Obelisk Motif Detector
        ObeliskMotifDetector obeliskDetector = new ObeliskMotifDetector(aprilTag, telemetry);

        telemetry.addData("Status", "Initialized - Red Alliance");
        telemetry.addData("Target Goal", "Red Alliance (Tag 24)");
        telemetry.addData("Obelisk", "Detecting...");
        telemetry.update();

        // === STAGE 1: CONTINUOUS DETECTION DURING INIT ===
        // Continuously detect obelisk motif while waiting for start
        // Driver can see the detected motif on dashboard before starting
        while (!isStarted() && !isStopRequested()) {
            obeliskDetector.update(); // Continuously update motif for driver dashboard
            ArtifactColor[] currentMotif = obeliskDetector.getMotif();
            if (currentMotif != null) {
                telemetry.addData("Obelisk", "Detected: " + motifToString(currentMotif) + " (Tag " + obeliskDetector.getMotifTagId() + ")");
            } else {
                telemetry.addData("Obelisk", "Not detected - waiting...");
            }
            telemetry.update();
            sleep(50); // Small delay to avoid CPU overload
        }

        // === STAGE 2: LOCK MOTIF AT START ===
        // Lock the motif that was detected at start - it won't change during the match
        lockedMotif = obeliskDetector.getMotif();
        lockedMotifTagId = obeliskDetector.getMotifTagId();

        telemetry.addData("Status", "Match Started - Red Alliance");
        if (lockedMotif != null) {
            telemetry.addData("Motif Locked", motifToString(lockedMotif) + " (Tag " + lockedMotifTagId + ")");
        } else {
            telemetry.addData("Motif Locked", "NONE - No obelisk detected at start");
        }
        telemetry.update();

        while (opModeIsActive()) {
            shooter.updateVoltageCompensation();

            // 1. MANUAL OVERRIDE TOGGLE
            boolean x = gamepad2.x;
            if (x && !xPressedLast) {
                if (currentMode == RobotMode.MANUAL_OVERRIDE) {
                    currentMode = RobotMode.INTAKE;
                } else {
                    currentMode = RobotMode.MANUAL_OVERRIDE;
                    spindexer.lockCurrentPosition();
                }
            }
            xPressedLast = x;

            // 1.5 INTAKE/OUTTAKE MODE TOGGLE (D-Pad Left)
            boolean dpadLeft = gamepad2.dpad_left;
            if (dpadLeft && !dpadLeftLast && currentMode != RobotMode.MANUAL_OVERRIDE) {
                boolean wasIntakeMode = intakeMode;
                intakeMode = !intakeMode;
                spindexer.setIntakeMode(intakeMode);
                spindexerPositionIndex = 0; // Reset position when mode changes

                // If switching from intake to outtake, use forward-only movement
                if (wasIntakeMode && !intakeMode) {
                    spindexer.goToPositionForwardOnly(OldSpindexerSubsystem.OUTTAKE_POSITIONS[0]);
                    currentMode = RobotMode.SHOOTING_SETUP;
                } else {
                    // Otherwise use normal shortest path
                    spindexer.goToPositionForCurrentMode(0);
                    if (intakeMode) {
                        currentMode = RobotMode.INTAKE;
                    }
                }
            }
            dpadLeftLast = dpadLeft;

            // 2. STATE MACHINE TRANSITIONS
            if (currentMode != RobotMode.MANUAL_OVERRIDE) {
                handleCircularTransitions();
            }

            // 3. DRIVE & AUTO-ALIGNMENT
            handleDriveAndAlignment();

            // 3.5 INTAKE CONTROL (Keeper Wheel Pattern)
            handleIntake();

            // 4. HARDWARE COMMANDS
            executeHardwareActions();

            // ✅ ALWAYS RUN PID: Keeps "Active Hold" working to resist external force
            spindexer.update();

            // 5. TELEMETRY
            updateTelemetry();
        }

        // Cleanup
        drive.stop();
        intake.stop();
        shooter.stop();
        funnel.retract(); // Retract funnels on stop
        aprilTag.closeVision();
    }

    private void handleCircularTransitions() {
        // ✅ COLLISION PREVENTION: Only allow spindexer movement when funnel is fully retracted
        if (funnelState != FunnelState.RETRACTED) {
            return; // Block all spindexer transitions while funnel is moving
        }

        // Press B: Start Shooting Sequence (Ball 1) - Switch to outtake mode
        boolean b = gamepad2.b;
        if (b && !bPressedLast && currentMode == RobotMode.INTAKE) {
            intakeMode = false;
            spindexerPositionIndex = 0;
            spindexer.setIntakeMode(false);
            // ✅ FORWARD-ONLY: Guarantee clockwise movement from intake to outtake
            spindexer.goToPositionForwardOnly(OldSpindexerSubsystem.OUTTAKE_POSITIONS[0]);
            currentMode = RobotMode.SHOOTING_SETUP;
        }
        bPressedLast = b;

        // Press A: Move to next Ball (1 -> 2 -> 3 -> Intake)
        boolean a = gamepad2.a;
        if (a && !aPressedLast) {
            spindexerPositionIndex++;
            if (spindexerPositionIndex > 2) {
                // Return to Intake configuration
                intakeMode = true;
                spindexerPositionIndex = 0;
                spindexer.setIntakeMode(true);
                spindexer.goToPositionForCurrentMode(0);
                currentMode = RobotMode.INTAKE;
            } else {
                spindexer.goToPositionForCurrentMode(spindexerPositionIndex);
                currentMode = RobotMode.SHOOTING_SETUP;
            }
        }
        aPressedLast = a;

        // Reset to Intake via D-Pad Down
        boolean dpadDown = gamepad2.dpad_down;
        if (dpadDown && !dpadDownLast) {
            intakeMode = true;
            spindexerPositionIndex = 0;
            spindexer.setIntakeMode(true);
            spindexer.goToPositionForCurrentMode(0);
            currentMode = RobotMode.INTAKE;
        }
        dpadDownLast = dpadDown;

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
        boolean y = gamepad2.y;
        if (y) {
            AimController.AlignmentResult result = aimController.update();
            drive.drive(result.strafe, result.forward, result.turn);
            return;
        }
        yPressedLast = y;

        // --- MANUAL DRIVER CONTROLS ---
        // Inverse direction toggle (D-Pad Up on Gamepad 1)
        boolean dpadUp = gamepad1.dpad_up;
        if (dpadUp && !dpadUpLast) {
            inverseDirection = !inverseDirection;
        }
        dpadUpLast = dpadUp;

        // Drive speed toggle
        if (gamepad1.a && !aPressedLast) driveSpeed = 1.0;
        if (gamepad1.b && !bPressedLast) driveSpeed = 0.5;
        aPressedLast = gamepad1.a;
        bPressedLast = gamepad1.b;

        // Get standard controls from left stick
        float forward = gamepad1.left_stick_y;
        float strafe = -gamepad1.left_stick_x;
        float turn = -gamepad1.right_stick_x;

        // Add diagonal forward/backward movement from right stick Y axis
        float diagonalForward = gamepad1.right_stick_y;
        forward += diagonalForward;

        // Apply inverse direction if enabled
        if (inverseDirection) {
            forward = -forward;
            strafe = -strafe;
            turn = -turn;
        }

        // Apply deadband to prevent stick drift
        forward = applyDeadband(forward, STICK_DEADBAND);
        strafe = applyDeadband(strafe, STICK_DEADBAND);
        turn = applyDeadband(turn, STICK_DEADBAND);

        // Apply cubic scaling for smoother low-speed control
        forward = (float) Math.copySign(Math.pow(Math.abs(forward), CUBIC_INPUT_POWER), forward);
        strafe = (float) Math.copySign(Math.pow(Math.abs(strafe), CUBIC_INPUT_POWER), strafe);
        turn = (float) Math.copySign(Math.pow(Math.abs(turn), CUBIC_INPUT_POWER), turn);

        // Normalize powers to prevent exceeding 1.0
        double max = Math.max(1.0, Math.abs(forward) + Math.abs(strafe) + Math.abs(turn));
        forward /= max;
        strafe /= max;
        turn /= max;

        drive.drive(forward * driveSpeed, strafe * driveSpeed, turn * driveSpeed);
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
        boolean rb = gamepad2.right_bumper;
        if ((currentMode == RobotMode.SHOOTING_READY || currentMode == RobotMode.SHOOTING_SETUP || rb) 
                && !intakeMode) {
            shooter.setTargetRPM(SHOOTER_TARGET_RPM);
        } else {
            shooter.stop();
        }
        rbPressedLast = rb;

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

        // Manual Spindexer (Gamepad 2 Right Stick) - Cubic Scaling for Smooth Control
        if (currentMode == RobotMode.MANUAL_OVERRIDE) {
            double raw = -gamepad2.right_stick_y;

            // Deadband
            if (Math.abs(raw) < STICK_DEADBAND) {
                spindexer.stopManual();
            } else {
                // Remove deadband and normalize to [0, 1]
                double normalized = (Math.abs(raw) - STICK_DEADBAND) / (1.0 - STICK_DEADBAND);
                normalized = Math.copySign(normalized, raw);

                // Cubic scaling (smooth near center, full power at extremes)
                double scaled = Math.copySign(Math.pow(Math.abs(normalized), SPINDEXER_CUBIC_POWER), normalized);

                // Final power with safety scale
                spindexer.setManualPower(scaled * SPINDEXER_MANUAL_SCALE);
            }

            // D-Pad Right: Manual position increment
            boolean dpadRight = gamepad2.dpad_right;
            if (dpadRight && !dpadRightLast && spindexer.isAtPosition()) {
                spindexerPositionIndex = (spindexerPositionIndex + 1) % 3;
                spindexer.setPIDEnabled(true);
                spindexer.goToPositionForCurrentMode(spindexerPositionIndex);
            }
            dpadRightLast = dpadRight;
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
        telemetry.addLine("D↓ = Reset | D← = Toggle Mode | RB = Spin");
        telemetry.addLine("RT = Shoot | LT = Intake | LB = Reverse");
        telemetry.addLine();
        
        telemetry.addData("PHASE", currentMode);
        telemetry.addData("TARGET SLOT", spindexerPositionIndex + 1);
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
        
        // Motif detection telemetry (locked at start)
        if (lockedMotif != null) {
            telemetry.addData("MOTIF", motifToString(lockedMotif) + " (Locked)");
            telemetry.addData("OBELISK TAG", "ID " + lockedMotifTagId);
        } else {
            telemetry.addData("MOTIF", "NOT DETECTED");
        }
        
        telemetry.update();
    }

    /**
     * Helper method to convert ArtifactColor[] motif to string representation
     */
    private String motifToString(ArtifactColor[] motif) {
        if (motif == null) return "";
        StringBuilder sb = new StringBuilder();
        for (ArtifactColor c : motif) {
            sb.append(c.getCharacter());
        }
        return sb.toString();
    }
}
