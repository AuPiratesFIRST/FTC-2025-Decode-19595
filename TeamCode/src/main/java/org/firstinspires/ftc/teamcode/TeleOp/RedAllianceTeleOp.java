package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
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
import static com.pedropathing.ivy.commands.Commands.*;

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
    private int spindexerPositionIndex = 0;
    private boolean intakeMode = true; // true = Intake, false = Outtake
    private double driveSpeed = 1.0;

    // Input Latches
    private boolean lastInputA = false;
    private boolean lastInputB = false;
    private boolean lastInputX = false;
    private boolean lastInputDpadDown = false;
    private boolean lastInputDpadLeft = false;
    private boolean lastInputDpadRight = false;

    // === DRIVE INPUT CONSTANTS ===
    private static final double STICK_DEADBAND = 0.05;     // Prevent drift from stick noise
    private static final double CUBIC_INPUT_POWER = 3.0;   // Smooth low-speed control

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

    // Shooter RPM target - uses centralized value from ShooterSubsystem

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
    private Command teleOpLoopCommand;
    private Command intakeCommand;
    private Command activeShotSequence;
    private Command alignCommand;
    private Command manualShooterCommand;

    @Override
    public void runOpMode() {
        Scheduler.reset();
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
        aimController.setDesiredDistance(134);
        aimController.setDesiredAngle(21);
        aimController.setGains(0.079, 0.075, 0.020);  // TagChaserOp-tuned
        aimController.setStrafeKd(0.030);
        aimController.setMaxPower(0.40);
        aimController.setDeadbands(1.0, 0.75, 1.5);   // forward (in), strafe (in), turn (deg)
        aimController.setTargetTagId(TARGET_TAG_ID);
        aimController.setVisionLossTimeout(500);

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

        teleOpLoopCommand = Command.build()
                .setExecute(this::runTeleOpStep)
                .setDone(() -> !opModeIsActive())
                .setEnd(endCondition -> {
                    drive.stop();
                    intake.stop();
                    shooter.stop();
                    funnel.retract();
                });
        Scheduler.schedule(teleOpLoopCommand);
        intakeCommand = infinite(this::applyIntakePower);
        Scheduler.schedule(intakeCommand);
        alignCommand = infinite(() -> {
            AimController.AlignmentResult result = aimController.update();
            drive.drive(result.forward, result.strafe, result.turn);
        }).requiring(drive);
        manualShooterCommand = infinite(() -> shooter.setTargetRPM(ShooterSubsystem.TARGET_RPM))
                .requiring(shooter);

        while (opModeIsActive()) {
            Scheduler.execute();
            updateTelemetry();
        }

        // Cleanup
        drive.stop();
        intake.stop();
        shooter.stop();
        funnel.retract(); // Retract funnels on stop
        aprilTag.closeVision();
    }

    private void runTeleOpStep() {
        shooter.updateVoltageCompensation();

        if (gamepad2.x && !lastInputX) {
            if (currentMode == RobotMode.MANUAL_OVERRIDE) currentMode = RobotMode.INTAKE;
            else {
                currentMode = RobotMode.MANUAL_OVERRIDE;
                spindexer.lockCurrentPosition();
            }
        }
        lastInputX = gamepad2.x;

        if (gamepad2.dpad_left && !lastInputDpadLeft) {
            intakeMode = !intakeMode;
            spindexer.setIntakeMode(intakeMode);
            spindexerPositionIndex = 0;
            spindexer.goToPositionForCurrentMode(0);

            if (intakeMode) {
                currentMode = RobotMode.INTAKE;
            }
        }
        lastInputDpadLeft = gamepad2.dpad_left;

        if (currentMode != RobotMode.MANUAL_OVERRIDE) {
            handleCircularTransitions();
        }
        handleAlignmentCommand();
        if (!isAlignActive()) {
            handleDriveManual();
        }
        executeHardwareActions();
        spindexer.update();
    }

    private void handleAlignmentCommand() {
        if (gamepad2.y) {
            if (!isAlignActive()) {
                alignCommand.schedule();
            }
            return;
        }
        if (isAlignActive()) {
            alignCommand.cancel();
            drive.stop();
        }
    }

    private void handleCircularTransitions() {
        if (gamepad2.b && !lastInputB && intakeMode) {
            spindexerPositionIndex = 0;
            scheduleShotSequenceForIndex(spindexerPositionIndex);
        }
        lastInputB = gamepad2.b;

        if (gamepad2.a && !lastInputA && !intakeMode && !isShotSequenceActive()) {
            spindexerPositionIndex++;
            if (spindexerPositionIndex > 2) {
                resetToIntakeMode();
            } else {
                scheduleShotSequenceForIndex(spindexerPositionIndex);
            }
        }
        lastInputA = gamepad2.a;

        if (gamepad2.dpad_down && !lastInputDpadDown) {
            if (isShotSequenceActive()) {
                activeShotSequence.cancel();
            }
            resetToIntakeMode();
        }
        lastInputDpadDown = gamepad2.dpad_down;
    }

    private void handleDriveManual() {
        if (gamepad1.a) driveSpeed = 1.0;
        if (gamepad1.b) driveSpeed = 0.5;

        // Apply deadband to prevent stick drift (combine sticks before deadband)
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
        if (gamepad2.right_bumper && !isShotSequenceActive()) {
            if (!isManualShooterActive()) {
                manualShooterCommand.schedule();
            }
        } else if (isManualShooterActive()) {
            manualShooterCommand.cancel();
            if (intakeMode) {
                shooter.stop();
            }
        }

        if (gamepad2.right_trigger > 0.5
                && currentMode == RobotMode.MANUAL_OVERRIDE
                && spindexer.isAtPosition()
                && !isShotSequenceActive()) {
            scheduleFunnelPulse();
        }

        // Manual Spindexer (Gamepad 2 Right Stick) - Cubic Scaling for Smooth Control
        if (currentMode == RobotMode.MANUAL_OVERRIDE) {
            if (funnel.isExtended()) {
                // Hard interlock: never allow manual spindexer movement while funnel is out.
                spindexer.stopManual();
                return;
            }
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
            if (gamepad2.dpad_right && !lastInputDpadRight && spindexer.isAtPosition()) {
                spindexerPositionIndex = (spindexerPositionIndex + 1) % 3;
                spindexer.setPIDEnabled(true);
                spindexer.goToPositionForCurrentMode(spindexerPositionIndex);
            }
            lastInputDpadRight = gamepad2.dpad_right;
        }
    }

    /**
     * KEEPER WHEEL PATTERN: Intake always runs at low hold power, overridable by driver.
     * Priority: Reverse > Full Intake > Hold Power
     */
    private void applyIntakePower() {
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
        telemetry.addData("SHOT SEQUENCE", isShotSequenceActive() ? "RUNNING" : "IDLE");
        
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

    private void scheduleShotSequenceForIndex(int index) {
        if (isShotSequenceActive()) {
            return;
        }
        activeShotSequence = instant(() -> {
            intakeMode = false;
            currentMode = RobotMode.SHOOTING_SETUP;
            spindexer.setIntakeMode(false);
            spindexer.goToPositionForwardOnly(OldSpindexerSubsystem.OUTTAKE_POSITIONS[index]);
        }).then(
                waitUntil(spindexer::isAtPosition),
                instant(() -> {
                    currentMode = RobotMode.SHOOTING_READY;
                    shooter.setTargetRPM(ShooterSubsystem.TARGET_RPM);
                }),
                waitUntil(() -> shooter.isAtTargetRPM()
                        || Math.abs(shooter.getCurrentRPM() - ShooterSubsystem.TARGET_RPM) < 100),
                instant(funnel::extend),
                waitMs(FUNNEL_EXTEND_TIME_MS),
                waitMs(FUNNEL_HOLD_TIME_MS),
                instant(funnel::retract),
                waitMs(FUNNEL_EXTEND_TIME_MS),
                instant(() -> currentMode = RobotMode.SHOOTING_SETUP)
        ).setEnd(endCondition -> {
            if (intakeMode) {
                shooter.stop();
            }
        }).requiring(spindexer, shooter, funnel);
        activeShotSequence.schedule();
    }

    private void scheduleFunnelPulse() {
        activeShotSequence = instant(funnel::extend).then(
                waitMs(FUNNEL_EXTEND_TIME_MS),
                waitMs(FUNNEL_HOLD_TIME_MS),
                instant(funnel::retract),
                waitMs(FUNNEL_EXTEND_TIME_MS)
        ).requiring(funnel);
        activeShotSequence.schedule();
    }

    private boolean isShotSequenceActive() {
        return activeShotSequence != null && activeShotSequence.isScheduled();
    }

    private boolean isAlignActive() {
        return alignCommand != null && alignCommand.isScheduled();
    }

    private boolean isManualShooterActive() {
        return manualShooterCommand != null && manualShooterCommand.isScheduled();
    }

    private void resetToIntakeMode() {
        intakeMode = true;
        spindexerPositionIndex = 0;
        spindexer.setIntakeMode(true);
        spindexer.goToPositionForCurrentMode(0);
        currentMode = RobotMode.INTAKE;
        funnel.retract();
        if (isManualShooterActive()) {
            manualShooterCommand.cancel();
        }
        shooter.stop();
    }
}
