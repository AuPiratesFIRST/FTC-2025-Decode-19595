package org.firstinspires.ftc.teamcode.SubSystems.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.teamcode.SubSystems.Drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Shooter.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Spindexer.OldSpindexerSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Vision.AprilTagNavigator;
import org.firstinspires.ftc.teamcode.SubSystems.Vision.ObeliskMotifDetector;
import org.firstinspires.ftc.teamcode.SubSystems.Scoring.AutoOuttakeController;
import org.firstinspires.ftc.teamcode.SubSystems.Scoring.ArtifactColor;
import org.firstinspires.ftc.teamcode.SubSystems.Funnel.FunnelSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Sensors.ColorSensorSubsystem;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name = "Red Alliance TeleOp TEST", group = "TeleOp")
public class RedAllianceTeleOpTest extends LinearOpMode {

    // === ROBOT STATE ENUM ===
    private enum RobotMode {
        INTAKE,                 // Spindexer/Shooter idle
        SHOOTING_SETUP,         // Spindexer moving to slot
        SHOOTING_READY,         // Spindexer ready, spinning up shooter
        MANUAL_OVERRIDE         // User manually controlling spindexer
    }

    // === CONSTANTS ===
    private static final ArtifactColor[] DEFAULT_MOTIF = {
            ArtifactColor.PURPLE, ArtifactColor.GREEN, ArtifactColor.PURPLE
    };

    private static final double DESIRED_SHOOTING_DISTANCE = 134;
    private static final double DESIRED_SHOOTING_ANGLE = 21;
    private static final double KP_STRAFE = 0.03;
    private static final double KP_FORWARD = 0.03;
    private static final double KP_ROT = 0.015;
    private static final double MAX_AUTO_POWER = 0.40;
    private static final double POSITION_DEADBAND = 0.75;
    private static final double ANGLE_DEADBAND_DEG = 1.5;

    private static final double SHOOTER_TARGET_RPM = 5225.0;
    private static final int TARGET_TAG_ID = 24;

    // === SUBSYSTEMS ===
    private DriveSubsystem drive;
    private IntakeSubsystem intake;
    private ShooterSubsystem shooter;
    private OldSpindexerSubsystem spindexer;
    private AprilTagNavigator aprilTag;
    private ColorSensorSubsystem colorSensorSubsystem;
    private ObeliskMotifDetector obeliskDetector;
    private FunnelSubsystem funnel;
    private AutoOuttakeController autoOuttake;

    // === STATE VARIABLES ===
    private RobotMode currentMode = RobotMode.INTAKE;
    private int spindexerIndex = 0;
    private boolean shooterManualToggle = false;
    private boolean intakeMode = true;
    private double driveSpeed = 1.0;
    private boolean funnelActive = false;
    private boolean spindexerManualMoving = false;

    // === INPUT LATCHES ===
    private boolean lastInputX = false;
    private boolean lastInputRB = false;
    private boolean lastInputDpadDown = false;
    private boolean lastInputDpadLeft = false;
    private boolean lastInputDpadRight = false;
    private boolean lastInputStart = false;
    private boolean lastInputA = false;
    private boolean lastInputB = false;
    private boolean lastInputFunnel = false;

    @Override
    public void runOpMode() {
        // --- 1. SUBSYSTEM INITIALIZATION ---
        drive = new DriveSubsystem(hardwareMap, telemetry);
        intake = new IntakeSubsystem(hardwareMap, telemetry);
        shooter = new ShooterSubsystem(hardwareMap, telemetry);
        spindexer = new OldSpindexerSubsystem(hardwareMap, telemetry);
        aprilTag = new AprilTagNavigator(drive, hardwareMap, telemetry);
        funnel = new FunnelSubsystem(hardwareMap, telemetry);

        ArtifactColor[] motif = DEFAULT_MOTIF;

        try {
            NormalizedColorSensor normColor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
            colorSensorSubsystem = new ColorSensorSubsystem(normColor, null, telemetry, motif);
        } catch (Exception e) {
            colorSensorSubsystem = null;
        }

        obeliskDetector = new ObeliskMotifDetector(aprilTag, telemetry);

        if (colorSensorSubsystem != null) {
            autoOuttake = new AutoOuttakeController(colorSensorSubsystem, motif, spindexer, shooter, funnel, telemetry);
            autoOuttake.setScoreThreshold(6);
            autoOuttake.setTargetShooterRPM(SHOOTER_TARGET_RPM);
        }

        waitForStart();

        // Detect motif once at start
        ArtifactColor[] detectedMotif = obeliskDetector.getMotif();
        if (detectedMotif == null) detectedMotif = DEFAULT_MOTIF;
        if (autoOuttake != null) autoOuttake.updateMotif(detectedMotif);

        while (opModeIsActive()) {
            // --- 2. UPDATE SENSORS & GLOBAL SYSTEMS ---
            shooter.updateVoltageCompensation();
            obeliskDetector.update();
            aprilTag.initializeCameraControls();
            if (autoOuttake != null) autoOuttake.update();

            // --- 3. INPUT TRACKING & STATE TRANSITIONS ---
            updateInputTracking();
            updateStateTransitions();

            // --- 4. EXECUTE STATE LOGIC ---
            executeStateActions();

            // ✅ ALWAYS RUN PID EVERY LOOP
            spindexer.update();

            // --- 5. PERIPHERAL SYSTEMS ---
            executeDrive();
            executeIntake();
            executeFunnel();

            // --- 6. TELEMETRY ---
            updateTelemetry();
        }

        // --- SHUTDOWN ---
        drive.stop();
        intake.stop();
        shooter.stop();
        aprilTag.closeVision();
    }

    private void updateInputTracking() {
        // Manual Override Toggle
        if (gamepad2.x && !lastInputX) {
            if (currentMode == RobotMode.MANUAL_OVERRIDE) {
                currentMode = RobotMode.INTAKE;
            } else {
                currentMode = RobotMode.MANUAL_OVERRIDE;
                spindexer.lockCurrentPosition();
            }
        }
        lastInputX = gamepad2.x;

        // Drive Controls
        if (gamepad1.a) driveSpeed = 1.0;
        if (gamepad1.b) driveSpeed = 0.5;
        if (gamepad1.start && !lastInputStart) drive.resetHeading();
        lastInputStart = gamepad1.start;

        // Shooter & Funnel Latches
        if (gamepad2.right_bumper && !lastInputRB) shooterManualToggle = !shooterManualToggle;
        lastInputRB = gamepad2.right_bumper;

        boolean funnelPressed = gamepad2.right_trigger > 0.5;
        if (funnelPressed && !lastInputFunnel) funnelActive = !funnelActive;
        lastInputFunnel = funnelPressed;
    }

    private void updateStateTransitions() {
        // If AutoOuttake is running, it owns the state. Do not allow manual changes.
        if (autoOuttake != null && autoOuttake.getState() != AutoOuttakeController.State.IDLE) return;

        // Handle Manual Mode D-Pad movement separately
        if (currentMode == RobotMode.MANUAL_OVERRIDE) {
            if (gamepad2.dpad_left && !lastInputDpadLeft) {
                spindexerIndex = (spindexerIndex - 1 + 3) % 3;
                spindexer.goToPosition(spindexerIndex);
            }
            if (gamepad2.dpad_right && !lastInputDpadRight) {
                spindexerIndex = (spindexerIndex + 1) % 3;
                spindexer.goToPosition(spindexerIndex);
            }
            updateInternalLatches();
            return;
        }

        // --- Standard State Transitions ---

        // RESET: Return to Intake
        if (gamepad2.dpad_down && !lastInputDpadDown) {
            intakeMode = true;
            spindexerIndex = 0;
            spindexer.setIntakeMode(true);
            currentMode = RobotMode.INTAKE;
        }

        // START SHOOTING: Rotate to Motif
        if (gamepad2.b && !lastInputB) {
            ArtifactColor[] detected = obeliskDetector.getMotif();
            intakeMode = false;
            spindexer.setIntakeMode(false);
            spindexer.rotateToMotifStartPosition(detected != null ? detected : DEFAULT_MOTIF);
            spindexerIndex = 0;
            currentMode = RobotMode.SHOOTING_SETUP;
        }

        // NEXT SLOT: Manual Indexing
        if (gamepad2.a && !lastInputA) {
            intakeMode = false;
            spindexer.setIntakeMode(false);
            spindexerIndex = (spindexerIndex + 1) % 3;
            spindexer.goToPositionForCurrentMode(spindexerIndex);
            currentMode = RobotMode.SHOOTING_SETUP;
        }

        // AUTO TRANSITION: SETUP -> READY
        if (currentMode == RobotMode.SHOOTING_SETUP && spindexer.isAtPosition()) {
            currentMode = RobotMode.SHOOTING_READY;
        }

        updateInternalLatches();
    }

    private void updateInternalLatches() {
        lastInputA = gamepad2.a;
        lastInputB = gamepad2.b;
        lastInputDpadDown = gamepad2.dpad_down;
        lastInputDpadLeft = gamepad2.dpad_left;
        lastInputDpadRight = gamepad2.dpad_right;
    }

    private void executeStateActions() {
        // Skip execution if AutoOuttake is managing subsystems
        if (autoOuttake != null && autoOuttake.getState() != AutoOuttakeController.State.IDLE) return;

        switch (currentMode) {
            case INTAKE:
                handleShooterLogic(false);
                break;

            case SHOOTING_SETUP:
                // We keep shooter off while spindexer is making large moves
                handleShooterLogic(false);
                break;

            case SHOOTING_READY:
                handleShooterLogic(true);
                break;

            case MANUAL_OVERRIDE:
                handleManualSpindexerStick();
                // Allow manual trigger/bumper firing in manual mode
                handleShooterLogic(gamepad2.y || shooterManualToggle);
                break;
        }
    }

    private void handleManualSpindexerStick() {
        float manualPower = -gamepad2.right_stick_y;
        if (Math.abs(manualPower) > 0.1) {
            spindexer.setManualPower(manualPower * OldSpindexerSubsystem.getRecommendedManualPowerMultiplier());
            spindexerManualMoving = true;
        } else {
            spindexerManualMoving = false;
        }
        // No lock, no PID reset - let PID maintain position naturally
    }

    private void handleShooterLogic(boolean stateWantsSpin) {
        // Priority: If Spindexer is actively rotating, force shooter off (safety/battery)
        // Unless we are in manual mode or have a manual toggle override.
        if (spindexer.isMoving() && !shooterManualToggle && currentMode != RobotMode.MANUAL_OVERRIDE) {
            stateWantsSpin = false;
        }

        if (stateWantsSpin || shooterManualToggle || gamepad2.y) {
            shooter.setTargetRPM(SHOOTER_TARGET_RPM);
        } else {
            shooter.stop();
        }
    }

    private void executeDrive() {
        if (gamepad2.y) {
            aprilTag.updateRobotPositionFromAllianceGoals();
            AprilTagDetection tag = aprilTag.getBestAllianceGoalDetection();
            if (tag != null && tag.id == TARGET_TAG_ID) {
                double[] corrections = aprilTag.calculateAlignmentCorrections(
                        tag, DESIRED_SHOOTING_DISTANCE, DESIRED_SHOOTING_ANGLE,
                        POSITION_DEADBAND, POSITION_DEADBAND, ANGLE_DEADBAND_DEG,
                        KP_STRAFE, KP_FORWARD, KP_ROT, MAX_AUTO_POWER);
                if (corrections != null) {
                    drive.drive(corrections[1], corrections[0], corrections[2]);
                    return;
                }
            }
        }

        float forward = gamepad1.left_stick_y + gamepad1.right_stick_y;
        float strafe = -gamepad1.left_stick_x;
        float turn = -gamepad1.right_stick_x;
        if (gamepad2.y) turn = 0;

        double denominator = Math.max(1.0, 0.5 + Math.abs(turn));
        drive.drive((forward / denominator) * driveSpeed, (strafe / denominator) * driveSpeed, (turn / denominator) * driveSpeed);
    }

    private void executeIntake() {
        if (gamepad2.left_trigger > 0.1) intake.setPower(1.0);
        else if (gamepad2.left_bumper) intake.setPower(-1.0);
        else intake.setPower(0);
    }

    private void executeFunnel() {
        if (funnelActive) funnel.extend();
        else funnel.retract();
    }

    private void updateTelemetry() {
        telemetry.addData("--- SYSTEM STATE ---", "");
        telemetry.addData("Current Mode", currentMode);
        telemetry.addData("Spindexer Index", spindexerIndex);
        telemetry.addData("Spindexer Moving", spindexer.isMoving());
        telemetry.addData("Shooter RPM", "%.0f / %.0f", shooter.getCurrentRPM(), shooter.getTargetRPM());
        telemetry.addData("Manual Lock", spindexerManualMoving ? "USER CONTROL" : "LOCKED");
        telemetry.update();
    }
}