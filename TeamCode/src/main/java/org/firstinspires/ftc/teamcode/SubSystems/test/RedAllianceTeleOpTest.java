package org.firstinspires.ftc.teamcode.SubSystems.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.teamcode.SubSystems.Sensors.ColorSensorSubsystem;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import org.firstinspires.ftc.teamcode.SubSystems.Drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Shooter.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Spindexer.OldSpindexerSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Vision.AprilTagNavigator;
import org.firstinspires.ftc.teamcode.SubSystems.Vision.ObeliskMotifDetector;
import org.firstinspires.ftc.teamcode.SubSystems.Scoring.AutoOuttakeController;
import org.firstinspires.ftc.teamcode.SubSystems.Scoring.ArtifactColor;
import org.firstinspires.ftc.teamcode.SubSystems.Funnel.FunnelSubsystem;


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
    // Default field pattern (PGP / ID 22). Change here if event field differs.
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
    
    private static final double SHOOTER_TARGET_RPM = 5220.0;
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
    
    // Funnel State (Toggle)
    private boolean funnelActive = false;

    // === INPUT LATCHES ===
    private boolean lastInputX = false;
    private boolean lastInputRB = false;
    private boolean lastInputDpadDown = false;
    private boolean lastInputDpadLeft = false;
    private boolean lastInputDpadRight = false;
    private boolean lastInputStart = false;
    private boolean lastInputA = false;
    private boolean lastInputB = false;
    private boolean lastInputFunnel = false; // Added for Funnel

    @Override
    public void runOpMode() {
        // 1. Initialize Subsystems
        drive = new DriveSubsystem(hardwareMap, telemetry);
        intake = new IntakeSubsystem(hardwareMap, telemetry);
        shooter = new ShooterSubsystem(hardwareMap, telemetry);
        spindexer = new OldSpindexerSubsystem(hardwareMap, telemetry);
        aprilTag = new AprilTagNavigator(drive, hardwareMap, telemetry);
        funnel = new FunnelSubsystem(hardwareMap, telemetry);

        // 2. Initialize Vision/Color
        ArtifactColor[] motif = DEFAULT_MOTIF;

        try {
            NormalizedColorSensor normColor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
            colorSensorSubsystem = new ColorSensorSubsystem(normColor, null, telemetry, motif);
        } catch (Exception e) {
            colorSensorSubsystem = null;
            telemetry.addData("ColorSensor", "Not found");
        }

        obeliskDetector = new ObeliskMotifDetector(aprilTag, telemetry);

        // 3. Initialize Auto Outtake
        if (colorSensorSubsystem != null) {
            autoOuttake = new AutoOuttakeController(colorSensorSubsystem, motif, spindexer, shooter, funnel, telemetry);
            autoOuttake.setScoreThreshold(6);
            autoOuttake.setTargetShooterRPM(SHOOTER_TARGET_RPM);
        }

        telemetry.addData("Status", "Initialized - RED ALLIANCE FINAL");
        telemetry.update();

        // 4. Scan during Init
        long initStart = System.currentTimeMillis();
        while (!isStarted() && !isStopRequested() && System.currentTimeMillis() - initStart < 3000) {
            obeliskDetector.update();
            telemetry.update();
            sleep(50);
        }

        waitForStart();

        // 5. Final Motif Check
        ArtifactColor[] detectedMotif = obeliskDetector.getMotif();
        if (detectedMotif == null) detectedMotif = DEFAULT_MOTIF;
        if (autoOuttake != null) autoOuttake.updateMotif(detectedMotif);

        // === MAIN LOOP ===
        while (opModeIsActive()) {
            updateInputTracking();
            updateRobotState();
            
            executeDrive();
            executeIntake();
            executeSpindexerAndShooter();
            executeFunnel(); // <--- ADDED THIS

            shooter.updateVoltageCompensation();
            obeliskDetector.update();
            aprilTag.initializeCameraControls();
            if (autoOuttake != null) autoOuttake.update();

            updateTelemetry();
        }

        drive.stop();
        intake.stop();
        shooter.stop();
        aprilTag.closeVision();
    }

    // =========================================================
    // SECTION 1: INPUT TRACKING
    // =========================================================
    private void updateInputTracking() {
        
        // Manual Mode Toggle
        if (gamepad2.x && !lastInputX) {
            currentMode = (currentMode == RobotMode.MANUAL_OVERRIDE) ? RobotMode.INTAKE : RobotMode.MANUAL_OVERRIDE;
        }
        lastInputX = gamepad2.x;

        // Intake/Outtake Toggle
        if (gamepad2.dpad_down && !lastInputDpadDown && currentMode != RobotMode.MANUAL_OVERRIDE) {
            intakeMode = !intakeMode;
            spindexerIndex = 0;
            spindexer.setIntakeMode(intakeMode);
        }
        lastInputDpadDown = gamepad2.dpad_down;

        // Shooter Manual Toggle
        if (gamepad2.right_bumper && !lastInputRB) {
            shooterManualToggle = !shooterManualToggle;
        }
        lastInputRB = gamepad2.right_bumper;

        // Funnel Toggle (Right Trigger treated as button)
        boolean funnelPressed = gamepad2.right_trigger > 0.5;
        if (funnelPressed && !lastInputFunnel) {
            funnelActive = !funnelActive; // Flip state
        }
        lastInputFunnel = funnelPressed;

        // Drive Speed
        if (gamepad1.a) driveSpeed = 1.0;
        if (gamepad1.b) driveSpeed = 0.5;

        // IMU Reset
        if (gamepad1.start && !lastInputStart) {
            drive.resetHeading();
        }
        lastInputStart = gamepad1.start;
    }

    // =========================================================
    // SECTION 2: ROBOT STATE
    // =========================================================
    private void updateRobotState() {
        if (autoOuttake != null && autoOuttake.getState() != AutoOuttakeController.State.IDLE) {
            return;
        }

        if (currentMode == RobotMode.MANUAL_OVERRIDE) {
            // Manual Indexing
            if (gamepad2.dpad_left && !lastInputDpadLeft) {
                spindexerIndex = (spindexerIndex - 1 + 3) % 3;
                spindexer.goToPosition(spindexerIndex);
            }
            if (gamepad2.dpad_right && !lastInputDpadRight) {
                spindexerIndex = (spindexerIndex + 1) % 3;
                spindexer.goToPosition(spindexerIndex);
            }
        } else {
            // Auto Indexing
            if (gamepad2.b && !lastInputB) {
                ArtifactColor[] detected = obeliskDetector.getMotif();
                if (detected == null) detected = DEFAULT_MOTIF;
                
                spindexer.setIntakeMode(false);
                spindexer.rotateToMotifStartPosition(detected);
                spindexerIndex = OldSpindexerSubsystem.getStartingPositionFromMotif(detected);
                currentMode = RobotMode.SHOOTING_SETUP;
            }

            if (gamepad2.a && !lastInputA) {
                spindexerIndex = (spindexerIndex + 1) % 3;
                spindexer.goToPositionForCurrentMode(spindexerIndex);
                currentMode = RobotMode.SHOOTING_SETUP;
            }

            if (currentMode == RobotMode.SHOOTING_SETUP && spindexer.isAtPosition()) {
                currentMode = RobotMode.SHOOTING_READY;
            }
        }

        lastInputA = gamepad2.a;
        lastInputB = gamepad2.b;
        lastInputDpadLeft = gamepad2.dpad_left;
        lastInputDpadRight = gamepad2.dpad_right;
    }

    // =========================================================
    // SECTION 3: HARDWARE EXECUTION
    // =========================================================

    private void executeFunnel() {
        // STANDARD SERVO LOGIC:
        // True  = Extend (Push cam out)
        // False = Retract (Return to rest)
        if (funnelActive) {
            funnel.extend();
        } else {
            funnel.retract();
        }
    }

    private void executeSpindexerAndShooter() {
        // Spindexer
        if (currentMode == RobotMode.MANUAL_OVERRIDE) {
            float manualPower = -gamepad2.right_stick_y;
            if (Math.abs(manualPower) > 0.1) {
                spindexer.setManualPower(manualPower * OldSpindexerSubsystem.getRecommendedManualPowerMultiplier());
            } else {
                spindexer.setManualPower(0);
            }
        } else {
            spindexer.update();
        }

        // Shooter
        boolean shouldSpin = false;
        if (shooterManualToggle) shouldSpin = true;
        if (gamepad2.y) shouldSpin = true; // Spin while holding Align
        if (currentMode == RobotMode.SHOOTING_READY && !intakeMode) shouldSpin = true;

        // Safety
        if (spindexer.isMoving() && !shooterManualToggle) {
            shouldSpin = false;
        }

        if (shouldSpin) shooter.setTargetRPM(SHOOTER_TARGET_RPM);
        else shooter.stop();
    }

    private void executeDrive() {
        // Auto Align
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

        // Manual Drive
        float forward = gamepad1.left_stick_y + gamepad1.right_stick_y;
        float strafe = -gamepad1.left_stick_x;
        float turn = -gamepad1.right_stick_x;
        double denominator = JavaUtil.maxOfList(JavaUtil.createListWith(0.5, 0.5 + Math.abs(turn)));
        
        if (gamepad2.y) turn = 0; // Lock heading while aligning

        drive.drive(
                (forward / denominator) * driveSpeed,
                (strafe / denominator) * driveSpeed,
                (turn / denominator) * driveSpeed);
    }

    private void executeIntake() {
        if (gamepad2.left_trigger > 0.1) intake.setPower(1.0);
        else if (gamepad2.left_bumper) intake.setPower(-1.0);
        else intake.setPower(0);
    }

    // =========================================================
    // SECTION 4: TELEMETRY
    // =========================================================
    private void updateTelemetry() {
        telemetry.addData("Mode", currentMode.toString());
        telemetry.addData("Intake", intakeMode ? "INTAKE" : "OUTTAKE");
        telemetry.addData("Shooter Manual", shooterManualToggle);
        telemetry.addData("Spindexer Index", spindexerIndex);
        telemetry.addData("RPM", "%.0f / %.0f", shooter.getCurrentRPM(), shooter.getTargetRPM());
        telemetry.addData("Spindexer At Position", spindexer.isAtPosition());
        telemetry.update();
    }
}

