package org.firstinspires.ftc.teamcode.SubSystems.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.teamcode.SubSystems.Drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Shooter.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Spindexer.OldSpindexerSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Vision.AprilTagNavigator;
import org.firstinspires.ftc.teamcode.SubSystems.Vision.ObeliskMotifDetector;
import org.firstinspires.ftc.teamcode.SubSystems.Sensors.ColorSensorSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Scoring.AutoOuttakeController;
import org.firstinspires.ftc.teamcode.SubSystems.Scoring.ArtifactColor;
import org.firstinspires.ftc.teamcode.SubSystems.Scoring.PatternScorer;
import org.firstinspires.ftc.teamcode.SubSystems.Funnel.FunnelSubsystem;

/**
 * Comprehensive Subsystem Test OpMode
 * 
 * Tests all subsystems in sequence:
 * 1. AprilTag Detection & Localization
 * 2. Obelisk Motif Detection
 * 3. Color Sensor & Pattern Detection
 * 4. Drive System (forward, strafe, turn)
 * 5. Intake System
 * 6. Spindexer Positions (all 3 positions in both modes)
 * 7. Shooter RPM Control
 * 8. Auto Outtake Controller
 * 9. Integrated Shooting Sequence (with shooter stopping during spindexer movement)
 * 
 * Controls:
 * - Gamepad 1: Drive controls
 * - Gamepad 2: Subsystem controls
 *   - A: Next test phase
 *   - B: Previous test phase
 *   - X: Run current test
 *   - Y: Stop all systems
 *   - D-Pad: Manual spindexer control
 *   - Triggers: Intake control
 *   - Bumpers: Shooter control
 */
@TeleOp(name = "Comprehensive Subsystem Test", group = "Test")
public class ComprehensiveSubsystemTest extends LinearOpMode {

    private DriveSubsystem drive;
    private IntakeSubsystem intake;
    private ShooterSubsystem shooter;
    private OldSpindexerSubsystem spindexer;
    private AprilTagNavigator aprilTag;
    private ObeliskMotifDetector obeliskDetector;
    private ColorSensorSubsystem colorSensorSubsystem;
    private AutoOuttakeController autoOuttake;
    private NormalizedColorSensor colorSensor;
    private FunnelSubsystem funnel;

    private enum TestPhase {
        APRILTAG_DETECTION,
        OBELISK_MOTIF,
        COLOR_SENSOR,
        DRIVE_TEST,
        INTAKE_TEST,
        SPINDEXER_POSITIONS,
        SHOOTER_RPM,
        AUTO_OUTTAKE,
        INTEGRATED_SHOOTING
    }

    private TestPhase currentPhase = TestPhase.APRILTAG_DETECTION;
    private int phaseIndex = 0;
    private boolean testRunning = false;
    private ElapsedTime testTimer = new ElapsedTime();
    private ElapsedTime phaseTimer = new ElapsedTime();

    // Button state tracking
    private boolean aPressedLast = false;
    private boolean bPressedLast = false;
    private boolean xPressedLast = false;
    private boolean yPressedLast = false;
    private boolean dpadUpLast = false;
    private boolean dpadDownLast = false;

    // Test state
    private int spindexerTestPosition = 0;
    private boolean shooterWasRunning = false;
    private static final double SHOOTER_TARGET_RPM = 5210.0;
    private ArtifactColor[] testMotif = new ArtifactColor[] {
        ArtifactColor.GREEN, ArtifactColor.PURPLE, ArtifactColor.GREEN
    };

    @Override
    public void runOpMode() {
        // Initialize subsystems
        telemetry.addData("Status", "Initializing subsystems...");
        telemetry.update();

        drive = new DriveSubsystem(hardwareMap, telemetry);
        intake = new IntakeSubsystem(hardwareMap, telemetry);
        shooter = new ShooterSubsystem(hardwareMap, telemetry);
        spindexer = new OldSpindexerSubsystem(hardwareMap, telemetry);
        aprilTag = new AprilTagNavigator(drive, hardwareMap, telemetry);
        obeliskDetector = new ObeliskMotifDetector(aprilTag, telemetry);
        funnel = new FunnelSubsystem(hardwareMap, telemetry);

        // Initialize color sensor (optional)
        try {
            colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
            colorSensorSubsystem = new ColorSensorSubsystem(colorSensor, null, telemetry, testMotif);
            autoOuttake = new AutoOuttakeController(colorSensorSubsystem, testMotif, spindexer, shooter, funnel, telemetry);
            autoOuttake.setScoreThreshold(6);
            autoOuttake.setTargetShooterRPM(SHOOTER_TARGET_RPM);
        } catch (Exception e) {
            colorSensorSubsystem = null;
            autoOuttake = null;
            telemetry.addData("ColorSensor", "Not found - auto outtake disabled");
        }

        // Initialize spindexer
        spindexer.setIntakeMode(false); // Start in outtake mode
        spindexer.goToPosition(0);

        telemetry.addData("Status", "Initialized - Ready to test");
        telemetry.addData("Current Phase", currentPhase.toString());
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Update subsystems
            spindexer.update();
            shooter.updateVoltageCompensation();
            aprilTag.updateRobotPositionFromTriangulation();
            obeliskDetector.update();

            if (colorSensorSubsystem != null) {
                colorSensorSubsystem.update();
            }
            if (autoOuttake != null) {
                autoOuttake.update();
            }

            // Handle test phase navigation
            handlePhaseNavigation();

            // Handle manual controls
            handleManualControls();

            // Run current test phase
            runCurrentTestPhase();

            // CRITICAL: Stop shooter when spindexer is moving
            enforceShooterSpindexerSafety();

            // Update telemetry
            updateTelemetry();
        }

        // Cleanup
        stopAllSystems();
        aprilTag.closeVision();
    }

    private void handlePhaseNavigation() {
        boolean a = gamepad2.a;
        boolean b = gamepad2.b;

        if (a && !aPressedLast) {
            // Next phase
            phaseIndex = (phaseIndex + 1) % TestPhase.values().length;
            currentPhase = TestPhase.values()[phaseIndex];
            testRunning = false;
            phaseTimer.reset();
        }
        if (b && !bPressedLast) {
            // Previous phase
            phaseIndex = (phaseIndex - 1 + TestPhase.values().length) % TestPhase.values().length;
            currentPhase = TestPhase.values()[phaseIndex];
            testRunning = false;
            phaseTimer.reset();
        }

        aPressedLast = a;
        bPressedLast = b;
    }

    private void handleManualControls() {
        boolean y = gamepad2.y;
        if (y && !yPressedLast) {
            stopAllSystems();
            testRunning = false;
        }
        yPressedLast = y;

        // Manual spindexer control
        boolean dpadUp = gamepad2.dpad_up;
        boolean dpadDown = gamepad2.dpad_down;
        if (dpadUp && !dpadUpLast) {
            spindexerTestPosition = (spindexerTestPosition + 1) % 3;
            spindexer.goToPosition(spindexerTestPosition);
        }
        if (dpadDown && !dpadDownLast) {
            spindexerTestPosition = (spindexerTestPosition - 1 + 3) % 3;
            spindexer.goToPosition(spindexerTestPosition);
        }
        dpadUpLast = dpadUp;
        dpadDownLast = dpadDown;

        // Manual intake control
        if (gamepad2.left_trigger > 0.1) {
            intake.setPower(1.0);
        } else if (gamepad2.left_bumper) {
            intake.setPower(-1.0);
        } else {
            intake.setPower(0);
        }

        // Manual shooter control
        if (gamepad2.right_bumper) {
            shooter.setTargetRPM(SHOOTER_TARGET_RPM);
        } else if (gamepad2.right_trigger > 0.1) {
            shooter.stop();
        }
    }

    private void runCurrentTestPhase() {
        boolean x = gamepad2.x;
        if (x && !xPressedLast && !testRunning) {
            testRunning = true;
            testTimer.reset();
        }
        xPressedLast = x;

        if (!testRunning) return;

        switch (currentPhase) {
            case APRILTAG_DETECTION:
                testAprilTagDetection();
                break;
            case OBELISK_MOTIF:
                testObeliskMotif();
                break;
            case COLOR_SENSOR:
                testColorSensor();
                break;
            case DRIVE_TEST:
                testDriveSystem();
                break;
            case INTAKE_TEST:
                testIntakeSystem();
                break;
            case SPINDEXER_POSITIONS:
                testSpindexerPositions();
                break;
            case SHOOTER_RPM:
                testShooterRPM();
                break;
            case AUTO_OUTTAKE:
                testAutoOuttake();
                break;
            case INTEGRATED_SHOOTING:
                testIntegratedShooting();
                break;
        }
    }

    private void testAprilTagDetection() {
        aprilTag.updateRobotPositionFromAllianceGoals();
        AprilTagDetection tag = aprilTag.getBestAllianceGoalDetection();

        if (tag != null) {
            telemetry.addData("TEST: AprilTag", "DETECTED - ID: %d, Range: %.1f\"", tag.id, tag.ftcPose.range);
            telemetry.addData("  X Offset", "%.2f\"", tag.ftcPose.x);
            telemetry.addData("  Y Distance", "%.2f\"", tag.ftcPose.y);
            telemetry.addData("  Yaw", "%.2f°", tag.ftcPose.yaw);
        } else {
            telemetry.addData("TEST: AprilTag", "NO DETECTION");
        }
    }

    private void testObeliskMotif() {
        ArtifactColor[] motif = obeliskDetector.getMotif();
        if (motif != null) {
            telemetry.addData("TEST: Obelisk Motif", "DETECTED: %s%s%s",
                motif[0].getCharacter(), motif[1].getCharacter(), motif[2].getCharacter());
            telemetry.addData("  Tag ID", obeliskDetector.getMotifTagId());
        } else {
            telemetry.addData("TEST: Obelisk Motif", "NOT DETECTED");
        }
    }

    private void testColorSensor() {
        if (colorSensorSubsystem == null) {
            telemetry.addData("TEST: Color Sensor", "NOT AVAILABLE");
            return;
        }

        ArtifactColor[] ramp = colorSensorSubsystem.getDetectedRamp();
        int score = colorSensorSubsystem.getPatternScore();
        telemetry.addData("TEST: Color Sensor", "Score: %d/9", score);
        telemetry.addData("  Detected Ramp", PatternScorer.patternString(ramp));
    }

    private void testDriveSystem() {
        float forward = gamepad1.left_stick_y;
        float strafe = -gamepad1.left_stick_x;
        float turn = -gamepad1.right_stick_x;

        double denominator = JavaUtil.maxOfList(JavaUtil.createListWith(0.5, 0.5 + Math.abs(turn)));
        drive.drive(forward / denominator, strafe / denominator, turn / denominator);

        telemetry.addData("TEST: Drive System", "ACTIVE");
        telemetry.addData("  Forward", "%.2f", forward);
        telemetry.addData("  Strafe", "%.2f", strafe);
        telemetry.addData("  Turn", "%.2f", turn);
    }

    private void testIntakeSystem() {
        if (testTimer.seconds() < 2.0) {
            intake.setPower(1.0);
            telemetry.addData("TEST: Intake", "RUNNING FORWARD");
        } else if (testTimer.seconds() < 4.0) {
            intake.setPower(-1.0);
            telemetry.addData("TEST: Intake", "RUNNING REVERSE");
        } else {
            intake.setPower(0);
            telemetry.addData("TEST: Intake", "STOPPED");
            testRunning = false;
        }
    }

    private void testSpindexerPositions() {
        if (testTimer.seconds() < 1.0) {
            // Test position 0
            spindexer.goToPosition(0);
            telemetry.addData("TEST: Spindexer", "Moving to Position 0");
        } else if (testTimer.seconds() < 2.0) {
            if (spindexer.isAtPosition()) {
                spindexer.goToPosition(1);
                telemetry.addData("TEST: Spindexer", "Moving to Position 1");
            }
        } else if (testTimer.seconds() < 3.0) {
            if (spindexer.isAtPosition()) {
                spindexer.goToPosition(2);
                telemetry.addData("TEST: Spindexer", "Moving to Position 2");
            }
        } else if (testTimer.seconds() < 4.0) {
            if (spindexer.isAtPosition()) {
                spindexer.setIntakeMode(true);
                spindexer.goToPosition(0);
                telemetry.addData("TEST: Spindexer", "Switched to INTAKE mode, Position 0");
            }
        } else {
            telemetry.addData("TEST: Spindexer", "COMPLETE");
            testRunning = false;
        }
    }

    private void testShooterRPM() {
        if (testTimer.seconds() < 1.0) {
            shooter.setTargetRPM(SHOOTER_TARGET_RPM);
            telemetry.addData("TEST: Shooter", "Spinning up to %.0f RPM", SHOOTER_TARGET_RPM);
        } else {
            telemetry.addData("TEST: Shooter", "Current: %.0f / Target: %.0f",
                shooter.getCurrentRPM(), shooter.getTargetRPM());
            telemetry.addData("  At Target", shooter.isAtTargetRPM() ? "YES" : "NO");
            if (testTimer.seconds() > 5.0) {
                shooter.stop();
                testRunning = false;
            }
        }
    }

    private void testAutoOuttake() {
        if (autoOuttake == null) {
            telemetry.addData("TEST: Auto Outtake", "NOT AVAILABLE");
            testRunning = false;
            return;
        }

        telemetry.addData("TEST: Auto Outtake", "Monitoring color sensor...");
        telemetry.addData("  State", autoOuttake.getState().toString());
    }

    private void testIntegratedShooting() {
        // Test integrated shooting sequence with safety enforcement
        if (testTimer.seconds() < 1.0) {
            // Step 1: Spin up shooter
            shooter.setTargetRPM(SHOOTER_TARGET_RPM);
            telemetry.addData("TEST: Integrated Shooting", "Step 1: Spinning up shooter");
        } else if (testTimer.seconds() < 3.0) {
            // Step 2: Wait for shooter to be ready
            if (shooter.isAtTargetRPM()) {
                // Step 3: Move spindexer (shooter should stop automatically)
                if (!spindexer.isMoving()) {
                    spindexer.goToPosition(1);
                    telemetry.addData("TEST: Integrated Shooting", "Step 2: Moving spindexer (shooter should stop)");
                }
            }
        } else if (testTimer.seconds() < 5.0) {
            // Step 4: Wait for spindexer to finish
            if (spindexer.isAtPosition() && !spindexer.isMoving()) {
                // Step 5: Restart shooter
                shooter.setTargetRPM(SHOOTER_TARGET_RPM);
                telemetry.addData("TEST: Integrated Shooting", "Step 3: Spindexer done, restarting shooter");
            }
        } else {
            shooter.stop();
            telemetry.addData("TEST: Integrated Shooting", "COMPLETE");
            testRunning = false;
        }
    }

    /**
     * CRITICAL SAFETY: Stop shooter when spindexer is moving
     * This prevents balls from popping off during spindexer movement
     */
    private void enforceShooterSpindexerSafety() {
        if (spindexer.isMoving() || !spindexer.isAtPosition()) {
            // Spindexer is moving - stop shooter
            if (shooter.getTargetRPM() > 0) {
                shooterWasRunning = true;
                shooter.stop();
            }
        } else if (spindexer.isAtPosition() && shooterWasRunning) {
            // Spindexer stopped - can restart shooter if needed
            // Don't auto-restart, let test logic handle it
            shooterWasRunning = false;
        }
    }

    private void stopAllSystems() {
        drive.stop();
        intake.stop();
        shooter.stop();
        spindexer.reset();
    }

    private void updateTelemetry() {
        telemetry.addLine("=== COMPREHENSIVE SUBSYSTEM TEST ===");
        telemetry.addData("Current Phase", "%d/%d: %s", phaseIndex + 1, TestPhase.values().length, currentPhase.toString());
        telemetry.addData("Test Running", testRunning ? "YES" : "NO");
        telemetry.addLine();

        // Subsystem status
        telemetry.addData("Spindexer", "Pos: %d, Moving: %s, At Pos: %s",
            spindexerTestPosition, spindexer.isMoving(), spindexer.isAtPosition());
        telemetry.addData("Shooter", "%.0f / %.0f RPM, At Target: %s",
            shooter.getCurrentRPM(), shooter.getTargetRPM(), shooter.isAtTargetRPM() ? "YES" : "NO");
        telemetry.addData("Safety", "Shooter stopped during spindexer movement: %s",
            (spindexer.isMoving() && shooter.getTargetRPM() == 0) ? "YES ✓" : "NO ✗");

        telemetry.addLine();
        telemetry.addData("Controls", "A: Next Phase, B: Prev Phase, X: Run Test, Y: Stop All");
        telemetry.addData("  D-Pad Up/Down", "Manual spindexer control");
        telemetry.addData("  Triggers/Bumpers", "Manual intake/shooter control");

        aprilTag.updateDECODELocalizationTelemetry();
        telemetry.update();
    }
}

