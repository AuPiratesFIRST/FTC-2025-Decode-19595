package org.firstinspires.ftc.teamcode.SubSystems.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

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

/**
 * RED ALLIANCE TEST VERSION - FIXED
 */
@TeleOp(name = "Red Alliance TeleOp TEST", group = "TeleOp")
public class RedAllianceTeleOpTest extends LinearOpMode {

    // === AUTO-ALIGNMENT CONSTANTS ===
    private static final double DESIRED_SHOOTING_DISTANCE = 134;
    private static final double DESIRED_SHOOTING_Angle = 21;
    private static final double KP_STRAFE = 0.03;
    private static final double KP_FORWARD = 0.03;
    private static final double KP_ROT = 0.015;

    private static final double MAX_AUTO_POWER = 0.40;
    private static final double POSITION_DEADBAND = 0.75;
    private static final double ANGLE_DEADBAND_DEG = 1.5;

    // Subsystems
    private DriveSubsystem drive;
    private IntakeSubsystem intake;
    private ShooterSubsystem shooter;
    private OldSpindexerSubsystem spindexer;
    private AprilTagNavigator aprilTag;
    private ColorSensorSubsystem colorSensorSubsystem;
    private ObeliskMotifDetector obeliskDetector;
    private NormalizedColorSensor colorSensor;
    private AutoOuttakeController autoOuttake;

    // State Variables
    private int spindexerPositionIndex = 0;
    private boolean intakeMode = true;
    private boolean spindexerIsMoving = false;
    private boolean shooterNeedsToSpinUp = true;
    private boolean ballSettling = false;
    private boolean manualControlMode = false;
    private boolean shooterManuallyControlled = false;
    private int shotNumber = 0;
    private double driveSpeed = 1.0;

    // Timers
    private ElapsedTime actionTimer; // Used for shooter stability
    private ElapsedTime shotTimer;   // Used for shot-to-shot delays

    // Button State Tracking
    private boolean spindexerPressLast = false;
    private boolean dpadDownLast = false;
    private boolean rbPressedLast = false;
    private boolean yPressedLast = false;
    private boolean xPressedLast = false;
    private boolean dpadLeftLast = false;
    private boolean dpadRightLast = false;
    private boolean aPressedLast = false;
    private boolean bPressedLast = false;

    private static final double SHOOTER_TARGET_RPM = 5210.0;
    private static final int TARGET_TAG_ID = 24;

    @Override
    public void runOpMode() {
        // Initialize subsystems
        drive = new DriveSubsystem(hardwareMap, telemetry);
        intake = new IntakeSubsystem(hardwareMap, telemetry);
        shooter = new ShooterSubsystem(hardwareMap, telemetry);
        spindexer = new OldSpindexerSubsystem(hardwareMap, telemetry);
        aprilTag = new AprilTagNavigator(drive, hardwareMap, telemetry);
        
        // Initialize Timers - FIXED: Both timers now initialized
        shotTimer = new ElapsedTime();
        actionTimer = new ElapsedTime();

        ArtifactColor[] motif = new ArtifactColor[] {
                ArtifactColor.PURPLE, ArtifactColor.GREEN, ArtifactColor.PURPLE
        };

        try {
            NormalizedColorSensor normColor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
            colorSensorSubsystem = new ColorSensorSubsystem(normColor, null, telemetry, motif);
            colorSensor = normColor;
        } catch (Exception e) {
            colorSensorSubsystem = null;
            colorSensor = null;
            telemetry.addData("ColorSensor", "sensor_color not found - auto outtake disabled");
        }

        obeliskDetector = new ObeliskMotifDetector(aprilTag, telemetry);

        telemetry.addData("Status", "Initialized - RED ALLIANCE TEST");
        telemetry.update();

        // Scan during init
        long initStart = System.currentTimeMillis();
        while (!isStarted() && !isStopRequested() && System.currentTimeMillis() - initStart < 3000) {
            obeliskDetector.update();
            telemetry.update();
            sleep(50);
        }

        waitForStart();

        // Final motif check
        motif = obeliskDetector.getMotif();
        if (motif == null) {
            motif = new ArtifactColor[] { ArtifactColor.GREEN, ArtifactColor.PURPLE, ArtifactColor.GREEN };
        }

        if (colorSensorSubsystem != null) {
            autoOuttake = new AutoOuttakeController(colorSensorSubsystem, motif, spindexer, shooter, telemetry);
            autoOuttake.setScoreThreshold(6);
            autoOuttake.setTargetShooterRPM(SHOOTER_TARGET_RPM);
        }

        while (opModeIsActive()) {
            handleDriving();
            handleIntake();
            handleShooter();
            shooter.updateVoltageCompensation();
            obeliskDetector.update();
            aprilTag.initializeCameraControls();

            if (autoOuttake != null) autoOuttake.update();

            handleSpindexer();
            enforceShooterSpindexerSafety();
            handleAprilTagAlignment();
            updateTelemetry();
        }

        drive.stop();
        intake.stop();
        shooter.stop();
        aprilTag.closeVision();
    }

    private void handleDriving() {
        float forward = gamepad1.left_stick_y + gamepad1.right_stick_y;
        float strafe = -gamepad1.left_stick_x;
        float turn = -gamepad1.right_stick_x;

        double denominator = JavaUtil.maxOfList(JavaUtil.createListWith(0.5, 0.5 + Math.abs(turn)));

        if (gamepad1.a && !aPressedLast) driveSpeed = 1.0;
        if (gamepad1.b && !bPressedLast) driveSpeed = 0.5;
        aPressedLast = gamepad1.a;
        bPressedLast = gamepad1.b;

        // Maintain heading if Y is held (no turn)
        float currentTurn = gamepad2.y ? 0 : turn;

        drive.drive(
                (forward / denominator) * driveSpeed,
                (strafe / denominator) * driveSpeed,
                (currentTurn / denominator) * driveSpeed);
    }

    private void handleIntake() {
        if (gamepad2.left_trigger > 0.1) intake.setPower(1.0);
        else if (gamepad2.left_bumper) intake.setPower(-1.0);
        else intake.setPower(0);
    }

    private void handleShooter() {
        boolean rb = gamepad2.right_bumper;
        if (rb && !rbPressedLast) {
            shooterManuallyControlled = true;
            shooterNeedsToSpinUp = false;
            if (shooter.getTargetRPM() > 0) {
                shooter.stop();
                shooterManuallyControlled = false;
            } else {
                shooter.setTargetRPM(SHOOTER_TARGET_RPM);
            }
        }
        rbPressedLast = rb;
    }

    private void handleSpindexer() {
        if (gamepad2.x && !xPressedLast) {
            manualControlMode = !manualControlMode;
            if (manualControlMode) {
                spindexerIsMoving = false;
                ballSettling = false;
            }
        }
        xPressedLast = gamepad2.x;

        if (gamepad2.dpad_down && !dpadDownLast && !manualControlMode) {
            intakeMode = !intakeMode;
            spindexerPositionIndex = 0;
            spindexer.setIntakeMode(intakeMode);
        }
        dpadDownLast = gamepad2.dpad_down;

        if (gamepad2.b && !bPressedLast && !manualControlMode) {
            ArtifactColor[] currentMotif = obeliskDetector.getMotif();
            if (currentMotif == null) {
                currentMotif = new ArtifactColor[] { ArtifactColor.GREEN, ArtifactColor.PURPLE, ArtifactColor.GREEN };
            }
            spindexer.setIntakeMode(false);
            spindexer.rotateToMotifStartPosition(currentMotif);
            spindexerIsMoving = true;
            spindexerPositionIndex = OldSpindexerSubsystem.getStartingPositionFromMotif(currentMotif);
        }
        bPressedLast = gamepad2.b;

        if (manualControlMode) {
            handleManualSpindexer();
        } else {
            handleAutomatedSpindexer();
        }
    }

    private void handleManualSpindexer() {
        float manualPower = -gamepad2.right_stick_y;
        if (Math.abs(manualPower) > 0.1) {
            spindexer.setManualPower(manualPower * OldSpindexerSubsystem.getRecommendedManualPowerMultiplier());
            spindexerIsMoving = false;
        } else {
            spindexer.setManualPower(0);
        }

        if (gamepad2.dpad_left && !dpadLeftLast && !spindexerIsMoving) {
            spindexerPositionIndex = (spindexerPositionIndex - 1 + 3) % 3;
            spindexer.goToPosition(spindexerPositionIndex);
            spindexerIsMoving = true;
        }
        if (gamepad2.dpad_right && !dpadRightLast && !spindexerIsMoving) {
            spindexerPositionIndex = (spindexerPositionIndex + 1) % 3;
            spindexer.goToPosition(spindexerPositionIndex);
            spindexerIsMoving = true;
        }
        dpadLeftLast = gamepad2.dpad_left;
        dpadRightLast = gamepad2.dpad_right;

        if (spindexerIsMoving) {
            spindexer.update();
            if (spindexer.isAtPosition()) spindexerIsMoving = false;
        }
    }

    private void handleAutomatedSpindexer() {
        spindexer.update();

        if (spindexerIsMoving && spindexer.isAtPosition()) {
            spindexerIsMoving = false;
            if (!intakeMode && !shooterManuallyControlled) {
                shooter.setTargetRPM(SHOOTER_TARGET_RPM);
                shooterNeedsToSpinUp = true;
                actionTimer.reset();
                shotTimer.reset();
            }
        }

        if (shooterNeedsToSpinUp && !shooterManuallyControlled) {
            if (shooter.isAtTargetRPM()) {
                long stabilityDelay = (shotNumber == 0) ? 300 : 200;
                if (actionTimer.milliseconds() > stabilityDelay) {
                    shooterNeedsToSpinUp = false;
                }
            } else {
                actionTimer.reset();
            }
        }

        boolean spPress = gamepad2.a;
        long minDelayBetweenShots = (!intakeMode && spindexerPositionIndex == 0) ? 400 : 250;
        
        boolean canMove = !spindexerIsMoving &&
                (!shooterNeedsToSpinUp || shooter.isAtTargetRPM() || shooterManuallyControlled) &&
                (shotTimer.milliseconds() > minDelayBetweenShots || intakeMode);

        if (spPress && !spindexerPressLast && canMove) {
            if (!intakeMode) shotNumber = spindexerPositionIndex;
            spindexerPositionIndex = (spindexerPositionIndex + 1) % 3;
            spindexer.goToPositionForCurrentMode(spindexerPositionIndex);
            spindexerIsMoving = true;
            actionTimer.reset();
            shotTimer.reset();
        }
        spindexerPressLast = spPress;
    }

    private void enforceShooterSpindexerSafety() {
        if (spindexer.isMoving() || !spindexer.isAtPosition()) {
            if (shooter.getTargetRPM() > 0 && !shooterManuallyControlled) {
                shooter.stop();
                shooterNeedsToSpinUp = true;
            }
        }
    }

    private void handleAprilTagAlignment() {
        if (gamepad2.y) {
            aprilTag.updateRobotPositionFromAllianceGoals();
            AprilTagDetection tag = aprilTag.getBestAllianceGoalDetection();

            if (tag == null || tag.id != TARGET_TAG_ID) {
                drive.drive(0, 0, 0);
                return;
            }

            if (!shooterManuallyControlled) shooter.setTargetRPM(SHOOTER_TARGET_RPM);

            double[] corrections = aprilTag.calculateAlignmentCorrections(
                    tag, DESIRED_SHOOTING_DISTANCE, DESIRED_SHOOTING_Angle,
                    POSITION_DEADBAND, POSITION_DEADBAND, ANGLE_DEADBAND_DEG,
                    KP_STRAFE, KP_FORWARD, KP_ROT, MAX_AUTO_POWER);

            if (corrections != null) {
                drive.drive(corrections[1], corrections[0], corrections[2]);
            }
        }
    }

    private void updateTelemetry() {
        telemetry.addData("Spindexer", manualControlMode ? "MANUAL" : "AUTO");
        telemetry.addData("Mode", intakeMode ? "INTAKE" : "OUTTAKE");
        telemetry.addData("RPM", "%.0f / %.0f", shooter.getCurrentRPM(), shooter.getTargetRPM());
        telemetry.update();
    }
}
