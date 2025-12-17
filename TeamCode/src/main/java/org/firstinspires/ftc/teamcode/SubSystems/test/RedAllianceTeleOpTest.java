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
 * RED ALLIANCE TEST VERSION
 */
@TeleOp(name = "Red Alliance TeleOp TEST", group = "TeleOp")
public class RedAllianceTeleOpTest extends LinearOpMode {

    // === AUTO-ALIGNMENT CONSTANTS ADDED HERE ===
    private static final double DESIRED_SHOOTING_DISTANCE = 134;
    private static final double DESIRED_SHOOTING_Angle = 21;
    private static final double KP_STRAFE = 0.03;
    private static final double KP_FORWARD = 0.03;
    private static final double KP_ROT = 0.015;

    private static final double MAX_AUTO_POWER = 0.40;
    private static final double POSITION_DEADBAND = 0.75;
    private static final double ANGLE_DEADBAND_DEG = 1.5;

    private DriveSubsystem drive;
    private IntakeSubsystem intake;
    private ShooterSubsystem shooter;
    private OldSpindexerSubsystem spindexer;
    private AprilTagNavigator aprilTag;

    private ColorSensorSubsystem colorSensorSubsystem;

    // New: vision -> obelisk motif + auto outtake controller
    private ObeliskMotifDetector obeliskDetector;
    private NormalizedColorSensor colorSensor;
    private AutoOuttakeController autoOuttake;

    private int spindexerPositionIndex = 0;
    private boolean intakeMode = true;
    private boolean spindexerIsMoving = false;
    private boolean shooterNeedsToSpinUp = true;
    private boolean ballSettling = false;
    private boolean manualControlMode = false;

    private boolean shooterManuallyControlled = false;

    // Shooting timing - track shot number for better timing
    private int shotNumber = 0;
    private ElapsedTime actionTimer;

    private ElapsedTime shotTimer;

    // === BUTTON STATE TRACKING (DEBOUNCING) ===
    private boolean spindexerPressLast = false;
    private boolean dpadDownLast = false;
    private boolean rbPressedLast = false;
    private boolean yPressedLast = false;
    private boolean xPressedLast = false;

    // === NEWLY ADDED DEBOUNCE VARIABLES ===
    private boolean aPressedLast2 = false;
    private boolean dpadLeftLast = false;
    private boolean dpadRightLast = false;

    private double driveSpeed = 1.0;
    private boolean aPressedLast = false;
    private boolean bPressedLast = false;
    private boolean colorSensorResetPressedLast = false;

    // Positions are now managed by OldSpindexerSubsystem
    // Use spindexer.getIntakePositionTicks(index) or
    // spindexer.getOuttakePositionTicks(index)
    // Or use spindexer.goToPositionForCurrentMode(index) for automatic mode-based
    // positioning

    private static final int BALL_SETTLING_TICKS = 34;

    private static final double SHOOTER_TARGET_RPM = 5210.0;

    private static final boolean IS_BLUE_ALLIANCE = false;
    private static final int TARGET_TAG_ID = 24;

    @Override
    public void runOpMode() {
        // initialize subsystems
        drive = new DriveSubsystem(hardwareMap, telemetry);
        intake = new IntakeSubsystem(hardwareMap, telemetry);
        shooter = new ShooterSubsystem(hardwareMap, telemetry);
        spindexer = new OldSpindexerSubsystem(hardwareMap, telemetry);
        aprilTag = new AprilTagNavigator(drive, hardwareMap, telemetry);
        // Color sensor and motif detection are optional; guard against missing hardware
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

        // Initialize timers
        shotTimer = new ElapsedTime();

        telemetry.addData("Status", "Initialized - RED ALLIANCE TEST");
        telemetry.update();

        telemetry.addData("Obelisk", "Scanning during init...");
        telemetry.update();

        long initStart = System.currentTimeMillis();
        while (!isStarted() && !isStopRequested() && System.currentTimeMillis() - initStart < 3000) {
            obeliskDetector.update();
            telemetry.update();
            sleep(50);
        }

        waitForStart();

        motif = obeliskDetector.getMotif();
        if (motif == null) {
            motif = new ArtifactColor[] { ArtifactColor.GREEN, ArtifactColor.PURPLE, ArtifactColor.GREEN };
            telemetry.addData("Obelisk", "No motif read - using fallback GPG");
        } else {
            telemetry.addData("Obelisk", "Using motif: " +
                    motif[0].getCharacter() + motif[1].getCharacter() + motif[2].getCharacter());
        }
        telemetry.update();

        if (colorSensorSubsystem != null) {
            autoOuttake = new AutoOuttakeController(colorSensorSubsystem, motif, spindexer, shooter, telemetry);
            autoOuttake.setScoreThreshold(6);
            autoOuttake.setTargetShooterRPM(SHOOTER_TARGET_RPM);
        } else {
            autoOuttake = null; // gracefully disable auto-outtake if sensor missing
        }

        while (opModeIsActive()) {
            handleDriving();
            handleIntake();
            handleShooter();
            shooter.updateVoltageCompensation();

            obeliskDetector.update();

            // Initialize camera controls when ready (non-blocking)
            aprilTag.initializeCameraControls();

            if (autoOuttake != null)
                autoOuttake.update();

            handleSpindexer();

            handleAprilTagAlignment();

            updateTelemetry();
        }

        drive.stop();
        intake.stop();
        shooter.stop();
        aprilTag.closeVision();
    }

    private void handleDriving() {
        float forward = gamepad1.left_stick_y;
        float strafe = -gamepad1.left_stick_x;
        float turn = -gamepad1.right_stick_x;

        double denominator = JavaUtil.maxOfList(JavaUtil.createListWith(0.5, 0.5 + Math.abs(turn)));

        boolean a = gamepad1.a;
        boolean b = gamepad1.b;
        if (a && !aPressedLast)
            driveSpeed = 1.0;
        if (b && !bPressedLast)
            driveSpeed = 0.5;
        aPressedLast = a;
        bPressedLast = b;

        if (!gamepad2.y) {
            drive.drive(
                    (forward / denominator) * driveSpeed,
                    (strafe / denominator) * driveSpeed,
                    (turn / denominator) * driveSpeed);
        } else {
            drive.drive(
                    (forward / denominator) * driveSpeed,
                    (strafe / denominator) * driveSpeed,
                    0);
        }
    }

    private void handleIntake() {
        if (gamepad2.left_trigger > 0.1)
            intake.setPower(1.0);
        else if (gamepad2.left_bumper)
            intake.setPower(-1.0);
        else
            intake.setPower(0);
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
        boolean x = gamepad2.x;
        if (x && !xPressedLast) {
            manualControlMode = !manualControlMode;
            if (manualControlMode) {
                spindexerIsMoving = false;
                ballSettling = false;
            }
        }
        xPressedLast = x;

        boolean dpadDown = gamepad2.dpad_down;
        if (dpadDown && !dpadDownLast && !manualControlMode) {
            intakeMode = !intakeMode;
            spindexerPositionIndex = 0;
            spindexer.setIntakeMode(intakeMode);
        }
        dpadDownLast = dpadDown;

        // Button B: Reset spindexer to motif start position (for shooting)
        boolean b = gamepad2.b;
        if (b && !bPressedLast && !manualControlMode) {
            // Get current motif from obelisk detector or use fallback
            ArtifactColor[] currentMotif = null;
            if (obeliskDetector != null) {
                currentMotif = obeliskDetector.getMotif();
            }
            if (currentMotif == null) {
                // Fallback motif
                currentMotif = new ArtifactColor[] {
                        ArtifactColor.GREEN, ArtifactColor.PURPLE, ArtifactColor.GREEN
                };
            }

            // Switch to outtake mode and rotate to starting position
            spindexer.setIntakeMode(false);
            spindexer.rotateToMotifStartPosition(currentMotif);
            spindexerIsMoving = true;
            spindexerPositionIndex = OldSpindexerSubsystem.getStartingPositionFromMotif(currentMotif);

            telemetry.addData("Spindexer", "Rotated to motif start position");
        }
        bPressedLast = b;

        if (manualControlMode) {
            handleManualSpindexer();
            return;
        }

        handleAutomatedSpindexer();
    }

    private void handleManualSpindexer() {
        boolean dpadLeft = gamepad2.dpad_left;
        boolean dpadRight = gamepad2.dpad_right;

        float manualPower = -gamepad2.right_stick_y;
        if (Math.abs(manualPower) > 0.1) {
            // Use recommended multiplier for better torque (312 RPM motor needs more power)
            // Increased from 0.5 to 0.75 for better torque to push past bar
            double powerMultiplier = OldSpindexerSubsystem.getRecommendedManualPowerMultiplier();
            spindexer.setManualPower(manualPower * powerMultiplier);
            spindexerIsMoving = false;
            return;
        } else {
            spindexer.setManualPower(0);
        }

        if (dpadLeft && !dpadLeftLast && !spindexerIsMoving) {
            spindexerPositionIndex = (spindexerPositionIndex - 1 + 3) % 3;
            spindexer.goToPosition(spindexerPositionIndex);
            spindexerIsMoving = true;
        }
        if (dpadRight && !dpadRightLast && !spindexerIsMoving) {
            spindexerPositionIndex = (spindexerPositionIndex + 1) % 3;
            spindexer.goToPosition(spindexerPositionIndex);
            spindexerIsMoving = true;
        }

        dpadLeftLast = dpadLeft;
        dpadRightLast = dpadRight;

        if (spindexerIsMoving) {
            spindexer.update();
            if (spindexer.isAtPosition())
                spindexerIsMoving = false;
        }
    }

    private int getCurrentModeTargetTicks(int index) {
        // Use subsystem method to get position based on current mode
        return spindexer.getPositionTicksForCurrentMode(index);
    }

    private void handleAutomatedSpindexer() {
        spindexer.update();

        if (spindexerIsMoving) {
            if (spindexer.isAtPosition()) {
                spindexerIsMoving = false;

                if (intakeMode) {
                    ballSettling = true;
                    ballSettling = false;
                } else {
                    if (!shooterManuallyControlled) {
                        shooter.setTargetRPM(SHOOTER_TARGET_RPM);
                        shooterNeedsToSpinUp = true;
                        actionTimer.reset(); // Reset timer for shooter spin-up
                        shotTimer.reset(); // Reset shot timer
                    }
                }
            }
        }

        if (ballSettling)
            ballSettling = false;

        if (shooterNeedsToSpinUp && !shooterManuallyControlled) {
            // Wait for shooter to be stable (at target RPM for at least 300ms for first
            // shot, 200ms for others)
            // This prevents first shot overshooting due to RPM instability
            if (shooter.isAtTargetRPM()) {
                long stabilityDelay = (shotNumber == 0) ? 300 : 200; // Longer delay for first shot
                if (actionTimer.milliseconds() > stabilityDelay) {
                    shooterNeedsToSpinUp = false;
                }
            } else {
                actionTimer.reset(); // Reset if RPM drops
            }
        }

        boolean spPress = gamepad2.a;

        // Add minimum delay between shots to prevent rapid firing
        // This helps with accuracy - first shot needs more time, subsequent shots can
        // be faster
        long minDelayBetweenShots = (!intakeMode && spindexerPositionIndex == 0) ? 400 : 250; // 400ms for first shot,
                                                                                              // 250ms for others
        boolean canMove = !spindexerIsMoving &&
                !ballSettling &&
                (!shooterNeedsToSpinUp || shooter.isAtTargetRPM() || shooterManuallyControlled) &&
                (shotTimer.milliseconds() > minDelayBetweenShots || intakeMode); // Enforce delay between shots

        if (spPress && !spindexerPressLast && canMove) {
            // Track shot number for timing adjustments
            if (!intakeMode) {
                shotNumber = spindexerPositionIndex;
            }

            spindexerPositionIndex = (spindexerPositionIndex + 1) % 3;
            // Use subsystem method that automatically selects position based on mode
            spindexer.goToPositionForCurrentMode(spindexerPositionIndex);
            spindexerIsMoving = true;

            // Reset timers for next shot
            actionTimer.reset();
            shotTimer.reset();
        }
        spindexerPressLast = spPress;
    }

    private void handleAprilTagAlignment() {
        if (gamepad2.y) {
            aprilTag.updateRobotPositionFromAllianceGoals();
            AprilTagDetection tag = aprilTag.getBestAllianceGoalDetection();

            if (tag == null || tag.id != TARGET_TAG_ID) {
                telemetry.addData("ALIGN", "Tag NOT found");
                drive.drive(0, 0, 0);
                return;
            }

            if (!shooterManuallyControlled) {
                shooter.setTargetRPM(SHOOTER_TARGET_RPM);
            }

            // Use improved alignment method from AprilTagNavigator
            double[] corrections = aprilTag.calculateAlignmentCorrections(
                    tag, DESIRED_SHOOTING_DISTANCE, DESIRED_SHOOTING_Angle,
                    POSITION_DEADBAND, POSITION_DEADBAND, ANGLE_DEADBAND_DEG,
                    KP_STRAFE, KP_FORWARD, KP_ROT, MAX_AUTO_POWER);

            if (corrections == null) {
                telemetry.addData("ALIGN", "Invalid tag");
                drive.drive(0, 0, 0);
                return;
            }

            double strafePower = corrections[0];
            double forwardPower = corrections[1];
            double turnPower = corrections[2];
            boolean aligned = (corrections[3] == 1.0);

            drive.drive(forwardPower, strafePower, turnPower);

            if (aligned) {
                telemetry.addData("ALIGN", "LOCKED ON");
            } else if (Math.abs(turnPower) > 0.01) {
                telemetry.addData("ALIGN", "STAGE 1: CORRECTING ANGLE...");
                telemetry.addData("  Yaw Error", "%.2f°", tag.ftcPose.yaw + DESIRED_SHOOTING_Angle);
            } else {
                telemetry.addData("ALIGN", "STAGE 2: CORRECTING POSITION...");
                telemetry.addData("  X Offset", "%.2f in", tag.ftcPose.x);
                telemetry.addData("  Y Dist", "%.2f in (Target %.1f)", tag.ftcPose.y, DESIRED_SHOOTING_DISTANCE);
            }
        }
    }

    private void updateTelemetry() {
        telemetry.addData("=== RED ALLIANCE TELEOP TEST ===", "");
        telemetry.addData("Drive Speed", driveSpeed);
        telemetry.addData("Spindexer Control",
                manualControlMode ? "MANUAL" : "AUTOMATED");
        telemetry.addData("Spindexer Mode",
                intakeMode ? "INTAKE" : "OUTTAKE");
        telemetry.addData("Spindexer Position", spindexerPositionIndex);
        telemetry.addData("Shooter RPM",
                "%.0f / %.0f", shooter.getCurrentRPM(), shooter.getTargetRPM());
        aprilTag.updateDECODELocalizationTelemetry();
        telemetry.update();
    }
}
