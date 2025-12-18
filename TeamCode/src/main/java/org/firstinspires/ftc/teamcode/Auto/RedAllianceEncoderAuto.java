package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.teamcode.SubSystems.Drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Shooter.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Spindexer.OldSpindexerSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Vision.AprilTagNavigator;
import org.firstinspires.ftc.teamcode.SubSystems.Vision.ObeliskMotifDetector;
import org.firstinspires.ftc.teamcode.SubSystems.Scoring.ArtifactColor;

/**
 * Red Alliance Autonomous OpMode
 * 
 * Sequence:
 * 1. Detect obelisk motif during init
 * 2. Drive to shooting position
 * 3. Align with AprilTag goal
 * 4. Shoot 3 preloaded artifacts
 * 5. (Optional) If time permits: drive forward 48", rotate 90° CCW, drive
 * backward to intake
 */
@Autonomous(name = "Red Alliance Auto (Encoders)", group = "Autonomous")
public class RedAllianceEncoderAuto extends OpMode {

    // Subsystems
    private DriveSubsystem drive;
    private IntakeSubsystem intake;
    private ShooterSubsystem shooter;
    private OldSpindexerSubsystem spindexer;
    private AprilTagNavigator aprilTag;
    private ObeliskMotifDetector obeliskDetector;

    // State management
    private int pathState;
    private ElapsedTime actionTimer;
    private ElapsedTime opmodeTimer;
    private ElapsedTime alignmentTimer;

    // Action state flags
    private boolean waitingForShooter;
    private boolean waitingForSpindexer;
    private boolean waitingForFire;
    private boolean intakeActive;
    private int spindexerPositionIndex;
    private int artifactsShot;
    private boolean isAligned;
    private boolean movementStarted;

    // Constants
    private static final double SHOOTER_TARGET_RPM = 5220.0; // Consistent RPM for shooting
    private static final long SHOOTER_SPINUP_TIME_MS = 2000; // 2 seconds for shooter to reach RPM
    private static final long SPINDEXER_MOVE_TIME_MS = 1500; // 1.5 seconds for spindexer movement
    private static final long FIRING_DELAY_MS = 300; // Delay after spindexer moves before counting as shot
    private static final long ALIGNMENT_TIMEOUT_MS = 5000; // Max time to spend aligning
    private static final long MAX_AUTONOMOUS_TIME_MS = 28000; // 28 seconds total (leave 2s buffer)

    // AprilTag Alignment Constants (from RedAllianceTeleOpTest)
    private static final boolean IS_BLUE_ALLIANCE = false;
    private static final int TARGET_TAG_ID = 24; // Red alliance goal tag
    private static final double DESIRED_SHOOTING_DISTANCE = 134.0; // inches
    private static final double DESIRED_SHOOTING_ANGLE = 21.0; // degrees
    private static final double KP_STRAFE = 0.03;
    private static final double KP_FORWARD = 0.03;
    private static final double KP_ROT = 0.015;
    private static final double MAX_AUTO_POWER = 0.40;
    private static final double POSITION_DEADBAND = 0.75; // inches
    private static final double ANGLE_DEADBAND_DEG = 1.5; // degrees

    // Motif detection
    private ArtifactColor[] detectedMotif = null;

    @Override
    public void init() {
        // Initialize timers
        actionTimer = new ElapsedTime();
        opmodeTimer = new ElapsedTime();
        alignmentTimer = new ElapsedTime();

        // Initialize subsystems
        drive = new DriveSubsystem(hardwareMap, telemetry);
        intake = new IntakeSubsystem(hardwareMap, telemetry);
        shooter = new ShooterSubsystem(hardwareMap, telemetry);
        spindexer = new OldSpindexerSubsystem(hardwareMap, telemetry);
        aprilTag = new AprilTagNavigator(drive, hardwareMap, telemetry);
        obeliskDetector = new ObeliskMotifDetector(aprilTag, telemetry);

        // Initialize state
        pathState = 0;
        waitingForShooter = false;
        waitingForSpindexer = false;
        waitingForFire = false;
        intakeActive = false;
        spindexerPositionIndex = 0;
        artifactsShot = 0;
        isAligned = false;
        movementStarted = false;

        // Initialize spindexer to position 0 (preload position)
        spindexer.goToPosition(0);
        spindexer.setIntakeMode(false); // Start in outtake mode

        telemetry.addData("Status", "Initialized - Red Alliance Auto");
        telemetry.addData("Obelisk", "Scanning for motif...");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        // Try to detect obelisk motif before match starts
        obeliskDetector.update();
        detectedMotif = obeliskDetector.getMotif();

        if (detectedMotif != null) {
            telemetry.addData("Obelisk Motif",
                    detectedMotif[0].getCharacter() +
                            detectedMotif[1].getCharacter() +
                            detectedMotif[2].getCharacter());
        } else {
            telemetry.addData("Obelisk Motif", "Not detected yet (will use fallback)");
        }

        // Initialize camera controls when ready
        aprilTag.initializeCameraControls();

        telemetry.addData("Status", "Waiting for start...");
        telemetry.update();
    }

    @Override
    public void start() {
        opmodeTimer.reset();

        // Use detected motif or fallback
        if (detectedMotif == null) {
            detectedMotif = new ArtifactColor[] {
                    ArtifactColor.GREEN, ArtifactColor.PURPLE, ArtifactColor.GREEN
            };
            telemetry.addData("Motif", "Using fallback: GPG");
        } else {
            telemetry.addData("Motif", "Using detected motif");
        }

        // Ensure camera controls are set
        aprilTag.initializeCameraControls();

        setPathState(0);
        telemetry.addData("Status", "Started!");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Update subsystems
        spindexer.update();
        shooter.updateVoltageCompensation();
        aprilTag.updateRobotPositionFromTriangulation();
        
        // CRITICAL SAFETY: Stop shooter when spindexer is moving
        if (spindexer.isMoving() || !spindexer.isAtPosition()) {
            shooter.stop();
        }

        // Check if we're running out of time
        if (opmodeTimer.milliseconds() > MAX_AUTONOMOUS_TIME_MS) {
            // Time's up - stop everything
            stopAllSystems();
            telemetry.addData("Status", "TIME UP - Stopping");
            telemetry.update();
            return;
        }

        // Update autonomous state machine
        autonomousPathUpdate();

        // Update telemetry
        updateTelemetry();
    }

    @Override
    public void stop() {
        // Cleanup
        stopAllSystems();
        if (aprilTag != null) {
            aprilTag.closeVision();
        }
    }

    // ==================== AUTONOMOUS STATE MACHINE ====================

    private void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // Drive to shooting position
                if (!movementStarted) {
                    startNonBlockingMove(48.0, 0, 0, 0.6); // Drive forward 48 inches
                    movementStarted = true;
                    telemetry.addData("Action", "Driving to shooting position");
                }
                if (isMovementComplete()) {
                    movementStarted = false;
                    setPathState(1);
                }
                break;

            case 1:
                // Align with AprilTag goal
                if (alignWithAprilTag()) {
                    isAligned = true;
                    setPathState(2);
                    telemetry.addData("Action", "Aligned - Starting to shoot");
                } else if (alignmentTimer.milliseconds() > ALIGNMENT_TIMEOUT_MS) {
                    // Timeout - proceed anyway
                    telemetry.addData("Action", "Alignment timeout - proceeding");
                    setPathState(2);
                }
                break;

            case 2:
                // Shoot first artifact
                if (artifactsShot == 0) {
                    startShootingSequence();
                }
                if (isShootingComplete()) {
                    artifactsShot++;
                    if (artifactsShot < 3) {
                        // Reset for next shot
                        waitingForShooter = false;
                        waitingForSpindexer = false;
                        waitingForFire = false;
                        actionTimer.reset();
                        telemetry.addData("Action", "Shot " + artifactsShot + "/3 - Preparing next");
                    } else {
                        // All 3 artifacts shot
                        setPathState(3);
                    }
                }
                break;

            case 3:
                // Check if we have time for intake sequence
                if (opmodeTimer.milliseconds() < (MAX_AUTONOMOUS_TIME_MS - 8000)) {
                    // We have at least 8 seconds left - do intake sequence
                    setPathState(10); // Go to intake sequence
                } else {
                    // Not enough time - finish
                    setPathState(-1);
                }
                break;

            case 10:
                // Intake sequence: Drive forward 48"
                if (!movementStarted) {
                    startNonBlockingMove(48.0, 0, 0, 0.6);
                    movementStarted = true;
                    telemetry.addData("Action", "Driving forward 48\" for intake");
                }
                if (isMovementComplete()) {
                    movementStarted = false;
                    setPathState(11);
                }
                break;

            case 11:
                // Rotate 90° counterclockwise
                if (!movementStarted) {
                    startNonBlockingRotate(90.0, 0.5);
                    movementStarted = true;
                    telemetry.addData("Action", "Rotating 90° CCW");
                }
                if (isMovementComplete()) {
                    movementStarted = false;
                    setPathState(12);
                }
                break;

            case 12:
                // Drive backward to intake balls
                if (!movementStarted) {
                    startNonBlockingMove(-24.0, 0, 0, 0.5); // Drive backward 24"
                    startIntake();
                    movementStarted = true;
                    telemetry.addData("Action", "Driving backward and intaking");
                }
                if (isMovementComplete() || actionTimer.seconds() > 3.0) {
                    stopIntake();
                    movementStarted = false;
                    setPathState(-1); // Finish
                }
                break;

            default:
                // Autonomous finished
                stopAllSystems();
                break;
        }
    }

    // ==================== MOVEMENT METHODS (NON-BLOCKING) ====================

    private boolean movementInProgress = false;
    private double movementTargetInches = 0;
    private double rotationTargetDegrees = 0;
    private boolean isRotating = false;

    private void startNonBlockingMove(double inches, double strafe, double turn, double power) {
        if (isRotating) {
            // Finish rotation first
            return;
        }

        double wheelCircumference = Math.PI * 4.0; // WHEEL_DIAMETER = 4.0
        double ticksRequired = (inches / wheelCircumference) * 560.0; // TICKS_PER_REVOLUTION = 560.0

        drive.leftFront.setTargetPosition(drive.leftFront.getCurrentPosition() + (int) ticksRequired);
        drive.leftRear.setTargetPosition(drive.leftRear.getCurrentPosition() + (int) ticksRequired);
        drive.rightFront.setTargetPosition(drive.rightFront.getCurrentPosition() + (int) ticksRequired);
        drive.rightRear.setTargetPosition(drive.rightRear.getCurrentPosition() + (int) ticksRequired);

        drive.leftFront.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION);
        drive.leftRear.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION);
        drive.rightFront.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION);
        drive.rightRear.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION);

        drive.leftFront.setPower(power);
        drive.leftRear.setPower(power);
        drive.rightFront.setPower(power);
        drive.rightRear.setPower(power);

        movementInProgress = true;
        movementTargetInches = inches;
        isRotating = false;
    }

    private void startNonBlockingRotate(double degrees, double power) {
        double wheelCircumference = Math.PI * 4.0;
        double turnDistance = (degrees / 360.0) * wheelCircumference;
        double ticksRequired = (turnDistance / wheelCircumference) * 560.0;
        int ticks = (int) ticksRequired;

        drive.leftFront.setTargetPosition(drive.leftFront.getCurrentPosition() - ticks);
        drive.leftRear.setTargetPosition(drive.leftRear.getCurrentPosition() - ticks);
        drive.rightFront.setTargetPosition(drive.rightFront.getCurrentPosition() + ticks);
        drive.rightRear.setTargetPosition(drive.rightRear.getCurrentPosition() + ticks);

        drive.leftFront.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION);
        drive.leftRear.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION);
        drive.rightFront.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION);
        drive.rightRear.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION);

        drive.leftFront.setPower(power);
        drive.leftRear.setPower(power);
        drive.rightFront.setPower(power);
        drive.rightRear.setPower(power);

        movementInProgress = true;
        rotationTargetDegrees = degrees;
        isRotating = true;
    }

    private boolean isMovementComplete() {
        if (!movementInProgress) {
            return true;
        }

        boolean complete = drive.isAtTarget();
        if (complete) {
            movementInProgress = false;
            drive.stop();
            // Reset to encoder mode
            drive.leftFront.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER);
            drive.leftRear.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER);
            drive.rightFront.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER);
            drive.rightRear.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER);
        }
        return complete;
    }

    // ==================== ALIGNMENT ====================

    private boolean alignWithAprilTag() {
        aprilTag.updateRobotPositionFromAllianceGoals();
        AprilTagDetection tag = aprilTag.getBestAllianceGoalDetection();

        if (tag == null || tag.id != TARGET_TAG_ID) {
            if (alignmentTimer.milliseconds() == 0) {
                alignmentTimer.reset();
            }
            drive.drive(0, 0, 0);
            return false;
        }

        // Use improved alignment method
        double[] corrections = aprilTag.calculateAlignmentCorrections(
                tag, DESIRED_SHOOTING_DISTANCE, DESIRED_SHOOTING_ANGLE,
                POSITION_DEADBAND, POSITION_DEADBAND, ANGLE_DEADBAND_DEG,
                KP_STRAFE, KP_FORWARD, KP_ROT, MAX_AUTO_POWER);

        if (corrections == null) {
            return false;
        }

        double strafePower = corrections[0];
        double forwardPower = corrections[1];
        double turnPower = corrections[2];
        boolean aligned = (corrections[3] == 1.0);

        drive.drive(forwardPower, strafePower, turnPower);

        if (aligned) {
            drive.stop();
            return true;
        }

        return false;
    }

    // ==================== SHOOTING SEQUENCE ====================

    private void startShootingSequence() {
        // Start shooter
        shooter.setTargetRPM(SHOOTER_TARGET_RPM);
        waitingForShooter = true;
        actionTimer.reset();
        telemetry.addData("Action", "Spinning up shooter");
    }

    private boolean isShootingComplete() {
        if (waitingForShooter) {
            if (shooter.isAtTargetRPM() || actionTimer.milliseconds() > SHOOTER_SPINUP_TIME_MS) {
                waitingForShooter = false;
                waitingForFire = true;
                actionTimer.reset();
                return false;
            }
            return false;
        }

        if (waitingForFire) {
            if (actionTimer.milliseconds() > FIRING_DELAY_MS) {
                // CRITICAL: Stop shooter before moving spindexer
                shooter.stop();
                // Move spindexer to next position
                spindexerPositionIndex = (spindexerPositionIndex + 1) % 3;
                spindexer.goToPosition(spindexerPositionIndex);
                waitingForFire = false;
                waitingForSpindexer = true;
                actionTimer.reset();
                return false;
            }
            return false;
        }

        if (waitingForSpindexer) {
            // CRITICAL: Ensure shooter stays stopped while spindexer is moving
            if (spindexer.isMoving() || !spindexer.isAtPosition()) {
                shooter.stop(); // Keep shooter stopped during movement
            }
            
            if (spindexer.isAtPosition() || actionTimer.milliseconds() > SPINDEXER_MOVE_TIME_MS) {
                waitingForSpindexer = false;
                // Restart shooter for next shot
                shooter.setTargetRPM(SHOOTER_TARGET_RPM);
                waitingForShooter = true;
                actionTimer.reset();
                return true; // This shot is complete
            }
            return false;
        }

        return false;
    }

    // ==================== INTAKE ====================

    private void startIntake() {
        intake.start();
        intakeActive = true;
        actionTimer.reset();
    }

    private void stopIntake() {
        intake.stop();
        intakeActive = false;
    }

    // ==================== UTILITIES ====================

    private void stopAllSystems() {
        drive.stop();
        intake.stop();
        shooter.stop();
        spindexer.reset();
    }

    private void setPathState(int state) {
        pathState = state;
        actionTimer.reset();
        if (state == 1) {
            alignmentTimer.reset();
        }
    }

    private void updateTelemetry() {
        telemetry.addData("=== RED ALLIANCE AUTO ===", "");
        telemetry.addData("Path State", pathState);
        telemetry.addData("Elapsed Time", "%.1f s", opmodeTimer.seconds());
        telemetry.addData("Time Remaining", "%.1f s", (MAX_AUTONOMOUS_TIME_MS - opmodeTimer.milliseconds()) / 1000.0);

        // Motif
        if (detectedMotif != null) {
            telemetry.addData("Motif",
                    detectedMotif[0].getCharacter() +
                            detectedMotif[1].getCharacter() +
                            detectedMotif[2].getCharacter());
        }

        // Subsystem status
        telemetry.addData("Artifacts Shot", artifactsShot + "/3");
        telemetry.addData("Aligned", isAligned ? "YES" : "NO");
        telemetry.addData("Shooter RPM", "%.0f / %.0f", shooter.getCurrentRPM(), shooter.getTargetRPM());
        telemetry.addData("Shooter Ready", shooter.isAtTargetRPM());
        telemetry.addData("Spindexer Position", spindexerPositionIndex);
        telemetry.addData("Waiting For Shooter", waitingForShooter);
        telemetry.addData("Waiting For Spindexer", waitingForSpindexer);
        telemetry.addData("Waiting For Fire", waitingForFire);

        // AprilTag info
        aprilTag.updateDECODELocalizationTelemetry();

        telemetry.update();
    }
}
