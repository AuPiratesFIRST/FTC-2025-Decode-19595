package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import org.firstinspires.ftc.teamcode.SubSystems.Drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Shooter.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Spindexer.OldSpindexerSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Vision.AprilTagNavigator;

@Autonomous(name = "Blue Alliance Auto (Encoders)", group = "Autonomous")
public class BlueAllianceEncoderAuto extends OpMode {

    // Subsystems
    private DriveSubsystem drive;
    private IntakeSubsystem intake;
    private ShooterSubsystem shooter;
    private OldSpindexerSubsystem spindexer;

    // State management
    private int pathState;
    private ElapsedTime actionTimer;
    private ElapsedTime opmodeTimer;

    // Action state flags
    private boolean waitingForShooter;
    private boolean waitingForSpindexer;
    private boolean waitingForFire;
    private boolean intakeActive;
    private int spindexerPositionIndex;

    // Constants
    private static final double SHOOTER_POWER = 0.7; // 70% = ~4200 RPM
    private static final long SHOOTER_SPINUP_TIME_MS = 2000; // 2 seconds for shooter to reach RPM
    private static final long INTAKE_TIME_MS = 1500; // 1.5 seconds for intake
    private static final long SPINDEXER_MOVE_TIME_MS = 1000; // 1 second for spindexer movement
    private static final long FIRING_DELAY_MS = 500; // 0.5 seconds for artifact to fire after spindexer moves

    // AprilTag Alignment
    private static final boolean IS_BLUE_ALLIANCE = true;
    private static final int TARGET_TAG_ID = 20; // Blue alliance goal tag
    private AprilTagNavigator aprilTag;
    private static final double ALIGNMENT_TOLERANCE = 2.0; // Tolerance in inches for alignment

    @Override
    public void init() {
        // Initialize timers
        actionTimer = new ElapsedTime();
        opmodeTimer = new ElapsedTime();

        // Initialize subsystems
        drive = new DriveSubsystem(hardwareMap, telemetry);
        
        // Set initial heading for Blue alliance starting at C1 facing goal (135°)
        drive.setPoseHeading(Math.toRadians(135));
        
        // Initialize AprilTag Navigator with corrected 9.0 inch Y-offset
        aprilTag = new AprilTagNavigator(drive, hardwareMap, telemetry);
        
        intake = new IntakeSubsystem(hardwareMap, telemetry);
        shooter = new ShooterSubsystem(hardwareMap, telemetry);
        spindexer = new OldSpindexerSubsystem(hardwareMap, telemetry);

        // Initialize state
        pathState = 0;
        waitingForShooter = false;
        waitingForSpindexer = false;
        waitingForFire = false;
        intakeActive = false;
        spindexerPositionIndex = 0;

        // Initialize spindexer to position 0 (preload position)
        spindexer.goToPosition(0);

        telemetry.addData("Status", "Initialized - Blue Alliance Auto");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        telemetry.addData("Status", "Waiting for start...");
        telemetry.addData("Path State", pathState);
        telemetry.update();
    }

    @Override
    public void start() {
        opmodeTimer.reset();
        setPathState(0);

        telemetry.addData("Status", "Started!");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Update subsystems
        spindexer.update();

        // Update autonomous state machine
        autonomousPathUpdate();

        // Update telemetry
        updateTelemetry();
    }

    @Override
    public void stop() {
        // Cleanup
        drive.stop();
        intake.stop();
        shooter.stop();
        spindexer.reset();
    }

    // Autonomous path state machine (based on encoder-driven actions)
    private void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // Start: Score preload
                driveToScorePreload();
                setPathState(1);
                break;

            case 1:
                // Wait for path completion, then score preload
                if (drive.isAtTarget()) {
                    scoreArtifact();
                    setPathState(2);
                }
                break;

            case 2:
                // Wait for scoring to complete, then move to first intake
                if (isActionComplete()) {
                    driveToIntakeFirstSet();
                    setPathState(3);
                }
                break;

            case 3:
                // Wait for path completion, then start intake
                if (drive.isAtTarget()) {
                    startIntake();
                    setPathState(4);
                }
                break;

            case 4:
                // Intake first set - wait for intake to complete
                if (isActionComplete()) {
                    stopIntake();
                    driveToShootFirstSet();
                    setPathState(5);
                }
                break;

            case 5:
                // Wait for path completion, then score first set
                if (drive.isAtTarget()) {
                    scoreArtifact();
                    setPathState(6);
                }
                break;

            case 6:
                // Wait for scoring to complete, then move to second intake
                if (isActionComplete()) {
                    driveToIntakeSecondSet();
                    setPathState(7);
                }
                break;

            case 7:
                // Wait for path completion, then start intake
                if (drive.isAtTarget()) {
                    startIntake();
                    setPathState(8);
                }
                break;

            case 8:
                // Intake second set - wait for intake to complete
                if (isActionComplete()) {
                    stopIntake();
                    driveToShootSecondSet();
                    setPathState(9);
                }
                break;

            case 9:
                // Wait for path completion, then score second set
                if (drive.isAtTarget()) {
                    scoreArtifact();
                    setPathState(10);
                }
                break;

            case 10:
                // Wait for scoring to complete, then finish
                if (isActionComplete()) {
                    // Autonomous complete
                    setPathState(-1);
                }
                break;

            default:
                // Autonomous finished - stop all systems
                stopAllSystems();
                break;
        }
    }

    // Methods for driving to positions using the new methods
    private void driveToScorePreload() {
        drive.moveInches(417, 1);  // Move forward
        telemetry.addData("Action", "Driving to score preload position");
    }

    private void driveToIntakeFirstSet() {
        drive.moveInches(213, 1);  // Move forward
        drive.strafeInches(-2762, 1);  // Strafe left
        telemetry.addData("Action", "Driving to first intake position");
    }

    private void driveToShootFirstSet() {
        drive.moveInches(-1518, 1);  // Move forward
        drive.strafeInches(2794, 1); // Strafe right
        drive.rotateDegrees(-162, 0.5);  // Rotate to target
        telemetry.addData("Action", "Driving to shoot first set");
    }

    private void driveToIntakeSecondSet() {
        drive.moveInches(279, 1);  // Move forward
        drive.strafeInches(-1747, 1);  // Strafe left
        telemetry.addData("Action", "Driving to second intake position");
    }

    private void driveToShootSecondSet() {
        drive.moveInches(-1293, 1);  // Move forward
        drive.strafeInches(1747, 1);  // Strafe right
        drive.rotateDegrees(-162, 0.5);  // Rotate to target
        telemetry.addData("Action", "Driving to shoot second set");
    }

    // Score an artifact: spin up shooter, move spindexer, fire
    private void scoreArtifact() {
        // Ensure robot is aligned with the target before shooting
        if (!alignWithAprilTag()) {
            telemetry.addData("Alignment", "Aligning with AprilTag...");
            return; // Keep waiting for alignment
        }

        // Start shooter
        shooter.setPower(SHOOTER_POWER);
        waitingForShooter = true;
        actionTimer.reset();

        telemetry.addData("Action", "Scoring artifact - Spinning up shooter");
        telemetry.update();
    }

    // Check for alignment with the AprilTag
    private boolean alignWithAprilTag() {
        // Update robot position from AprilTags
        aprilTag.updateRobotPositionFromAllianceGoals();
        AprilTagDetection blueGoal = aprilTag.getBestAllianceGoalDetection();

        if (blueGoal != null && blueGoal.id == TARGET_TAG_ID) {
            // Calculate left/right offset to determine alignment
            double xOffset = blueGoal.ftcPose.x;  // left/right offset in inches

            // Check if we are within the alignment tolerance
            if (Math.abs(xOffset) < ALIGNMENT_TOLERANCE) {
                return true; // Alignment complete
            }
        }
        return false; // Not aligned yet
    }

    // Start intake sequence
    private void startIntake() {
        intake.start();
        intakeActive = true;
        actionTimer.reset();

        telemetry.addData("Action", "Starting intake");
    }

    // Stop intake
    private void stopIntake() {
        intake.stop();
        intakeActive = false;
    }

    // Check if current action is complete
    private boolean isActionComplete() {
        if (waitingForShooter) {
            if (shooter.isAtTargetRPM() || actionTimer.seconds() > (SHOOTER_SPINUP_TIME_MS / 1000.0)) {
                waitingForShooter = false;
                waitingForFire = true;
                actionTimer.reset();
                return false; // Still waiting for fire
            }
            return false;
        }

        if (waitingForFire) {
            if (actionTimer.seconds() > (FIRING_DELAY_MS / 1000.0)) {
                spindexerPositionIndex = (spindexerPositionIndex + 1) % 3;
                spindexer.goToPosition(spindexerPositionIndex);
                waitingForFire = false;
                waitingForSpindexer = true;
                actionTimer.reset();
                return false; // Still waiting for spindexer
            }
            return false;
        }

        if (waitingForSpindexer) {
            if (spindexer.isAtPosition() || actionTimer.seconds() > (SPINDEXER_MOVE_TIME_MS / 1000.0)) {
                waitingForSpindexer = false;
                shooter.stop();
                return true; // Action complete
            }
            return false;
        }

        if (intakeActive) {
            if (actionTimer.seconds() > (INTAKE_TIME_MS / 1000.0)) {
                return true; // Intake complete
            }
            return false;
        }

        // No active action
        return true;
    }

    // Stop all systems
    private void stopAllSystems() {
        intake.stop();
        shooter.stop();
        spindexer.reset();
    }

    // Set path state and reset timer
    private void setPathState(int state) {
        pathState = state;
    }

    // Update telemetry
    private void updateTelemetry() {
        telemetry.addData("=== BLUE ALLIANCE AUTO ===", "");
        telemetry.addData("Path State", pathState);
        telemetry.addData("Elapsed Time", "%.1f s", opmodeTimer.seconds());

        // Subsystem status
        telemetry.addData("Intake Active", intakeActive);
        telemetry.addData("Shooter RPM", "%.0f / %.0f", shooter.getCurrentRPM(), shooter.getTargetRPM());
        telemetry.addData("Shooter Ready", shooter.isAtTargetRPM());
        telemetry.addData("Spindexer Position", spindexerPositionIndex);
        telemetry.addData("Waiting For Shooter", waitingForShooter);
        telemetry.addData("Waiting For Spindexer", waitingForSpindexer);
        telemetry.addData("Waiting For Fire", waitingForFire);

        telemetry.update();
    }
}
