package org.firstinspires.ftc.teamcode.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.SubSystems.Drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Shooter.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Spindexer.OldSpindexerSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Vision.AprilTagNavigator;

/**
 * Red Alliance Autonomous using Pedro Pathing.
 * 
 * Autonomous sequence:
 * 1. Score preload
 * 2. Intake first set of artifacts
 * 3. Score first set
 * 4. Intake second set of artifacts
 * 5. Score second set
 * 
 * Uses AprilTag localization for position updates.
 */
@Autonomous(name = "Red Alliance Auto", group = "Autonomous")
public class RedAllianceAuto extends OpMode {

    // Pedro Pathing
    private Follower follower;
    
    // Subsystems
    private DriveSubsystem drive;
    private IntakeSubsystem intake;
    private ShooterSubsystem shooter;
    private OldSpindexerSubsystem spindexer;
    private AprilTagNavigator aprilTag;
    
    // State management
    private int pathState;
    private Timer pathTimer;
    private Timer actionTimer;
    private Timer opmodeTimer;
    
    // Action state flags
    private boolean waitingForShooter;
    private boolean waitingForSpindexer;
    private boolean waitingForFire;
    private boolean intakeActive;
    private int spindexerPositionIndex;
    
    // Paths
    private PathChain scorePreload;
    private PathChain goingToIntakeFirstSet;
    private PathChain intakeFirstSet;
    private PathChain shootFirstSet;
    private PathChain goingToIntakeSecondSet;
    private PathChain intakeSecondSet;
    private PathChain shootSecondSet;
    
    // Starting pose (Red Alliance starting position)
    // Pedro coordinates: [0, 144] on both axes, (0,0) is bottom-left
    // Add +72 to RoadRunner coordinates to convert
    private final Pose startPose = new Pose(72, 8, Math.toRadians(90));
    
    // Shooter settings
    private static final double SHOOTER_POWER = 0.7; // 70% = ~4200 RPM
    private static final long SHOOTER_SPINUP_TIME_MS = 2000; // 2 seconds for shooter to reach RPM
    private static final long INTAKE_TIME_MS = 1500; // 1.5 seconds for intake
    private static final long SPINDEXER_MOVE_TIME_MS = 1000; // 1 second for spindexer movement
    private static final long FIRING_DELAY_MS = 500; // 0.5 seconds for artifact to fire after spindexer moves
    
    @Override
    public void init() {
        // Initialize timers
        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        
        // Initialize Pedro Pathing follower
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        
        // Initialize subsystems
        drive = new DriveSubsystem(hardwareMap, telemetry);
        intake = new IntakeSubsystem(hardwareMap, telemetry);
        shooter = new ShooterSubsystem(hardwareMap, telemetry);
        spindexer = new OldSpindexerSubsystem(hardwareMap, telemetry);
        aprilTag = new AprilTagNavigator(drive, hardwareMap, telemetry);
        
        // Build paths
        buildPaths();
        
        // Initialize state
        pathState = 0;
        waitingForShooter = false;
        waitingForSpindexer = false;
        waitingForFire = false;
        intakeActive = false;
        spindexerPositionIndex = 0;
        
        // Initialize spindexer to position 0 (preload position)
        spindexer.goToPosition(0);
        
        telemetry.addData("Status", "Initialized - Red Alliance Auto");
        telemetry.addData("Starting Pose", "X: %.1f, Y: %.1f, Heading: %.1f°", 
                startPose.getX(), startPose.getY(), Math.toDegrees(startPose.getHeading()));
        telemetry.update();
    }
    
    @Override
    public void init_loop() {
        // Update follower during init for localization
        follower.update();
        
        // Update AprilTag localization
        aprilTag.updateRobotPositionFromAllianceGoals();
        
        telemetry.addData("Status", "Waiting for start...");
        telemetry.addData("Path State", pathState);
        telemetry.update();
    }
    
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
        
        telemetry.addData("Status", "Started!");
        telemetry.update();
    }
    
    @Override
    public void loop() {
        // Update Pedro Pathing follower (must be called continuously)
        follower.update();
        
        // Update AprilTag localization
        aprilTag.updateRobotPositionFromAllianceGoals();
        
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
        aprilTag.closeVision();
    }
    
    /**
     * Build all paths for the autonomous routine
     */
    private void buildPaths() {
        // Score preload path
        scorePreload = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(90.551, 3.122),
                        new Pose(87.612, 39.306),
                        new Pose(99.918, 97.714)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(-107))
                .build();
        
        // Going to intake first set
        goingToIntakeFirstSet = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(99.918, 97.714),
                        new Pose(88.531, 33.429),
                        new Pose(104.694, 35.816)
                ))
                .setTangentHeadingInterpolation()
                .build();
        
        // Intake first set
        intakeFirstSet = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(104.694, 35.816),
                        new Pose(134.082, 35.449)
                ))
                .setTangentHeadingInterpolation()
                .build();
        
        // Shoot first set
        shootFirstSet = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(134.082, 35.449),
                        new Pose(90.551, 47.571),
                        new Pose(100.102, 98.082)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(-133))
                .build();
        
        // Going to intake second set
        goingToIntakeSecondSet = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(100.102, 98.082),
                        new Pose(93.857, 59.143),
                        new Pose(106.347, 58.959)
                ))
                .setTangentHeadingInterpolation()
                .build();
        
        // Intake second set
        intakeSecondSet = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(106.347, 58.959),
                        new Pose(129.673, 58.776)
                ))
                .setTangentHeadingInterpolation()
                .build();
        
        // Shoot second set
        shootSecondSet = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(129.673, 58.776),
                        new Pose(94.959, 53.082),
                        new Pose(100.653, 97.898)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(-133))
                .build();
    }
    
    /**
     * Autonomous path state machine
     */
    private void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // Start: Score preload
                follower.followPath(scorePreload);
                setPathState(1);
                break;
                
            case 1:
                // Wait for path completion, then score preload
                if (!follower.isBusy()) {
                    // Score preload action
                    scoreArtifact();
                    setPathState(2);
                }
                break;
                
            case 2:
                // Wait for scoring to complete, then move to first intake
                if (isActionComplete()) {
                    follower.followPath(goingToIntakeFirstSet, true);
                    setPathState(3);
                }
                break;
                
            case 3:
                // Wait for path completion, then start intake
                if (!follower.isBusy()) {
                    follower.followPath(intakeFirstSet, true);
                    startIntake();
                    setPathState(4);
                }
                break;
                
            case 4:
                // Intake first set - wait for intake to complete
                if (!follower.isBusy() && isActionComplete()) {
                    stopIntake();
                    follower.followPath(shootFirstSet, true);
                    setPathState(5);
                }
                break;
                
            case 5:
                // Wait for path completion, then score first set
                if (!follower.isBusy()) {
                    scoreArtifact();
                    setPathState(6);
                }
                break;
                
            case 6:
                // Wait for scoring to complete, then move to second intake
                if (isActionComplete()) {
                    follower.followPath(goingToIntakeSecondSet, true);
                    setPathState(7);
                }
                break;
                
            case 7:
                // Wait for path completion, then start intake
                if (!follower.isBusy()) {
                    follower.followPath(intakeSecondSet, true);
                    startIntake();
                    setPathState(8);
                }
                break;
                
            case 8:
                // Intake second set - wait for intake to complete
                if (!follower.isBusy() && isActionComplete()) {
                    stopIntake();
                    follower.followPath(shootSecondSet, true);
                    setPathState(9);
                }
                break;
                
            case 9:
                // Wait for path completion, then score second set
                if (!follower.isBusy()) {
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
    
    /**
     * Score an artifact: spin up shooter, move spindexer, fire
     */
    private void scoreArtifact() {
        // Start shooter
        shooter.setPower(SHOOTER_POWER);
        waitingForShooter = true;
        actionTimer.resetTimer();
        
        telemetry.addData("Action", "Scoring artifact - Spinning up shooter");
    }
    
    /**
     * Start intake sequence
     */
    private void startIntake() {
        intake.start();
        intakeActive = true;
        actionTimer.resetTimer();
        
        telemetry.addData("Action", "Starting intake");
    }
    
    /**
     * Stop intake
     */
    private void stopIntake() {
        intake.stop();
        intakeActive = false;
    }
    
    /**
     * Check if current action is complete
     * 
     * Scoring sequence:
     * 1. Wait for shooter to reach RPM (fire from current position)
     * 2. Wait for firing delay (artifact fires)
     * 3. Advance spindexer to next position
     * 4. Wait for spindexer to reach position
     * 5. Action complete
     */
    private boolean isActionComplete() {
        if (waitingForShooter) {
            // Wait for shooter to reach target RPM
            if (shooter.isAtTargetRPM() || actionTimer.getElapsedTimeSeconds() > (SHOOTER_SPINUP_TIME_MS / 1000.0)) {
                // Shooter ready - artifact fires from current position
                // Now wait for firing delay
                waitingForShooter = false;
                waitingForFire = true;
                actionTimer.resetTimer();
                return false; // Still waiting for fire
            }
            return false;
        }
        
        if (waitingForFire) {
            // Wait for artifact to fire from current position
            if (actionTimer.getElapsedTimeSeconds() > (FIRING_DELAY_MS / 1000.0)) {
                // Artifact fired - now advance spindexer to next position
                spindexerPositionIndex = (spindexerPositionIndex + 1) % 3;
                spindexer.goToPosition(spindexerPositionIndex);
                waitingForFire = false;
                waitingForSpindexer = true;
                actionTimer.resetTimer();
                return false; // Still waiting for spindexer
            }
            return false;
        }
        
        if (waitingForSpindexer) {
            // Wait for spindexer to reach next position
            if (spindexer.isAtPosition() || actionTimer.getElapsedTimeSeconds() > (SPINDEXER_MOVE_TIME_MS / 1000.0)) {
                // Spindexer at next position - action complete
                waitingForSpindexer = false;
                // Stop shooter after scoring sequence
                shooter.stop();
                return true; // Action complete
            }
            return false;
        }
        
        if (intakeActive) {
            // Wait for intake duration
            if (actionTimer.getElapsedTimeSeconds() > (INTAKE_TIME_MS / 1000.0)) {
                return true; // Intake complete
            }
            return false;
        }
        
        // No active action
        return true;
    }
    
    /**
     * Stop all systems
     */
    private void stopAllSystems() {
        intake.stop();
        shooter.stop();
        spindexer.reset();
    }
    
    /**
     * Set path state and reset timer
     */
    private void setPathState(int state) {
        pathState = state;
        pathTimer.resetTimer();
    }
    
    /**
     * Update telemetry
     */
    private void updateTelemetry() {
        telemetry.addData("=== RED ALLIANCE AUTO ===", "");
        telemetry.addData("Path State", pathState);
        telemetry.addData("Follower Busy", follower.isBusy());
        telemetry.addData("Robot Position", "X: %.1f, Y: %.1f", 
                follower.getPose().getX(), follower.getPose().getY());
        telemetry.addData("Robot Heading", "%.1f°", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Elapsed Time", "%.1f s", opmodeTimer.getElapsedTimeSeconds());
        
        // Subsystem status
        telemetry.addData("Intake Active", intakeActive);
        telemetry.addData("Shooter RPM", "%.0f / %.0f", shooter.getCurrentRPM(), shooter.getTargetRPM());
        telemetry.addData("Shooter Ready", shooter.isAtTargetRPM());
        telemetry.addData("Spindexer Position", spindexerPositionIndex);
        telemetry.addData("Spindexer At Position", spindexer.isAtPosition());
        telemetry.addData("Waiting For Shooter", waitingForShooter);
        telemetry.addData("Waiting For Spindexer", waitingForSpindexer);
        telemetry.addData("Waiting For Fire", waitingForFire);
        
        // AprilTag info
        aprilTag.updateDECODELocalizationTelemetry();
        
        telemetry.update();
    }
}

