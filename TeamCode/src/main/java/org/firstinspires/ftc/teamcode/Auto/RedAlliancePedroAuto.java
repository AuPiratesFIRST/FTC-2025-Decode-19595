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
// import org.firstinspires.ftc.teamcode.SubSystems.Vision.AprilTagNavigator; // Not needed - Pedro uses encoder localization
import org.firstinspires.ftc.teamcode.SubSystems.Funnel.FunnelSubsystem;

/**
 * Red Alliance Autonomous using Pedro Pathing with state machines.
 * 
 * Autonomous sequence:
 * 1. Drive to shooting position
 * 2. Shoot 3 preloaded artifacts
 * 3. Drive to intake area
 * 4. Intake artifacts
 * 5. Drive back to shooting position
 * 6. Shoot intaked artifacts
 * 
 * Uses parallel state machines for path following and mechanism control.
 */
@Autonomous(name = "Red Alliance Pedro Auto", group = "Autonomous")
public class RedAlliancePedroAuto extends OpMode {

    // Pedro Pathing
    private Follower follower;
    
    // Subsystems
    private DriveSubsystem drive;
    private IntakeSubsystem intake;
    private ShooterSubsystem shooter;
    private OldSpindexerSubsystem spindexer;
    // Note: AprilTag localization disabled - Pedro Pathing uses its own encoder-based localizer
    // private AprilTagNavigator aprilTag;
    private FunnelSubsystem funnel; // Note: Actually a camshaft mechanism that moves balls from spindexer to shooter
    
    // State machines
    private ShooterStateMachine shooterStateMachine;
    
    // Path state enumeration
    private enum PathState {
        DRIVE_TO_SHOOT,
        SHOOT_PRELOAD,
        DRIVE_TO_INTAKE,
        INTAKE_ARTIFACTS,
        DRIVE_TO_SHOOT_AGAIN,
        SHOOT_INTAKED,
        FINISHED
    }
    
    private PathState currentPathState;
    
    // Action flags
    private boolean shotsTriggered = false;
    private boolean intakeActive = false;
    private Timer actionTimer;
    
    // Path chains
    private PathChain driveToShoot;
    private PathChain driveToIntake;
    private PathChain intakePath;
    private PathChain driveToShootAgain;
    
    // Starting pose (Red Alliance starting position)
    // Pedro coordinates: [0, 144] on both axes, (0,0) is bottom-left
    private final Pose startPose = new Pose(72, 8, Math.toRadians(90));
    
    // Constants
    private static final long INTAKE_TIME_MS = 3000; // 3 seconds for intake (enough time for 3 artifacts)
    private static final int PRELOAD_SHOTS = 3; // 3 preloaded artifacts
    private static final int INTAKED_SHOTS = 3; // 3 artifacts intaked

    @Override
    public void init() {
        // Initialize timers
        actionTimer = new Timer();
        
        // Initialize Pedro Pathing follower
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        
        // Initialize subsystems
        drive = new DriveSubsystem(hardwareMap, telemetry);
        intake = new IntakeSubsystem(hardwareMap, telemetry);
        shooter = new ShooterSubsystem(hardwareMap, telemetry);
        spindexer = new OldSpindexerSubsystem(hardwareMap, telemetry);
        // Note: AprilTag localization disabled - Pedro Pathing uses its own encoder-based localizer
        // AprilTag would use a different coordinate system and could conflict with Pedro's localization
        funnel = new FunnelSubsystem(hardwareMap, telemetry); // Note: Actually a camshaft mechanism
        
        // Initialize shooter state machine
        shooterStateMachine = new ShooterStateMachine(
            hardwareMap, shooter, spindexer, funnel, telemetry);
        
        // Build paths
        buildPaths();
        
        // Initialize state
        currentPathState = PathState.DRIVE_TO_SHOOT;
        shotsTriggered = false;
        intakeActive = false;
        
        telemetry.addData("Status", "Initialized - Red Alliance Pedro Auto");
        telemetry.addData("Starting Pose", "X: %.1f, Y: %.1f, Heading: %.1f°", 
                startPose.getX(), startPose.getY(), Math.toDegrees(startPose.getHeading()));
        telemetry.update();
    }
    
    @Override
    public void init_loop() {
        // Update follower during init for localization
        follower.update();
        
        // Note: Pedro Pathing handles its own localization via encoders
        // AprilTag localization disabled to avoid coordinate system conflicts
        
        telemetry.addData("Status", "Waiting for start...");
        telemetry.addData("Path State", currentPathState.toString());
        telemetry.update();
    }
    
    @Override
    public void start() {
        actionTimer.resetTimer();
        
        // Set initial path state
        setPathState(PathState.DRIVE_TO_SHOOT);
        
        telemetry.addData("Status", "Started!");
        telemetry.update();
    }
    
    @Override
    public void loop() {
        // Update Pedro Pathing follower (must be called continuously)
        // Pedro Pathing handles its own localization via encoders
        follower.update();
        
        // Note: AprilTag localization disabled - Pedro uses encoder-based localization
        
        // Update shooter voltage compensation
        shooter.updateVoltageCompensation();
        
        // Update shooter state machine (must be called every loop)
        shooterStateMachine.update();
        
        // Update autonomous path state machine
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
        funnel.retract();
        shooterStateMachine.stop();
    }
    
    /**
     * Build all paths for the autonomous routine
     */
    private void buildPaths() {
        // Drive from start to shooting position
        driveToShoot = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(72, 8),        // Start position
                        new Pose(80, 20),       // Control point 1
                        new Pose(90, 40)        // Shooting position
                ))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(64))
                .build();
        
        // Drive from shooting position to intake area
        driveToIntake = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(90, 40),       // Shooting position
                        new Pose(100, 35),      // Control point 1
                        new Pose(110, 35)       // Intake approach
                ))
                .setTangentHeadingInterpolation()
                .build();
        
        // Intake path (move forward while intaking)
        intakePath = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(110, 35),      // Intake start
                        new Pose(130, 35)       // Intake end
                ))
                .setTangentHeadingInterpolation()
                .build();
        
        // Drive back to shooting position
        driveToShootAgain = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(130, 35),      // Intake end
                        new Pose(100, 45),      // Control point 1
                        new Pose(90, 40)        // Shooting position
                ))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();
    }
    
    /**
     * Autonomous path state machine
     */
    private void autonomousPathUpdate() {
        switch (currentPathState) {
            case DRIVE_TO_SHOOT:
                // Drive to shooting position
                if (!follower.isBusy()) {
                    follower.followPath(driveToShoot);
                    setPathState(PathState.SHOOT_PRELOAD);
                }
                break;
                
            case SHOOT_PRELOAD:
                // Wait for path to complete (robot is now stationary at shooting position)
                if (!follower.isBusy()) {
                    if (!shotsTriggered) {
                        // Robot is stationary - trigger shooting sequence (fires all 3 preloaded artifacts)
                        shooterStateMachine.fireShots(PRELOAD_SHOTS);
                        shotsTriggered = true;
                    }
                    
                    // Wait for shooting to complete (all 3 shots fired while stationary)
                    if (!shooterStateMachine.isBusy()) {
                        // All shots complete - now drive to intake
                        setPathState(PathState.DRIVE_TO_INTAKE);
                    }
                }
                break;
                
            case DRIVE_TO_INTAKE:
                // Drive to intake area
                if (!follower.isBusy()) {
                    follower.followPath(driveToIntake);
                    setPathState(PathState.INTAKE_ARTIFACTS);
                    shotsTriggered = false; // Reset for next shooting sequence
                }
                break;
                
            case INTAKE_ARTIFACTS:
                // Wait for approach path to complete, then start intake
                if (!follower.isBusy()) {
                    if (!intakeActive) {
                        // Start intake and drive forward while intaking (intakes 3 artifacts)
                        follower.followPath(intakePath);
                        intake.start();
                        intakeActive = true;
                        actionTimer.resetTimer();
                    } else {
                        // Check if intake is complete (path finished and enough time elapsed for 3 artifacts)
                        if (!follower.isBusy() && 
                            actionTimer.getElapsedTimeMs() >= INTAKE_TIME_MS) {
                            // Intake complete - stop intake
                            intake.stop();
                            intakeActive = false;
                            setPathState(PathState.DRIVE_TO_SHOOT_AGAIN);
                        }
                    }
                }
                break;
                
            case DRIVE_TO_SHOOT_AGAIN:
                // Drive back to shooting position
                if (!follower.isBusy()) {
                    follower.followPath(driveToShootAgain);
                    setPathState(PathState.SHOOT_INTAKED);
                    shotsTriggered = false; // Reset for next shooting sequence
                }
                break;
                
            case SHOOT_INTAKED:
                // Wait for path to complete (robot is now stationary at shooting position)
                if (!follower.isBusy()) {
                    if (!shotsTriggered) {
                        // Robot is stationary - trigger shooting sequence (fires all 3 intaked artifacts)
                        shooterStateMachine.fireShots(INTAKED_SHOTS);
                        shotsTriggered = true;
                    }
                    
                    // Wait for shooting to complete (all 3 shots fired while stationary)
                    if (!shooterStateMachine.isBusy()) {
                        // All shots complete - autonomous finished
                        setPathState(PathState.FINISHED);
                    }
                }
                break;
                
            case FINISHED:
                // Autonomous complete - stop all systems
                stopAllSystems();
                break;
        }
    }
    
    /**
     * Set path state and reset trigger flags
     */
    private void setPathState(PathState state) {
        currentPathState = state;
        shotsTriggered = false;
        actionTimer.resetTimer();
    }
    
    /**
     * Stop all systems
     */
    private void stopAllSystems() {
        intake.stop();
        shooter.stop();
        funnel.retract();
        shooterStateMachine.stop();
    }
    
    /**
     * Update telemetry
     */
    private void updateTelemetry() {
        telemetry.addData("=== RED ALLIANCE PEDRO AUTO ===", "");
        telemetry.addData("Path State", currentPathState.toString());
        telemetry.addData("Follower Busy", follower.isBusy());
        telemetry.addData("Robot Position", "X: %.1f, Y: %.1f", 
                follower.getPose().getX(), follower.getPose().getY());
        telemetry.addData("Robot Heading", "%.1f°", 
                Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Elapsed Time", "%.1f s", 
                actionTimer.getElapsedTimeSeconds());
        
        // Shooter state machine status
        shooterStateMachine.updateTelemetry();
        
        // Intake status
        telemetry.addData("Intake Active", intakeActive);
        
        // Note: Pedro Pathing uses encoder-based localization (no AprilTag needed)
        // Position shown above is from Pedro Pathing's localizer
        
        telemetry.update();
    }
}

