package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SubSystems.Shooter.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Spindexer.OldSpindexerSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Funnel.FunnelSubsystem;

/**
 * Shooter state machine for autonomous and teleop.
 * Manages shooter RPM, spindexer position, and funnel servos to fire multiple shots.
 * 
 * State flow:
 * IDLE -> SPIN_UP -> LAUNCH -> RESET_GATE -> (repeat or IDLE)
 */
public class ShooterStateMachine {

    private final ShooterSubsystem shooter;
    private final OldSpindexerSubsystem spindexer;
    private final FunnelSubsystem funnel;
    private final Telemetry telemetry;

    // State enumeration
    private enum ShooterState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        RESET_GATE
    }

    private ShooterState currentState;
    private ElapsedTime stateTimer;
    
    // Shots management
    private int shotsRemaining = 0;
    private int currentSpindexerIndex = 0;
    
    // Constants
    private static final double TARGET_SHOOTER_RPM = 5220.0;
    private static final double MIN_SHOOTER_RPM = 5000.0; // Minimum RPM before launching
    private static final double MAX_SPIN_UP_TIME_SEC = 3.0; // Max time to wait for spin-up
    
    private static final double FUNNEL_OPEN_TIME_SEC = 0.4; // Time for funnel to open
    private static final double FUNNEL_CLOSE_TIME_SEC = 0.4; // Time for funnel to close
    
    // State flags
    private boolean intakeMode = false; // false = outtake mode for shooting

    /**
     * Initialize shooter state machine
     * 
     * @param hardwareMap Hardware map from OpMode
     * @param shooter Shooter subsystem
     * @param spindexer Spindexer subsystem
     * @param funnel Funnel subsystem
     * @param telemetry Telemetry for debugging
     */
    public ShooterStateMachine(HardwareMap hardwareMap, ShooterSubsystem shooter, 
                               OldSpindexerSubsystem spindexer, FunnelSubsystem funnel,
                               Telemetry telemetry) {
        this.shooter = shooter;
        this.spindexer = spindexer;
        this.funnel = funnel;
        this.telemetry = telemetry;
        
        this.stateTimer = new ElapsedTime();
        this.currentState = ShooterState.IDLE;
        
        // Set spindexer to outtake mode
        spindexer.setIntakeMode(false);
    }

    /**
     * Update the state machine - must be called every loop
     */
    public void update() {
        switch (currentState) {
            case IDLE:
                if (shotsRemaining > 0) {
                    // Advance spindexer to next position first (will be 0 on first call since currentSpindexerIndex starts at -1)
                    currentSpindexerIndex = (currentSpindexerIndex + 1) % 3;
                    spindexer.goToPositionForCurrentMode(currentSpindexerIndex);
                    // Close funnel and start spinning up
                    funnel.retract();
                    shooter.setTargetRPM(TARGET_SHOOTER_RPM);
                    stateTimer.reset();
                    currentState = ShooterState.SPIN_UP;
                }
                break;
                
            case SPIN_UP:
                // Keep funnel closed and shooter running
                funnel.retract();
                shooter.setTargetRPM(TARGET_SHOOTER_RPM);
                
                // Wait for spindexer to reach position AND shooter to be at speed
                boolean spindexerReady = spindexer.isAtPosition();
                boolean shooterReady = shooter.getCurrentRPM() >= MIN_SHOOTER_RPM || 
                                      stateTimer.seconds() >= MAX_SPIN_UP_TIME_SEC;
                
                if (spindexerReady && shooterReady) {
                    // Both ready - open funnel to launch
                    funnel.extend();
                    stateTimer.reset();
                    currentState = ShooterState.LAUNCH;
                }
                break;
                
            case LAUNCH:
                // Keep shooter running and funnel open
                shooter.setTargetRPM(TARGET_SHOOTER_RPM);
                funnel.extend();
                
                // Wait for funnel to open fully
                if (stateTimer.seconds() >= FUNNEL_OPEN_TIME_SEC) {
                    // Funnel is open, ball should have fired - close funnel
                    funnel.retract();
                    shotsRemaining--;
                    stateTimer.reset();
                    currentState = ShooterState.RESET_GATE;
                }
                break;
                
            case RESET_GATE:
                // Keep shooter running, close funnel
                shooter.setTargetRPM(TARGET_SHOOTER_RPM);
                funnel.retract();
                
                // Wait for funnel to close fully
                if (stateTimer.seconds() >= FUNNEL_CLOSE_TIME_SEC) {
                    // Check if we have more shots remaining
                    if (shotsRemaining > 0) {
                        // More shots to fire - advance spindexer to next position
                        currentSpindexerIndex = (currentSpindexerIndex + 1) % 3;
                        spindexer.goToPositionForCurrentMode(currentSpindexerIndex);
                        stateTimer.reset();
                        currentState = ShooterState.SPIN_UP;
                    } else {
                        // No more shots - stop shooter and return to idle
                        shooter.stop();
                        currentState = ShooterState.IDLE;
                    }
                }
                break;
        }
        
        // Update spindexer PID (must be called every loop)
        spindexer.update();
    }

    /**
     * Request to fire a number of shots
     * Can only be called when state machine is idle
     * 
     * @param numberOfShots Number of shots to fire (1-3)
     */
    public void fireShots(int numberOfShots) {
        if (currentState == ShooterState.IDLE) {
            shotsRemaining = Math.max(1, Math.min(3, numberOfShots)); // Clamp between 1 and 3
            currentSpindexerIndex = -1; // Start at -1 so first increment goes to 0
        }
    }

    /**
     * Check if state machine is currently busy (not idle)
     * 
     * @return True if busy, false if idle
     */
    public boolean isBusy() {
        return currentState != ShooterState.IDLE;
    }

    /**
     * Get number of shots remaining
     * 
     * @return Shots remaining
     */
    public int getShotsRemaining() {
        return shotsRemaining;
    }

    /**
     * Get current state name (for telemetry)
     * 
     * @return State name as string
     */
    public String getStateName() {
        return currentState.toString();
    }

    /**
     * Force stop and reset to idle (emergency stop)
     */
    public void stop() {
        shooter.stop();
        funnel.retract();
        shotsRemaining = 0;
        currentState = ShooterState.IDLE;
    }

    /**
     * Update telemetry with state machine information
     */
    public void updateTelemetry() {
        if (telemetry != null) {
            telemetry.addData("Shooter State", getStateName());
            telemetry.addData("Shots Remaining", shotsRemaining);
            telemetry.addData("Spindexer Index", currentSpindexerIndex);
            telemetry.addData("Shooter RPM", "%.0f / %.0f", shooter.getCurrentRPM(), shooter.getTargetRPM());
            telemetry.addData("Shooter Ready", shooter.isAtTargetRPM());
        }
    }
}

