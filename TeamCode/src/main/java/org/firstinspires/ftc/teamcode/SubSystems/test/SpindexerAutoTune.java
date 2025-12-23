package org.firstinspires.ftc.teamcode.SubSystems.test;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.SubSystems.Spindexer.OldSpindexerSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Intake.IntakeSubsystem;

/**
 * Spindexer Auto PID Tuner
 * 
 * Based on WPI Turret Position Controller tuning principles:
 * - Tests all 3 positions in sequence (0→1→2) like real TeleOp usage
 * - Uses goToPositionForCurrentMode() to match actual system behavior
 * - Tuning order: P first, then D, then I (per WPI position control tuning)
 * - Aggregates metrics across all positions for comprehensive evaluation
 * 
 * WPI Tuning Procedure (for position control):
 * 1. Set K_p, K_i, K_d to zero
 * 2. Increase K_p until output oscillates around setpoint, then reduce until oscillations stop
 * 3. Increase K_d as much as possible without introducing jittering
 * 4. If steady-state error exists, increase K_i (but feedforward is preferred over I)
 * 
 * This ensures the tuned PID values work well for the actual cycling pattern
 * used during intake and outtake operations.
 */
@TeleOp(name = "Spindexer Auto-Tune", group = "Test")
public class SpindexerAutoTune extends LinearOpMode {

    private OldSpindexerSubsystem spindexer;
    private IntakeSubsystem intake;
    private TelemetryManager telemetryM;
    
    private enum TuningPhase {
        IDLE,
        TUNING_P,
        LOADING_BALLS,  // Pause for loading balls between P and D/I tuning
        TUNING_D,
        TUNING_I,
        COMPLETE
    }

    private TuningPhase currentPhase = TuningPhase.IDLE;

    // PID values and limits
    private double bestP = 0.01;
    private double bestD = 0.02;
    private double bestI = 0.0;
    private double currentP = 0.001;
    private double currentD = 0.0;
    private double currentI = 0.0;

    // WPI Exponential Search: Multiply by 2 until too large, then binary search
    // This is much faster than linear increments
    private static final double P_MIN = 0.001;
    private static final double P_MAX = 0.05;
    private double pLow = P_MIN;      // Lower bound for binary search
    private double pHigh = P_MAX;     // Upper bound for binary search
    private boolean pOscillating = false;  // Track if we've seen oscillations
    private boolean pInBinarySearch = false; // Are we in binary search phase?

    private static final double D_MIN = 0.0;
    private static final double D_MAX = 0.15;
    private double dLow = D_MIN;      // Lower bound for binary search
    private double dHigh = D_MAX;     // Upper bound for binary search
    private boolean dJittering = false;  // Track if we've seen jittering
    private boolean dInBinarySearch = false; // Are we in binary search phase?

    private static final double I_MIN = 0.0;
    private static final double I_MAX = 0.005;
    private static final double I_INCREMENT = 0.0001; // I uses linear search (small range)

    private static class PerformanceMetrics {
        double settlingTime;
        double maxOvershoot;
        double steadyStateError;
        int oscillationCount;
        boolean oscillating;
        boolean jittering;  // High-frequency small oscillations (D too high)
        double score;

        void reset() {
            settlingTime = 0;
            maxOvershoot = 0;
            steadyStateError = 0;
            oscillationCount = 0;
            oscillating = false;
            jittering = false;
            score = Double.MAX_VALUE;
        }
    }

    private final PerformanceMetrics metrics = new PerformanceMetrics();
    private final PerformanceMetrics bestMetrics = new PerformanceMetrics();

    private boolean testInProgress = false;
    private boolean testJustFinished = false;
    private boolean spindexerIsMoving = false;
    private boolean hasReachedTarget = false;
    private long motionStartTime = 0;
    private long targetReachedTime = 0;
    private static final long MAX_TEST_TIME_MS = 15000; // Increased for 3-position sequence
    private static final long STEADY_STATE_TIME_MS = 300; // Time to maintain position for steady state
    private static final int POSITION_TOLERANCE = 15;
    private double lastError = 0;
    private int errorSignChanges = 0;
    private double maxOvershoot = 0; // Track overshoot only after reaching target
    private double steadyStateErrorSum = 0;
    private int steadyStateSamples = 0;
    private boolean hasCrossedTarget = false; // Track if we've crossed target at least once
    private long settledStartTime = 0; // When we first entered tolerance
    
    // Jitter detection for D tuning (WPI: increase D until jittering appears)
    private double lastErrorChange = 0;
    private int rapidErrorChanges = 0; // Count rapid small error changes (jitter)
    private static final double JITTER_THRESHOLD = 2.0; // Small error changes indicate jitter
    private static final int JITTER_COUNT_THRESHOLD = 10; // Number of rapid changes to consider jittering
    
    // Position sequence tracking (like TeleOp: 0→1→2→0)
    private int currentPositionIndex = 0;
    private int positionsCompleted = 0;
    private static final int TOTAL_POSITIONS_TO_TEST = 3; // Test all 3 positions
    private boolean intakeMode = true; // Test in intake mode (can be changed if needed)
    
    // Aggregate metrics across all positions
    private double totalSettlingTime = 0;
    private double totalMaxOvershoot = 0;
    private double totalSteadyStateError = 0;
    private int totalOscillationCount = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        spindexer = new OldSpindexerSubsystem(hardwareMap, telemetry);
        intake = new IntakeSubsystem(hardwareMap, telemetry);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        
        // Step 1: Manual physical alignment
        telemetryM.addLine("=== MANUAL ALIGNMENT ===");
        telemetryM.addLine("Please move spindexer to physical zero (0) position.");
        telemetryM.addLine("This should be the starting position (position 0).");
        telemetryM.addLine("");
        telemetryM.addLine("Press A when spindexer is at physical zero...");
        telemetryM.update(telemetry);
        
        // Wait for operator to physically align spindexer
        boolean aPressed = false;
        while (opModeIsActive() && !isStarted() && !isStopRequested()) {
            if (gamepad1.a && !aPressed) {
                aPressed = true;
                break;
            }
            aPressed = gamepad1.a;
            sleep(20);
        }
        
        // Step 2: Software encoder reset (aligns software zero with physical zero)
        telemetryM.addLine("Resetting encoder to software zero...");
        telemetryM.update(telemetry);
        
        spindexer.reset();  // Resets encoder to define software zero
        sleep(500);
        
        // Step 3: Verify encoder is near zero (optional safety check)
        int currentPos = spindexer.getCurrentPosition();
        if (Math.abs(currentPos) > 5) {  // Allow small tolerance
            telemetryM.addLine("⚠️ WARNING: Encoder not zeroed correctly!");
            telemetryM.addData("Encoder reads", currentPos);
            telemetryM.addLine("Expected: ~0");
            telemetryM.addLine("This may indicate mechanical misalignment.");
            telemetryM.addLine("");
            telemetryM.addLine("Press A to continue anyway, or X to reset.");
            telemetryM.update(telemetry);
            
            // Wait for user acknowledgment
            boolean acknowledged = false;
            while (opModeIsActive() && !isStarted() && !isStopRequested() && !acknowledged) {
                if (gamepad1.a) {
                    acknowledged = true;
                } else if (gamepad1.x) {
                    // Retry reset
                    spindexer.reset();
                    sleep(500);
                    currentPos = spindexer.getCurrentPosition();
                    if (Math.abs(currentPos) <= 5) {
                        acknowledged = true;
                    }
                }
                sleep(20);
            }
        }
        
        // Step 4: Ready to start tuning
        telemetryM.addLine("Spindexer Auto PID Tuner");
        telemetryM.addLine("Encoder zeroed: " + spindexer.getCurrentPosition());
        telemetryM.addLine("");
        telemetryM.addLine("A = Start");
        telemetryM.addLine("B = Skip Phase");
        telemetryM.addLine("X = Reset");
        telemetryM.addLine("Y = Finish");
        telemetryM.update(telemetry);

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a) {
                if (currentPhase == TuningPhase.IDLE) {
                    startTuning();
                    sleep(300);
                } else if (currentPhase == TuningPhase.LOADING_BALLS) {
                    // User has loaded balls and is ready to continue
                    startDITuning();
                    sleep(300);
                }
            }

            if (gamepad1.b && !testInProgress && currentPhase != TuningPhase.COMPLETE && currentPhase != TuningPhase.LOADING_BALLS) {
                skipPhase();
                sleep(300);
            }

            if (gamepad1.x) {
                resetTuning();
                sleep(300);
            }

            if (gamepad1.y && currentPhase != TuningPhase.IDLE) {
                finishTuning();
                sleep(300);
            }

            if (testInProgress) {
                runTest();
            } else if (testJustFinished && currentPhase != TuningPhase.COMPLETE) {
                testJustFinished = false;
                continueTuning();
            }

            spindexer.update();
            updateTelemetry();
            sleep(20);
        }

        spindexer.setManualPower(0);
        intake.stop();
    }

    // Start tuning process
    // WPI Procedure: Set K_p, K_i, K_d to zero, then increase K_p until oscillations
    private void startTuning() {
        currentPhase = TuningPhase.TUNING_P;
        currentP = P_MIN;
        currentD = 0;
        currentI = 0;
        pLow = P_MIN;
        pHigh = P_MAX;
        pOscillating = false;
        pInBinarySearch = false;
        dLow = D_MIN;
        dHigh = D_MAX;
        dJittering = false;
        dInBinarySearch = false;
        bestMetrics.reset();
        startTest();
    }

    // Continue tuning based on current phase
    // Uses WPI exponential search: multiply by 2 until oscillations/jitter, then binary search
    private void continueTuning() {
        if (currentPhase == TuningPhase.TUNING_P) {
            // WPI: Increase K_p until oscillations start, then reduce until they stop
            if (!pOscillating && !pInBinarySearch) {
                // Exponential search phase: multiply by 2
                double previousP = currentP;
                currentP *= 2.0;
                if (currentP > P_MAX) {
                    currentP = P_MAX;
                    // If we hit max without oscillations, P tuning might be complete
                    if (!metrics.oscillating && metrics.steadyStateError <= POSITION_TOLERANCE) {
                        currentPhase = TuningPhase.LOADING_BALLS;
                        prepareForBallLoading();
                        return;
                    }
                }
                // Check if we found oscillations in last test (metrics from previous test)
                if (metrics.oscillating) {
                    pOscillating = true;
                    pHigh = previousP; // Previous P was the last good one
                    pLow = previousP / 2.0; // One step before that
                    currentP = previousP; // Use the last good P
                    pInBinarySearch = true; // Switch to binary search
                }
            } else if (pInBinarySearch) {
                // Binary search phase: find optimal P between pLow and pHigh
                if (pHigh - pLow < 0.0005) { // Convergence threshold
                    // P tuning complete - use best value found
                    currentPhase = TuningPhase.LOADING_BALLS;
                    prepareForBallLoading();
                    return;
                }
                // Test midpoint
                currentP = (pLow + pHigh) / 2.0;
            }
            
            if (currentPhase == TuningPhase.TUNING_P) {
                startTest();
            }
        } else if (currentPhase == TuningPhase.LOADING_BALLS) {
            // This phase is handled by button press in main loop
            // When user presses A, it will call startDITuning()
        } else if (currentPhase == TuningPhase.TUNING_D) {
            // WPI: Increase K_d as much as possible without introducing jittering
            if (!dJittering && !dInBinarySearch) {
                // Exponential search phase: multiply by 2
                double previousD = currentD;
                if (currentD == 0) {
                    currentD = 0.001; // Start from small non-zero value
                } else {
                    currentD *= 2.0;
                }
                if (currentD > D_MAX) {
                    currentD = D_MAX;
                    // If we hit max without jittering, D tuning might be complete
                    if (!metrics.jittering && metrics.steadyStateError <= POSITION_TOLERANCE) {
                        currentPhase = TuningPhase.TUNING_I;
                        currentD = bestD; // Use best D found
                        currentI = I_MIN;
                    }
                }
                // Check if we found jittering in last test
                if (metrics.jittering) {
                    dJittering = true;
                    dHigh = previousD; // Previous D was the last good one
                    dLow = (previousD == 0) ? 0 : previousD / 2.0; // One step before that
                    currentD = previousD; // Use the last good D
                    dInBinarySearch = true; // Switch to binary search
                }
            } else if (dInBinarySearch) {
                // Binary search phase: find optimal D between dLow and dHigh
                if (dHigh - dLow < 0.001) { // Convergence threshold
                    // D tuning complete - move to I (if needed)
                    currentPhase = TuningPhase.TUNING_I;
                    currentD = bestD; // Use best D found
                    currentI = I_MIN;
                } else {
                    // Test midpoint
                    currentD = (dLow + dHigh) / 2.0;
                }
            }
            
            if (currentPhase == TuningPhase.TUNING_D) {
                startTest();
            } else if (currentPhase == TuningPhase.TUNING_I) {
                // I uses linear search (small range, WPI says it's rarely needed)
                // WPI Note: I tuning should only be done if steady-state error exists after P+D
                // Feedforward is preferred over integral control
                currentI += I_INCREMENT;
                if (currentI > I_MAX) {
                    finishTuning();
                    return;
                }
                startTest();
            }
        } else if (currentPhase == TuningPhase.TUNING_I) {
            // I uses linear search (small range)
            currentI += I_INCREMENT;
            if (currentI > I_MAX) {
                finishTuning();
                return;
            }
            startTest();
        }
    }

    private void skipPhase() {
        if (currentPhase == TuningPhase.TUNING_P) {
            currentPhase = TuningPhase.LOADING_BALLS;
            prepareForBallLoading();
        } else if (currentPhase == TuningPhase.TUNING_D) {
            currentPhase = TuningPhase.TUNING_I;
            startTest();
        } else if (currentPhase == TuningPhase.TUNING_I) {
            finishTuning();
            return;
        }
    }
    
    // Prepare for ball loading: freeze spindexer and reset PID state
    private void prepareForBallLoading() {
        // Stop all movement
        spindexer.setManualPower(0);
        intake.stop();
        
        // Reset PID state to prevent windup
        spindexer.setPIDCoefficients(0, 0, 0);
            // Balls inside
        spindexer.resetPIDOnly(); // what you should add
        
        // Ensure PID is disabled during loading
        spindexer.setPIDEnabled(false);
        
        testInProgress = false;
        testJustFinished = false;
    }
    
    // Start D/I tuning after balls are loaded
    private void startDITuning() {
        // Re-enable PID
        spindexer.setPIDEnabled(true);
        
        // Apply best P value, zero I and D
        spindexer.setPIDCoefficients(bestP, 0, 0);
        
        // Go to position 0 first to normalize state
        spindexer.goToPositionForCurrentMode(0);
        
        // Wait for balls to settle
        sleep(500);
        
        // Wait for position 0 to be reached
        long startWait = System.currentTimeMillis();
        while (opModeIsActive() && !spindexer.isAtPosition() && 
               (System.currentTimeMillis() - startWait) < 3000) {
            spindexer.update();
            sleep(20);
        }
        
        // Now start D tuning
        currentPhase = TuningPhase.TUNING_D;
        currentP = bestP;  // Use best P from previous phase
        currentD = D_MIN;
        currentI = 0;
        startTest();
    }

    private void resetTuning() {
        currentPhase = TuningPhase.IDLE;
        testInProgress = false;
            // Balls inside
        spindexer.resetPIDOnly(); // what you should add
        intake.stop();
    }

    private void finishTuning() {
        spindexer.setPIDCoefficients(bestP, bestI, bestD);
        currentPhase = TuningPhase.COMPLETE;
    }

    // Run the test for the current PID coefficients
    // Tests all 3 positions in sequence (0→1→2) like real TeleOp usage
    private void startTest() {
        // Use resetPIDOnly when balls are loaded (preserve encoder position)
        // Use reset() when empty (can reset encoder)
        boolean ballsLoaded = (currentPhase == TuningPhase.TUNING_D || 
                              currentPhase == TuningPhase.TUNING_I);
        if (ballsLoaded) {
            spindexer.resetPIDOnly(); // Preserve encoder position with balls
        } else {
            spindexer.reset(); // Can reset encoder when empty
        }
        sleep(200);

        // Intake control: OFF for P tuning, ON for D/I tuning (per WPI recommendations)
        // P should be tuned without load, D/I with load
        boolean intakeShouldRun = (currentPhase == TuningPhase.TUNING_D || 
                                   currentPhase == TuningPhase.TUNING_I);
        if (intakeShouldRun) {
            intake.setPower(0.9);
        } else {
            intake.setPower(0);
        }

        // Set mode (intake mode like TeleOp)
        spindexer.setIntakeMode(intakeMode);

        // Ensure PID is enabled
        spindexer.setPIDEnabled(true);
        
        // Reset PID state - ensure clean slate
        spindexer.setPIDCoefficients(currentP, currentI, currentD);
        
        // Start at position 0, will cycle through 0→1→2
        currentPositionIndex = 0;
        positionsCompleted = 0;
        spindexer.goToPositionForCurrentMode(currentPositionIndex);

        spindexerIsMoving = true;
        hasReachedTarget = false;
        hasCrossedTarget = false;
        testInProgress = true;
        testJustFinished = false;

        motionStartTime = System.currentTimeMillis();
        targetReachedTime = 0;
        settledStartTime = 0;
        metrics.reset();
        lastError = 0;
        lastErrorChange = 0;
        errorSignChanges = 0;
        rapidErrorChanges = 0;
        maxOvershoot = 0;
        steadyStateErrorSum = 0;
        steadyStateSamples = 0;
        
        // Reset aggregate metrics
        totalSettlingTime = 0;
        totalMaxOvershoot = 0;
        totalSteadyStateError = 0;
        totalOscillationCount = 0;
    }

    // Run the test to evaluate the current PID coefficients
    // Tests sequence: 0→1→2 (like TeleOp) and aggregates metrics
    // NOTE: spindexer.update() is called in main loop, NOT here
    private void runTest() {
        int current = spindexer.getCurrentPosition();
        int target = spindexer.getTargetPosition();
        double error = current - target;
        double absError = Math.abs(error);

        // Track overshoot ONLY after we've crossed the target
        // Overshoot = error after first crossing target
        if (hasCrossedTarget && hasReachedTarget) {
            maxOvershoot = Math.max(maxOvershoot, absError);
        }

        // Detect oscillations with deadband to avoid noise
        // WPI definition: oscillations = system repeatedly crosses setpoint (error sign changes)
        // Only count sign changes when error is significant (outside deadband)
        // This detects when system oscillates around the setpoint
        double deadband = POSITION_TOLERANCE * 1.5;
        if (Math.abs(error) > deadband && Math.abs(lastError) > deadband) {
            if ((error > 0 && lastError < 0) || (error < 0 && lastError > 0)) {
                errorSignChanges++;
            }
        }
        
        // Detect jittering (for D tuning) - rapid small error changes
        // WPI: Increase D until jittering appears, then reduce slightly
        // Jitter = high-frequency small oscillations near setpoint
        double errorChange = Math.abs(error - lastError);
        if (hasReachedTarget && errorChange < JITTER_THRESHOLD && errorChange > 0.1) {
            // Small rapid changes near setpoint indicate jittering
            rapidErrorChanges++;
        } else if (errorChange > JITTER_THRESHOLD) {
            // Large changes reset jitter count (normal movement)
            rapidErrorChanges = Math.max(0, rapidErrorChanges - 1);
        }
        
        lastError = error;

        long now = System.currentTimeMillis();
        long elapsed = now - motionStartTime;

        // Track when we first cross into tolerance
        if (absError <= POSITION_TOLERANCE) {
            if (settledStartTime == 0) {
                settledStartTime = now;
            }
            
            // Verify we've stayed settled for the required time
            long timeSettled = now - settledStartTime;
            if (timeSettled >= STEADY_STATE_TIME_MS && !hasReachedTarget) {
                // Now we've truly reached and settled at target
                hasReachedTarget = true;
                hasCrossedTarget = true;
                targetReachedTime = now;
                // Settling time = time to first enter tolerance + time to stay settled
                long positionSettlingTime = now - motionStartTime;
                totalSettlingTime += positionSettlingTime;
            }
        } else {
            // Lost tolerance - reset settled timer
            if (!hasReachedTarget) {
                settledStartTime = 0;
            }
            // Track if we've crossed target (for overshoot detection)
            if (!hasCrossedTarget && absError < Math.abs(lastError)) {
                // Error is decreasing - we're approaching target
                // If we were on opposite side, we've crossed
                if ((error > 0 && lastError < 0) || (error < 0 && lastError > 0)) {
                    hasCrossedTarget = true;
                }
            }
        }

        // Collect steady-state error samples after reaching target
        if (hasReachedTarget && absError <= POSITION_TOLERANCE) {
            steadyStateErrorSum += absError;
            steadyStateSamples++;
        }

        // Check if current position is complete and move to next
        if (hasReachedTarget) {
            long steadyStateElapsed = now - targetReachedTime;
            if (steadyStateElapsed >= STEADY_STATE_TIME_MS) {
                // Position complete - record metrics for this position
                if (steadyStateSamples > 0) {
                    totalSteadyStateError += steadyStateErrorSum / steadyStateSamples;
                } else {
                    totalSteadyStateError += absError;
                }
                totalMaxOvershoot = Math.max(totalMaxOvershoot, maxOvershoot);
                totalOscillationCount += errorSignChanges;
                
                positionsCompleted++;
                
                // Move to next position if not done
                if (positionsCompleted < TOTAL_POSITIONS_TO_TEST) {
                    // Reset for next position
                    currentPositionIndex = (currentPositionIndex + 1) % 3;
                    spindexer.goToPositionForCurrentMode(currentPositionIndex);
                    spindexerIsMoving = true;
                    hasReachedTarget = false;
                    hasCrossedTarget = false;
                    motionStartTime = now; // Reset timer for next position
                    targetReachedTime = 0;
                    settledStartTime = 0;
                    lastError = 0;
                    lastErrorChange = 0;
                    errorSignChanges = 0;
                    rapidErrorChanges = 0;
                    maxOvershoot = 0;
                    steadyStateErrorSum = 0;
                    steadyStateSamples = 0;
                    return; // Continue testing next position
                } else {
                    // All positions tested - calculate final metrics
                    metrics.settlingTime = totalSettlingTime / TOTAL_POSITIONS_TO_TEST; // Average
                    metrics.steadyStateError = totalSteadyStateError / TOTAL_POSITIONS_TO_TEST; // Average
                    metrics.maxOvershoot = totalMaxOvershoot;
                    metrics.oscillationCount = totalOscillationCount;
                    // WPI: Oscillations = system repeatedly crosses setpoint
                    // For 3 positions, allow up to 2-3 sign changes per position (6-9 total)
                    // More than that indicates sustained oscillation
                    metrics.oscillating = totalOscillationCount > (TOTAL_POSITIONS_TO_TEST * 3);
                    // WPI: Jittering = high-frequency small oscillations (D too high)
                    // Detect jittering from rapid error changes near setpoint
                    metrics.jittering = (rapidErrorChanges > JITTER_COUNT_THRESHOLD);
                    metrics.score = calculateScore();

                    // Update best if this is better
                    // WPI: Only accept solutions that reach setpoint, don't oscillate, and don't jitter
                    if (metrics.steadyStateError <= POSITION_TOLERANCE &&
                            !metrics.oscillating &&
                            !metrics.jittering &&
                            metrics.score < bestMetrics.score) {
                        copyMetrics(metrics, bestMetrics);
                        bestP = currentP;
                        bestD = currentD;
                        bestI = currentI;
                    }
                    
                    // Update binary search bounds based on results
                    if (currentPhase == TuningPhase.TUNING_P && pInBinarySearch) {
                        if (metrics.oscillating) {
                            pHigh = currentP; // Too high - reduce upper bound
                        } else if (metrics.steadyStateError <= POSITION_TOLERANCE) {
                            pLow = currentP; // Good - increase lower bound
                        }
                    } else if (currentPhase == TuningPhase.TUNING_D && dInBinarySearch) {
                        if (metrics.jittering) {
                            dHigh = currentD; // Too high - reduce upper bound
                        } else if (metrics.steadyStateError <= POSITION_TOLERANCE && !metrics.oscillating) {
                            dLow = currentD; // Good - increase lower bound
                        }
                    }

                    spindexerIsMoving = false;
                    testInProgress = false;
                    testJustFinished = true;
                    return;
                }
            }
        }

        // Timeout handling - evaluate even if we didn't complete all positions
        if (elapsed > MAX_TEST_TIME_MS) {
            // Calculate metrics from what we've completed so far
            if (positionsCompleted > 0) {
                metrics.settlingTime = totalSettlingTime / positionsCompleted;
                metrics.steadyStateError = totalSteadyStateError / positionsCompleted;
            } else {
                // Never reached any target - use current error
                metrics.settlingTime = elapsed;
                metrics.steadyStateError = absError;
            }
            
            metrics.maxOvershoot = totalMaxOvershoot;
            metrics.oscillationCount = totalOscillationCount;
            // WPI: Oscillations = system repeatedly crosses setpoint
            metrics.oscillating = totalOscillationCount > (positionsCompleted * 3);
            // WPI: Jittering = high-frequency small oscillations (D too high)
            metrics.jittering = (rapidErrorChanges > JITTER_COUNT_THRESHOLD);
            metrics.score = calculateScore();

            // Only update best if we completed at least one position, reached setpoint, 
            // no oscillations, no jittering, and it's better
            if (positionsCompleted > 0 && 
                    metrics.steadyStateError <= POSITION_TOLERANCE &&
                    !metrics.oscillating &&
                    !metrics.jittering &&
                    metrics.score < bestMetrics.score) {
                copyMetrics(metrics, bestMetrics);
                bestP = currentP;
                bestD = currentD;
                bestI = currentI;
            }
            
            // Update binary search bounds on timeout as well
            if (currentPhase == TuningPhase.TUNING_P && pInBinarySearch) {
                if (metrics.oscillating) {
                    pHigh = currentP;
                } else if (metrics.steadyStateError <= POSITION_TOLERANCE) {
                    pLow = currentP;
                }
            } else if (currentPhase == TuningPhase.TUNING_D && dInBinarySearch) {
                if (metrics.jittering) {
                    dHigh = currentD;
                } else if (metrics.steadyStateError <= POSITION_TOLERANCE && !metrics.oscillating) {
                    dLow = currentD;
                }
            }

            spindexerIsMoving = false;
            testInProgress = false;
            testJustFinished = true;
        }
    }

    // Improved scoring function for position control (aligned with WPI principles)
    private double calculateScore() {
        // Lower score is better
        // WPI prioritizes: no oscillations/jittering > accuracy > settling time > overshoot
        
        // Oscillations are unacceptable - reject any oscillating solution
        if (metrics.oscillating) {
            return Double.MAX_VALUE; // Reject oscillating solutions completely
        }
        
        // Jittering is also unacceptable (D too high) - reject jittering solutions
        if (metrics.jittering) {
            return Double.MAX_VALUE; // Reject jittering solutions completely
        }
        
        // For stable (non-oscillating, non-jittering) solutions, prioritize:
        // 1. Accuracy (steady-state error) - most important
        // 2. Settling time - faster is better
        // 3. Overshoot - less is better
        return metrics.steadyStateError * 200.0 +           // Most important: accuracy
                metrics.settlingTime * 0.15 +                // Speed to target
                metrics.maxOvershoot * 15.0;                 // Overshoot penalty
    }

    // Copy performance metrics
    private void copyMetrics(PerformanceMetrics src, PerformanceMetrics dst) {
        dst.settlingTime = src.settlingTime;
        dst.maxOvershoot = src.maxOvershoot;
        dst.steadyStateError = src.steadyStateError;
        dst.oscillationCount = src.oscillationCount;
        dst.oscillating = src.oscillating;
        dst.jittering = src.jittering;
        dst.score = src.score;
    }

    // ===================== TELEMETRY (GRAPHABLE) =====================
    private void updateTelemetry() {
        int currentPos = spindexer.getCurrentPosition();
        int targetPos = spindexer.getTargetPosition();
        double error = currentPos - targetPos;
        double timeSinceMove = testInProgress
                ? (System.currentTimeMillis() - motionStartTime)
                : 0;

        // ===== GRAPH SIGNALS =====
        // Graph plugin automatically detects numeric telemetry data
        telemetryM.addData("Graph_Position", currentPos);
        telemetryM.addData("Graph_Target", targetPos);
        telemetryM.addData("Graph_Error", error);
        telemetryM.addData("Graph_Time_ms", timeSinceMove);

        telemetryM.addData("Graph_P", currentP);
        telemetryM.addData("Graph_I", currentI);
        telemetryM.addData("Graph_D", currentD);

        telemetryM.addData("Phase", currentPhase);
        
        // Show exponential search progress
        if (currentPhase == TuningPhase.TUNING_P) {
            if (pInBinarySearch) {
                telemetryM.addLine("P: Binary Search (pLow=" + String.format("%.4f", pLow) + 
                                  ", pHigh=" + String.format("%.4f", pHigh) + ")");
            } else {
                telemetryM.addLine("P: Exponential Search (multiplying by 2)");
            }
        } else if (currentPhase == TuningPhase.TUNING_D) {
            if (dInBinarySearch) {
                telemetryM.addLine("D: Binary Search (dLow=" + String.format("%.4f", dLow) + 
                                  ", dHigh=" + String.format("%.4f", dHigh) + ")");
            } else {
                telemetryM.addLine("D: Exponential Search (multiplying by 2)");
            }
        }
        
        // Ball loading reminder (critical for accurate tuning)
        if (currentPhase == TuningPhase.LOADING_BALLS) {
            telemetryM.addLine("");
            telemetryM.addLine("*** LOAD 3 BALLS NOW ***");
            telemetryM.addLine("Press A when ready to continue");
            telemetryM.addLine("");
        } else if (currentPhase == TuningPhase.TUNING_P) {
            telemetryM.addLine("");
            telemetryM.addLine("*** REMOVE ALL BALLS ***");
            telemetryM.addLine("P tuning requires empty spindexer");
            telemetryM.addLine("");
        } else if (currentPhase == TuningPhase.TUNING_D || currentPhase == TuningPhase.TUNING_I) {
            telemetryM.addLine("");
            telemetryM.addLine("*** 3 BALLS LOADED ***");
            telemetryM.addLine("D/I tuning with load");
            telemetryM.addLine("");
        }
        
        telemetryM.addData("Moving", spindexerIsMoving);
        telemetryM.addData("Reached Target", hasReachedTarget);
        telemetryM.addData("Settling", spindexer.isSettling());
        telemetryM.addData("Oscillating", metrics.oscillating);
        telemetryM.addData("Jittering", metrics.jittering);
        telemetryM.addData("Position Index", currentPositionIndex);
        telemetryM.addData("Positions Completed", positionsCompleted);
        telemetryM.addData("Total Positions", TOTAL_POSITIONS_TO_TEST);
        telemetryM.addData("Mode", intakeMode ? "INTAKE" : "OUTTAKE");
        
        // Only show score if test is complete (score is calculated)
        if (!testInProgress && metrics.score < Double.MAX_VALUE) {
            telemetryM.addData("Current Score", metrics.score);
        }
        if (bestMetrics.score < Double.MAX_VALUE) {
            telemetryM.addData("Best Score", bestMetrics.score);
        }
        
        // Show intake status
        if (currentPhase == TuningPhase.LOADING_BALLS) {
            telemetryM.addData("Intake", "OFF (loading balls)");
        } else {
            telemetryM.addData("Intake", 
                (currentPhase == TuningPhase.TUNING_D || currentPhase == TuningPhase.TUNING_I) 
                    ? "ON (D/I tuning)" : "OFF (P tuning)");
        }
        
        // Show best P value when transitioning to D/I
        if (currentPhase == TuningPhase.LOADING_BALLS || 
            currentPhase == TuningPhase.TUNING_D || 
            currentPhase == TuningPhase.TUNING_I) {
            telemetryM.addData("Best P", bestP);
        }

        // Graph data - Panels Graph plugin reads numeric values from telemetry automatically
        // All values below are numeric and will be automatically detected by Graph plugin
        telemetryM.addData("Position", currentPos);
        telemetryM.addData("Target", targetPos);
        telemetryM.addData("Error", error);
        telemetryM.addData("P", currentP);
        telemetryM.addData("I", currentI);
        telemetryM.addData("D", currentD);
        telemetryM.addData("Time", timeSinceMove);
        
        // Performance metrics for graphing (when available)
        if (metrics.score < Double.MAX_VALUE) {
            telemetryM.addData("SettlingTime", metrics.settlingTime);
            telemetryM.addData("Overshoot", metrics.maxOvershoot);
            telemetryM.addData("SteadyStateError", metrics.steadyStateError);
            telemetryM.addData("Oscillations", metrics.oscillationCount);
            telemetryM.addData("Score", metrics.score);
        }
        
        // Best values for comparison graphing
        if (bestMetrics.score < Double.MAX_VALUE) {
            telemetryM.addData("BestScore", bestMetrics.score);
        }
        telemetryM.addData("BestP", bestP);
        telemetryM.addData("BestD", bestD);
        telemetryM.addData("BestI", bestI);
        
        // Position tracking for graphs
        telemetryM.addData("PositionIndex", currentPositionIndex);
        telemetryM.addData("PositionsCompleted", positionsCompleted);
        
        // Update both Panels and FTC telemetry
        telemetryM.update(telemetry);
    }
}
