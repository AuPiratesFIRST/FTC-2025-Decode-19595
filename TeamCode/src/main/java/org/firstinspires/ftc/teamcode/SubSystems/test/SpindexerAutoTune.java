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
 * - Tuning order: P first, then D, then I (per WPI turret tuning)
 * - Aggregates metrics across all positions for comprehensive evaluation
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

    private static final double P_MIN = 0.001;
    private static final double P_MAX = 0.05;
    private static final double P_INCREMENT = 0.001;

    private static final double D_MIN = 0.0;
    private static final double D_MAX = 0.15;
    private static final double D_INCREMENT = 0.003;

    private static final double I_MIN = 0.0;
    private static final double I_MAX = 0.005;
    private static final double I_INCREMENT = 0.0001;

    private static class PerformanceMetrics {
        double settlingTime;
        double maxOvershoot;
        double steadyStateError;
        int oscillationCount;
        boolean oscillating;
        double score;

        void reset() {
            settlingTime = 0;
            maxOvershoot = 0;
            steadyStateError = 0;
            oscillationCount = 0;
            oscillating = false;
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
        telemetry.clear();
        telemetry.addLine("=== MANUAL ALIGNMENT ===");
        telemetry.addLine("Please move spindexer to physical zero (0) position.");
        telemetry.addLine("This should be the starting position (position 0).");
        telemetry.addLine("");
        telemetry.addLine("Press A when spindexer is at physical zero...");
        telemetry.update();
        
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
        telemetry.clear();
        telemetry.addLine("Resetting encoder to software zero...");
        telemetry.update();
        
        spindexer.reset();  // Resets encoder to define software zero
        sleep(500);
        
        // Step 3: Verify encoder is near zero (optional safety check)
        int currentPos = spindexer.getCurrentPosition();
        if (Math.abs(currentPos) > 5) {  // Allow small tolerance
            telemetry.clear();
            telemetry.addLine("⚠️ WARNING: Encoder not zeroed correctly!");
            telemetry.addData("Encoder reads", currentPos);
            telemetry.addLine("Expected: ~0");
            telemetry.addLine("This may indicate mechanical misalignment.");
            telemetry.addLine("");
            telemetry.addLine("Press A to continue anyway, or X to reset.");
            telemetry.update();
            
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
        telemetry.clear();
        telemetry.addLine("Spindexer Auto PID Tuner");
        telemetry.addLine("Encoder zeroed: " + spindexer.getCurrentPosition());
        telemetry.addLine("");
        telemetry.addLine("A = Start");
        telemetry.addLine("B = Skip Phase");
        telemetry.addLine("X = Reset");
        telemetry.addLine("Y = Finish");
        telemetry.update();

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
    private void startTuning() {
        currentPhase = TuningPhase.TUNING_P;
        currentP = P_MIN;
        currentD = 0;
        currentI = 0;
        bestMetrics.reset();
        startTest();
    }

    // Continue tuning based on current phase
    private void continueTuning() {
        if (currentPhase == TuningPhase.TUNING_P) {
            currentP += P_INCREMENT;
            if (currentP > P_MAX) {
                // P tuning complete - transition to loading balls phase
                currentPhase = TuningPhase.LOADING_BALLS;
                prepareForBallLoading();
            } else {
                startTest();
            }
        } else if (currentPhase == TuningPhase.LOADING_BALLS) {
            // This phase is handled by button press in main loop
            // When user presses A, it will call startDITuning()
        } else if (currentPhase == TuningPhase.TUNING_D) {
            currentD += D_INCREMENT;
            if (currentD > D_MAX) {
                currentPhase = TuningPhase.TUNING_I;
                currentD = bestD;
                currentI = I_MIN;
            }
            startTest();
        } else if (currentPhase == TuningPhase.TUNING_I) {
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
        errorSignChanges = 0;
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
        // Only count sign changes when error is significant
        double deadband = POSITION_TOLERANCE * 1.5;
        if (Math.abs(error) > deadband && Math.abs(lastError) > deadband) {
            if ((error > 0 && lastError < 0) || (error < 0 && lastError > 0)) {
                errorSignChanges++;
            }
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
                    errorSignChanges = 0;
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
                    metrics.oscillating = totalOscillationCount > (TOTAL_POSITIONS_TO_TEST * 4); // More lenient for multiple positions
                    metrics.score = calculateScore();

                    // Update best if this is better
                    if (metrics.steadyStateError <= POSITION_TOLERANCE &&
                            metrics.score < bestMetrics.score) {
                        copyMetrics(metrics, bestMetrics);
                        bestP = currentP;
                        bestD = currentD;
                        bestI = currentI;
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
            metrics.oscillating = totalOscillationCount > (positionsCompleted * 4);
            metrics.score = calculateScore();

            // Only update best if we completed at least one position and it's better
            if (positionsCompleted > 0 && metrics.steadyStateError <= POSITION_TOLERANCE &&
                    metrics.score < bestMetrics.score) {
                copyMetrics(metrics, bestMetrics);
                bestP = currentP;
                bestD = currentD;
                bestI = currentI;
            }

            spindexerIsMoving = false;
            testInProgress = false;
            testJustFinished = true;
        }
    }

    // Improved scoring function for position control
    private double calculateScore() {
        // Lower score is better
        // Prioritize: steady-state error > settling time > overshoot > oscillations
        return metrics.steadyStateError * 200.0 +           // Most important: accuracy
                metrics.settlingTime * 0.15 +                // Speed to target
                metrics.maxOvershoot * 15.0 +                // Overshoot penalty
                (metrics.oscillating ? 3000.0 : 0.0);        // Heavy penalty for oscillations
    }

    // Copy performance metrics
    private void copyMetrics(PerformanceMetrics src, PerformanceMetrics dst) {
        dst.settlingTime = src.settlingTime;
        dst.maxOvershoot = src.maxOvershoot;
        dst.steadyStateError = src.steadyStateError;
        dst.oscillationCount = src.oscillationCount;
        dst.oscillating = src.oscillating;
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
        telemetryM.addData("Position Index", currentPositionIndex);
        telemetryM.addData("Positions Completed", String.format("%d/%d", positionsCompleted, TOTAL_POSITIONS_TO_TEST));
        telemetryM.addData("Mode", intakeMode ? "INTAKE" : "OUTTAKE");
        
        // Only show score if test is complete (score is calculated)
        if (!testInProgress && metrics.score < Double.MAX_VALUE) {
            telemetryM.addData("Current Score", String.format("%.2f", metrics.score));
        }
        if (bestMetrics.score < Double.MAX_VALUE) {
            telemetryM.addData("Best Score", String.format("%.2f", bestMetrics.score));
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
            telemetryM.addData("Best P", String.format("%.4f", bestP));
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
