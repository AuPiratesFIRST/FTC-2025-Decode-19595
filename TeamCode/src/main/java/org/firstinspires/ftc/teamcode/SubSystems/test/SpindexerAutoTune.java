package org.firstinspires.ftc.teamcode.SubSystems.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SubSystems.Spindexer.SpindexerSubsystem;

/**
 * Automatic PID tuning test for Spindexer subsystem.
 * Systematically tunes P, D, and I coefficients by testing different values
 * and measuring performance metrics.
 * 
 * Tuning Process:
 * 1. Tune P: Tests P values from 0.001 to 0.1, finds highest P without excessive oscillation
 * 2. Tune D: Tests D values to dampen oscillation found in step 1
 * 3. Tune I: Tests I values if steady-state error exists
 * 
 * Metrics Measured:
 * - Settling Time: Time to reach and stay within tolerance
 * - Overshoot: Maximum error beyond target
 * - Steady-State Error: Final error after settling
 * - Oscillation: Number of error sign changes
 * 
 * Controls:
 * - A Button: Start auto-tuning (runs automatically after start)
 * - B Button: Skip current tuning phase
 * - X Button: Reset and restart tuning
 * - Y Button: Stop tuning and use current values
 */
@TeleOp(name = "Spindexer Auto-Tune", group = "Test")
public class SpindexerAutoTune extends LinearOpMode {

    private SpindexerSubsystem spindexer;
    
    // Tuning state
    private enum TuningPhase {
        IDLE,
        TUNING_P,
        TUNING_D,
        TUNING_I,
        COMPLETE
    }
    
    private TuningPhase currentPhase = TuningPhase.IDLE;
    
    // Tuning parameters
    private double bestP = 0.01;
    private double bestD = 0.02;
    private double bestI = 0.0;
    
    private double currentP = 0.001;
    private double currentD = 0.0;
    private double currentI = 0.0;
    
    // Test parameters
    private static final double P_MIN = 0.001;
    private static final double P_MAX = 0.1;
    private static final double P_INCREMENT = 0.002;
    
    private static final double D_MIN = 0.0;
    private static final double D_MAX = 0.2;
    private static final double D_INCREMENT = 0.005;
    
    private static final double I_MIN = 0.0;
    private static final double I_MAX = 0.01;
    private static final double I_INCREMENT = 0.0002;
    
    // Performance metrics
    private static class PerformanceMetrics {
        double settlingTime = 0; // milliseconds
        double maxOvershoot = 0; // ticks
        double steadyStateError = 0; // ticks
        int oscillationCount = 0; // number of error sign changes
        boolean oscillating = false;
        double bestScore = Double.MAX_VALUE; // lower is better
        
        void reset() {
            settlingTime = 0;
            maxOvershoot = 0;
            steadyStateError = 0;
            oscillationCount = 0;
            oscillating = false;
            bestScore = Double.MAX_VALUE;
        }
    }
    
    private PerformanceMetrics metrics = new PerformanceMetrics();
    private PerformanceMetrics bestMetrics = new PerformanceMetrics();
    
    // Test state
    private long testStartTime = 0;
    private int startPosition = 0;
    private int targetPosition = 0;
    private boolean testInProgress = false;
    private boolean atTargetReached = false;
    private long timeAtTarget = 0;
    private boolean testJustFinished = false; // Flag to trigger auto-continue
    private static final long SETTLING_TIME_MS = 500; // Must stay at target for this long
    private static final long MAX_TEST_TIME_MS = 5000; // Maximum time per test
    
    // Position history for oscillation detection
    private double lastError = 0;
    private int errorSignChanges = 0;
    private double maxError = 0;
    private double minError = 0;
    private boolean crossedTarget = false;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initializing Auto-Tune...");
        telemetry.update();

        // Initialize spindexer subsystem
        spindexer = new SpindexerSubsystem(hardwareMap, telemetry);
        
        // Reset to known position
        spindexer.reset();
        sleep(500);
        startPosition = spindexer.getCurrentPosition();
        
        // Set initial target (test position 2, which is 120 degrees away)
        targetPosition = SpindexerSubsystem.SpindexerPosition.POSITION_2.getTicks();

        telemetry.addData("Status", "Ready for Auto-Tuning");
        telemetry.addLine();
        telemetry.addLine("Controls:");
        telemetry.addLine("A: Start tuning (runs automatically)");
        telemetry.addLine("B: Skip current phase");
        telemetry.addLine("X: Reset and restart");
        telemetry.addLine("Y: Stop and use current values");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Handle button controls
            if (gamepad1.a && currentPhase == TuningPhase.IDLE) {
                startTuning();
                sleep(300);
            }
            
            if (gamepad1.b && !testInProgress && currentPhase != TuningPhase.IDLE && currentPhase != TuningPhase.COMPLETE) {
                skipPhase();
                // Auto-start next phase if skipping
                if (currentPhase != TuningPhase.COMPLETE) {
                    startTest();
                }
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

            // Run test if in progress
            if (testInProgress) {
                runTest();
            } else if (testJustFinished && currentPhase != TuningPhase.IDLE && currentPhase != TuningPhase.COMPLETE) {
                // Test just finished - automatically continue to next test
                testJustFinished = false;
                continueTuning();
            } else if (currentPhase == TuningPhase.IDLE || currentPhase == TuningPhase.COMPLETE) {
                // Update PID control when idle or complete
                spindexer.update();
            }

            // Update telemetry
            updateTelemetry();

            sleep(20);
        }

        // Stop spindexer when OpMode ends
        spindexer.setManualPower(0);
    }
    
    private void startTuning() {
        currentPhase = TuningPhase.TUNING_P;
        currentP = P_MIN;
        currentD = 0.0;
        currentI = 0.0;
        bestP = P_MIN;
        bestD = 0.0;
        bestI = 0.0;
        metrics.reset();
        bestMetrics.reset();
        startTest();
    }
    
    private void continueTuning() {
        // Only continue if not in progress and not complete
        if (testInProgress || currentPhase == TuningPhase.COMPLETE || currentPhase == TuningPhase.IDLE) {
            return;
        }
        
        if (currentPhase == TuningPhase.TUNING_P) {
            // Move to next P value
            currentP += P_INCREMENT;
            if (currentP > P_MAX) {
                // Finished tuning P, move to D
                currentPhase = TuningPhase.TUNING_D;
                currentP = bestP;
                currentD = D_MIN;
                telemetry.addData("Phase Complete", "P tuning finished. Best P: %.4f", bestP);
                // Auto-start D tuning
                startTest();
            } else {
                startTest();
            }
        } else if (currentPhase == TuningPhase.TUNING_D) {
            // Move to next D value
            currentD += D_INCREMENT;
            if (currentD > D_MAX) {
                // Finished tuning D, move to I
                currentPhase = TuningPhase.TUNING_I;
                currentD = bestD;
                currentI = I_MIN;
                telemetry.addData("Phase Complete", "D tuning finished. Best D: %.4f", bestD);
                // Auto-start I tuning
                startTest();
            } else {
                startTest();
            }
        } else if (currentPhase == TuningPhase.TUNING_I) {
            // Move to next I value
            currentI += I_INCREMENT;
            if (currentI > I_MAX) {
                // Finished all tuning
                currentPhase = TuningPhase.COMPLETE;
                currentI = bestI;
                // Apply best values automatically
                spindexer.setPIDCoefficients(bestP, bestI, bestD);
                telemetry.addData("Tuning Complete", "All phases finished!");
            } else {
                startTest();
            }
        }
    }
    
    private void skipPhase() {
        if (currentPhase == TuningPhase.TUNING_P) {
            currentPhase = TuningPhase.TUNING_D;
            currentP = bestP;
            currentD = D_MIN;
            startTest();
        } else if (currentPhase == TuningPhase.TUNING_D) {
            currentPhase = TuningPhase.TUNING_I;
            currentD = bestD;
            currentI = I_MIN;
            startTest();
        } else if (currentPhase == TuningPhase.TUNING_I) {
            currentPhase = TuningPhase.COMPLETE;
            currentI = bestI;
            spindexer.setPIDCoefficients(bestP, bestI, bestD);
        }
    }
    
    private void resetTuning() {
        currentPhase = TuningPhase.IDLE;
        testInProgress = false;
        testJustFinished = false;
        spindexer.reset();
        sleep(500);
        startPosition = spindexer.getCurrentPosition();
        metrics.reset();
        bestMetrics.reset();
    }
    
    private void finishTuning() {
        spindexer.setPIDCoefficients(bestP, bestI, bestD);
        currentPhase = TuningPhase.COMPLETE;
        telemetry.addData("Final Values", "P: %.4f, I: %.4f, D: %.4f", bestP, bestI, bestD);
    }
    
    private void startTest() {
        // Reset spindexer
        spindexer.reset();
        sleep(300);
        
        // Set PID coefficients
        spindexer.setPIDCoefficients(currentP, currentI, currentD);
        
        // Set target position
        spindexer.goToPosition(SpindexerSubsystem.SpindexerPosition.POSITION_2);
        
        // Reset test metrics
        metrics.reset();
        testStartTime = System.currentTimeMillis();
        atTargetReached = false;
        timeAtTarget = 0;
        startPosition = spindexer.getCurrentPosition();
        lastError = Math.abs(spindexer.getTargetPosition() - spindexer.getCurrentPosition());
        errorSignChanges = 0;
        maxError = lastError;
        minError = lastError;
        crossedTarget = false;
        testJustFinished = false;
        
        testInProgress = true;
    }
    
    private void runTest() {
        long currentTime = System.currentTimeMillis();
        long elapsedTime = currentTime - testStartTime;
        
        // Update PID control
        spindexer.update();
        
        // Get current error
        int currentPos = spindexer.getCurrentPosition();
        int targetPos = spindexer.getTargetPosition();
        double error = targetPos - currentPos;
        double absError = Math.abs(error);
        
        // Track error bounds
        if (absError > maxError) {
            maxError = absError;
        }
        if (absError < minError) {
            minError = absError;
        }
        
        // Detect oscillation (error sign changes)
        if ((error > 0 && lastError < 0) || (error < 0 && lastError > 0)) {
            errorSignChanges++;
            crossedTarget = true;
        }
        lastError = error;
        
        // Check if at target
        boolean atTarget = spindexer.isAtPosition();
        
        if (atTarget) {
            if (!atTargetReached) {
                atTargetReached = true;
                timeAtTarget = currentTime;
            }
            
            // Check if settled (stayed at target for SETTLING_TIME_MS)
            if (currentTime - timeAtTarget >= SETTLING_TIME_MS) {
                // Test complete - calculate metrics
                metrics.settlingTime = elapsedTime;
                metrics.steadyStateError = absError;
                metrics.maxOvershoot = maxError - Math.abs(targetPos - startPosition);
                metrics.oscillationCount = errorSignChanges;
                metrics.oscillating = errorSignChanges > 3; // More than 3 sign changes = oscillating
                
                // Calculate score (lower is better)
                // Penalize: settling time, overshoot, steady-state error, oscillation
                metrics.bestScore = metrics.settlingTime * 0.3 +
                                   metrics.maxOvershoot * 10.0 +
                                   metrics.steadyStateError * 5.0 +
                                   (metrics.oscillating ? 1000 : 0);
                
                // Check if this is the best so far
                if (metrics.bestScore < bestMetrics.bestScore || bestMetrics.bestScore == Double.MAX_VALUE) {
                    bestMetrics = new PerformanceMetrics();
                    bestMetrics.settlingTime = metrics.settlingTime;
                    bestMetrics.maxOvershoot = metrics.maxOvershoot;
                    bestMetrics.steadyStateError = metrics.steadyStateError;
                    bestMetrics.oscillationCount = metrics.oscillationCount;
                    bestMetrics.oscillating = metrics.oscillating;
                    bestMetrics.bestScore = metrics.bestScore;
                    
                    // Save best coefficients
                    if (currentPhase == TuningPhase.TUNING_P) {
                        bestP = currentP;
                    } else if (currentPhase == TuningPhase.TUNING_D) {
                        bestD = currentD;
                    } else if (currentPhase == TuningPhase.TUNING_I) {
                        bestI = currentI;
                    }
                }
                
                testInProgress = false;
                testJustFinished = true;
                
                // Small delay before next test
                sleep(500);
            }
        } else {
            atTargetReached = false;
        }
        
        // Timeout check
        if (elapsedTime > MAX_TEST_TIME_MS) {
            // Test timed out - mark as failed
            metrics.settlingTime = MAX_TEST_TIME_MS;
            metrics.steadyStateError = absError;
            metrics.maxOvershoot = maxError;
            metrics.oscillationCount = errorSignChanges;
            metrics.oscillating = errorSignChanges > 3;
            metrics.bestScore = Double.MAX_VALUE; // Very bad score
            
            testInProgress = false;
            testJustFinished = true;
            sleep(500);
        }
    }
    
    private void updateTelemetry() {
        telemetry.clearAll();
        
        // Phase information
        telemetry.addData("=== AUTO-TUNE STATUS ===", "");
        telemetry.addData("Current Phase", currentPhase.toString());
        telemetry.addData("Test In Progress", testInProgress ? "YES" : "NO");
        telemetry.addLine();
        
        // Current PID values being tested
        telemetry.addData("=== CURRENT TEST VALUES ===", "");
        telemetry.addData("Testing P", "%.4f", currentP);
        telemetry.addData("Testing I", "%.4f", currentI);
        telemetry.addData("Testing D", "%.4f", currentD);
        telemetry.addLine();
        
        // Best values found so far
        telemetry.addData("=== BEST VALUES FOUND ===", "");
        telemetry.addData("Best P", "%.4f", bestP);
        telemetry.addData("Best I", "%.4f", bestI);
        telemetry.addData("Best D", "%.4f", bestD);
        telemetry.addLine();
        
        // Current test metrics
        if (testInProgress) {
            long elapsed = System.currentTimeMillis() - testStartTime;
            telemetry.addData("=== CURRENT TEST METRICS ===", "");
            telemetry.addData("Elapsed Time", "%d ms", elapsed);
            telemetry.addData("Current Error", "%.1f ticks", 
                (double) Math.abs(spindexer.getTargetPosition() - spindexer.getCurrentPosition()));
            telemetry.addData("At Target", spindexer.isAtPosition() ? "YES" : "NO");
            telemetry.addData("Max Error", "%.1f ticks", maxError);
            telemetry.addData("Error Sign Changes", errorSignChanges);
            telemetry.addLine();
        } else if (metrics.bestScore != Double.MAX_VALUE) {
            telemetry.addData("=== LAST TEST RESULTS ===", "");
            telemetry.addData("Settling Time", "%.0f ms", metrics.settlingTime);
            telemetry.addData("Max Overshoot", "%.1f ticks", metrics.maxOvershoot);
            telemetry.addData("Steady-State Error", "%.1f ticks", metrics.steadyStateError);
            telemetry.addData("Oscillation Count", metrics.oscillationCount);
            telemetry.addData("Oscillating", metrics.oscillating ? "YES" : "NO");
            telemetry.addData("Score", "%.1f (lower is better)", metrics.bestScore);
            telemetry.addLine();
        }
        
        // Best metrics
        if (bestMetrics.bestScore != Double.MAX_VALUE) {
            telemetry.addData("=== BEST TEST RESULTS ===", "");
            telemetry.addData("Best Settling Time", "%.0f ms", bestMetrics.settlingTime);
            telemetry.addData("Best Max Overshoot", "%.1f ticks", bestMetrics.maxOvershoot);
            telemetry.addData("Best Steady-State Error", "%.1f ticks", bestMetrics.steadyStateError);
            telemetry.addData("Best Oscillation Count", bestMetrics.oscillationCount);
            telemetry.addData("Best Oscillating", bestMetrics.oscillating ? "YES" : "NO");
            telemetry.addData("Best Score", "%.1f", bestMetrics.bestScore);
            telemetry.addLine();
        }
        
        // Spindexer status
        telemetry.addData("=== SPINDEXER STATUS ===", "");
        spindexer.updateTelemetry();
        telemetry.addLine();
        
        // Progress information
        if (currentPhase == TuningPhase.TUNING_P) {
            double progress = ((currentP - P_MIN) / (P_MAX - P_MIN)) * 100.0;
            telemetry.addData("P Tuning Progress", "%.1f%%", Math.min(100, progress));
        } else if (currentPhase == TuningPhase.TUNING_D) {
            double progress = ((currentD - D_MIN) / (D_MAX - D_MIN)) * 100.0;
            telemetry.addData("D Tuning Progress", "%.1f%%", Math.min(100, progress));
        } else if (currentPhase == TuningPhase.TUNING_I) {
            double progress = ((currentI - I_MIN) / (I_MAX - I_MIN)) * 100.0;
            telemetry.addData("I Tuning Progress", "%.1f%%", Math.min(100, progress));
        } else if (currentPhase == TuningPhase.COMPLETE) {
            telemetry.addData("=== TUNING COMPLETE ===", "");
            telemetry.addData("Final P", "%.4f", bestP);
            telemetry.addData("Final I", "%.4f", bestI);
            telemetry.addData("Final D", "%.4f", bestD);
            telemetry.addLine();
            telemetry.addData("Best values have been applied!", "");
            telemetry.addData("Press Y to confirm or X to restart", "");
        }
        
        telemetry.update();
    }
}

