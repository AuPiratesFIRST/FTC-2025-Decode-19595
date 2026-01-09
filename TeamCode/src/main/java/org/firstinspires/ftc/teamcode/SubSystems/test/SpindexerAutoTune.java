package org.firstinspires.ftc.teamcode.SubSystems.test;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.SubSystems.Spindexer.OldSpindexerSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Intake.IntakeSubsystem;

@TeleOp(name = "Spindexer Auto-Tune (Final v2.0)", group = "Test")
public class SpindexerAutoTune extends LinearOpMode {

    private OldSpindexerSubsystem spindexer;
    private IntakeSubsystem intake;
    private TelemetryManager telemetryM;

    private enum TuningPhase {
        IDLE, TUNING_P, LOADING_BALLS, TUNING_D, TUNING_I, COMPLETE
    }

    private TuningPhase currentPhase = TuningPhase.IDLE;

    private static final double STARTING_P = 0.006;
    private static final double STARTING_D = 0.0007;

    private double bestP = STARTING_P, bestD = STARTING_D, bestI = 0.0001;
    private double currentP = STARTING_P, currentD = STARTING_D, currentI = 0.0;

    private static final double P_MIN = 0.002, P_MAX = 0.015;
    private double pLow = P_MIN, pHigh = P_MAX;
    private boolean pInBinarySearch = false;

    private static final double D_MIN = 0.0001, D_MAX = 0.005;
    private double dLow = D_MIN, dHigh = D_MAX;
    private boolean dInBinarySearch = false;

    private static final double I_MIN = 0.00005, I_MAX = 0.001;
    private double iLow = I_MIN, iHigh = I_MAX;
    private boolean iInBinarySearch = false;

    private static class PerformanceMetrics {
        double settlingTime, maxOvershoot, steadyStateError, score;
        int oscillationCount;
        boolean oscillating, jittering;

        void reset() {
            settlingTime = 0; maxOvershoot = 0; steadyStateError = 0;
            oscillationCount = 0; oscillating = false; jittering = false;
            score = Double.MAX_VALUE;
        }
    }

    private final PerformanceMetrics metrics = new PerformanceMetrics();
    private final PerformanceMetrics bestMetrics = new PerformanceMetrics();

    private boolean testInProgress = false, testJustFinished = false;
    private long motionStartTime = 0;
    private static final long MAX_TEST_TIME_MS = 12000;
    private static final int POSITION_TOLERANCE = 15;

    // Fixed tracking variables
    private double lastError = 0;
    private int currentPositionRapidErrorChanges = 0;
    private double currentPositionMaxOvershoot = 0;
    private boolean hasCrossedTarget = false, hasReachedTarget = false;
    private long settledStartTime = 0;

    private int currentPositionIndex = 0, positionsCompleted = 0;
    private static final int TOTAL_POSITIONS_TO_TEST = 3;

    // Aggregators
    private double totalSettlingTime = 0, totalMaxOvershoot = 0, totalSteadyStateError = 0;
    private int totalOscillationCount = 0;
    private int totalRapidErrorChanges = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        spindexer = new OldSpindexerSubsystem(hardwareMap, telemetry);
        intake = new IntakeSubsystem(hardwareMap, telemetry);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        telemetryM.addLine("Spindexer ACTIVE HOLD Tuner v2.0");
        telemetryM.addLine("Intake Disabled | Timeout Penalties Enabled");
        telemetryM.addLine("Press A to Start");
        telemetryM.update(telemetry);

        waitForStart();

        while (opModeIsActive()) {
            handleInputs();
            if (testInProgress) runTest();
            else if (testJustFinished) {
                testJustFinished = false;
                continueTuning();
            }
            spindexer.update();
            updateTelemetry();
            sleep(20);
        }
    }

    private void handleInputs() {
        if (gamepad1.a && !testInProgress) {
            if (currentPhase == TuningPhase.IDLE) startTuning();
            else if (currentPhase == TuningPhase.LOADING_BALLS) startDITuning();
        }
        if (gamepad1.x) resetTuning();
    }

    private void startTuning() {
        currentPhase = TuningPhase.TUNING_P;
        currentP = STARTING_P; currentD = 0; currentI = 0;
        pLow = P_MIN; pHigh = P_MAX; pInBinarySearch = false;
        bestMetrics.reset();
        startTest();
    }

    private void continueTuning() {
        if (currentPhase == TuningPhase.TUNING_P) handleBinarySearchP();
        else if (currentPhase == TuningPhase.TUNING_D) handleBinarySearchD();
        else if (currentPhase == TuningPhase.TUNING_I) handleBinarySearchI();
    }

    private void handleBinarySearchP() {
        if (!pInBinarySearch) {
            if (metrics.oscillating) {
                pInBinarySearch = true; pHigh = currentP; pLow = P_MIN;
            } else {
                currentP += 0.001;
                if (currentP > P_MAX) pInBinarySearch = true;
            }
        } else {
            if (metrics.oscillating) pHigh = currentP; else pLow = currentP;
            currentP = (pLow + pHigh) / 2.0;
            if (pHigh - pLow < 0.0002) {
                bestP = (pLow + pHigh) / 2.0;
                startDITuning();
                return;
            }
        }
        startTest();
    }

    private void handleBinarySearchD() {
        if (!dInBinarySearch) {
            if (metrics.jittering || metrics.oscillating) {
                dInBinarySearch = true; dHigh = currentD; dLow = D_MIN;
            } else {
                currentD += 0.0005;
                if (currentD > D_MAX) dInBinarySearch = true;
            }
        } else {
            if (metrics.jittering || metrics.oscillating) dHigh = currentD; else dLow = currentD;
            currentD = (dLow + dHigh) / 2.0;
            if (dHigh - dLow < 0.0001) {
                bestD = (dLow + dHigh) / 2.0;
                currentPhase = TuningPhase.TUNING_I;
            }
        }
        startTest();
    }

    private void handleBinarySearchI() {
        // Integral Hold "Good Enough" Exit
        if (metrics.steadyStateError < 2.0) {
            bestI = currentI; finishTuning(); return;
        }

        if (!iInBinarySearch) {
            if (metrics.oscillating) {
                iInBinarySearch = true; iHigh = currentI; iLow = I_MIN;
            } else {
                currentI += 0.00005;
                if (currentI > I_MAX) iInBinarySearch = true;
            }
        } else {
            if (metrics.oscillating) iHigh = currentI; else iLow = currentI;
            currentI = (iLow + iHigh) / 2.0;
            if (iHigh - iLow < 0.00001) {
                bestI = (iLow + iHigh) / 2.0;
                finishTuning();
                return;
            }
        }
        startTest();
    }

    private void startDITuning() {
        currentPhase = TuningPhase.TUNING_D;
        currentD = STARTING_D; dLow = D_MIN; dHigh = D_MAX;
        currentI = 0.0001; iLow = I_MIN; iHigh = I_MAX;
        startTest();
    }

    private void startTest() {
        spindexer.setTuningMode(true);
        spindexer.resetPIDOnly();
        spindexer.setPIDCoefficients(currentP, currentI, currentD);
        intake.stop();

        currentPositionIndex = 0; positionsCompleted = 0;
        totalSettlingTime = 0; totalMaxOvershoot = 0; totalOscillationCount = 0; 
        totalSteadyStateError = 0; totalRapidErrorChanges = 0;

        prepareNewPosition();
        testInProgress = true;
    }

    private void prepareNewPosition() {
        spindexer.goToPositionForCurrentMode(currentPositionIndex);
        motionStartTime = System.currentTimeMillis();
        hasReachedTarget = false; hasCrossedTarget = false;
        currentPositionRapidErrorChanges = 0; currentPositionMaxOvershoot = 0; settledStartTime = 0; 
        lastError = spindexer.shortestError(spindexer.getTargetPosition(), spindexer.getCurrentPosition());
    }

    private void runTest() {
        double error = spindexer.shortestError(spindexer.getTargetPosition(), spindexer.getCurrentPosition());
        double absError = Math.abs(error);
        double prevError = lastError;

        if (hasCrossedTarget) currentPositionMaxOvershoot = Math.max(currentPositionMaxOvershoot, absError);

        // Sign change detection (Cache prevError for consistency)
        if ((error > 0 && prevError < 0) || (error < 0 && prevError > 0)) {
            if (Math.abs(error) > 5) { // Noise gate
                totalOscillationCount++;
                hasCrossedTarget = true;
            }
        }

        if (hasReachedTarget && Math.abs(error - prevError) > 3.0) currentPositionRapidErrorChanges++;

        lastError = error;
        long now = System.currentTimeMillis();
        long steadyWait = (currentPhase == TuningPhase.TUNING_I) ? 800 : 400;

        if (absError <= POSITION_TOLERANCE) {
            if (settledStartTime == 0) settledStartTime = now;
            if (now - settledStartTime > steadyWait) hasReachedTarget = true;
        } else {
            settledStartTime = 0;
        }

        // Exit position logic with Timeout Penalty
        if (hasReachedTarget || (now - motionStartTime > MAX_TEST_TIME_MS / 3)) {
            if (!hasReachedTarget) totalSettlingTime += (MAX_TEST_TIME_MS / 3) * 1.5; // Penalty
            else totalSettlingTime += (now - motionStartTime);

            totalMaxOvershoot += currentPositionMaxOvershoot;
            totalSteadyStateError += absError;
            totalRapidErrorChanges += currentPositionRapidErrorChanges;
            
            positionsCompleted++;
            if (positionsCompleted < TOTAL_POSITIONS_TO_TEST) {
                currentPositionIndex = (currentPositionIndex + 1) % 3;
                prepareNewPosition();
            } else {
                finalizeMetrics();
            }
        }
    }

    private void finalizeMetrics() {
        // Frequency check: detect real chatter vs one-time overshoot
        double oscPerSecond = totalOscillationCount / (totalSettlingTime / 1000.0 + 0.001);
        metrics.oscillating = oscPerSecond > 2.5;
        
        metrics.jittering = totalRapidErrorChanges > 20;
        metrics.steadyStateError = totalSteadyStateError / TOTAL_POSITIONS_TO_TEST;
        metrics.maxOvershoot = totalMaxOvershoot / TOTAL_POSITIONS_TO_TEST;
        metrics.score = (totalSettlingTime / 1000.0) + (metrics.maxOvershoot * 0.25) + 
                        (metrics.steadyStateError * 0.75) + (totalOscillationCount * 0.5);

        if (!metrics.oscillating && !metrics.jittering && metrics.score < bestMetrics.score) {
            bestMetrics.score = metrics.score; 
            bestP = currentP; bestD = currentD; bestI = currentI;
        }
        testInProgress = false; testJustFinished = true;
    }

    private void resetTuning() {
        currentPhase = TuningPhase.IDLE; testInProgress = false; intake.stop();
        spindexer.setTuningMode(false);
    }

    private void finishTuning() {
        currentPhase = TuningPhase.COMPLETE;
        spindexer.setPIDCoefficients(bestP, bestI, bestD);
        spindexer.setTuningMode(false);
        intake.stop();
    }


    private void updateTelemetry() {
        telemetryM.addData("PHASE", currentPhase);
        telemetryM.addData("Target Position", spindexer.getTargetPosition());
        telemetryM.addData("Current Position", spindexer.getCurrentPosition());

        // Use String.format to bypass TelemetryManager format limitations
        telemetryM.addData("Current P", String.format("%.5f", currentP));
        telemetryM.addData("Current I", String.format("%.6f", currentI));
        telemetryM.addData("Current D", String.format("%.5f", currentD));

        double scoreToShow = (bestMetrics.score == Double.MAX_VALUE) ? 0 : bestMetrics.score;
        telemetryM.addData("Best Score", String.format("%.3f", scoreToShow));
        telemetryM.update(telemetry);
    }
}