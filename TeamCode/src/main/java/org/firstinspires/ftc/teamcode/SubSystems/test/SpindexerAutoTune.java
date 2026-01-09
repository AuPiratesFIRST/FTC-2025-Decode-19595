package org.firstinspires.ftc.teamcode.SubSystems.test;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.SubSystems.Spindexer.OldSpindexerSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Intake.IntakeSubsystem;

@TeleOp(name = "Spindexer Auto-Tune", group = "Test")
public class SpindexerAutoTune extends LinearOpMode {

    private OldSpindexerSubsystem spindexer;
    private IntakeSubsystem intake;
    private TelemetryManager telemetryM;

    private enum TuningPhase {
        IDLE, TUNING_P, LOADING_BALLS, TUNING_D, TUNING_I, COMPLETE
    }

    private TuningPhase currentPhase = TuningPhase.IDLE;

    private static final double STARTING_P = 0.006;
    private static final double STARTING_D = 0.007;

    private double bestP = STARTING_P, bestD = STARTING_D, bestI = 0.0;
    private double currentP = STARTING_P, currentD = STARTING_D, currentI = 0.0;

    private static final double P_MIN = 0.002, P_MAX = 0.015;
    private double pLow = P_MIN, pHigh = P_MAX;
    private boolean pInBinarySearch = false;

    private static final double D_MIN = 0.001, D_MAX = 0.020;
    private double dLow = D_MIN, dHigh = D_MAX;
    private boolean dInBinarySearch = false;

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
    private static final long STEADY_STATE_TIME_MS = 400;
    private static final int POSITION_TOLERANCE = 15;

    private double lastError = 0;
    private int errorSignChanges = 0, rapidErrorChanges = 0;
    private boolean hasCrossedTarget = false, hasReachedTarget = false;
    private long settledStartTime = 0;

    private int currentPositionIndex = 0, positionsCompleted = 0;
    private static final int TOTAL_POSITIONS_TO_TEST = 3;

    private double totalSettlingTime = 0, totalMaxOvershoot = 0, totalSteadyStateError = 0;
    private int totalOscillationCount = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        spindexer = new OldSpindexerSubsystem(hardwareMap, telemetry);
        intake = new IntakeSubsystem(hardwareMap, telemetry);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        telemetryM.addLine("Spindexer PID Auto-Tuner");
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
        currentP = STARTING_P; currentD = 0;
        pLow = P_MIN; pHigh = P_MAX; pInBinarySearch = false;
        bestMetrics.reset();
        startTest();
    }

    private void continueTuning() {
        if (currentPhase == TuningPhase.TUNING_P) {
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
                    bestP = pLow; currentPhase = TuningPhase.LOADING_BALLS;
                    return;
                }
            }
            startTest();
        } else if (currentPhase == TuningPhase.TUNING_D) {
            if (!dInBinarySearch) {
                if (metrics.jittering) {
                    dInBinarySearch = true; dHigh = currentD; dLow = D_MIN;
                } else {
                    currentD += 0.001;
                    if (currentD > D_MAX) dInBinarySearch = true;
                }
            } else {
                if (metrics.jittering) dHigh = currentD; else dLow = currentD;
                currentD = (dLow + dHigh) / 2.0;
                if (dHigh - dLow < 0.0002) {
                    bestD = dLow; currentPhase = TuningPhase.COMPLETE;
                    return;
                }
            }
            startTest();
        }
    }

    private void startDITuning() {
        currentPhase = TuningPhase.TUNING_D;
        currentD = STARTING_D; dLow = D_MIN; dHigh = D_MAX;
        startTest();
    }

    private void startTest() {
        spindexer.setTuningMode(true);
        spindexer.resetPIDOnly();
        spindexer.setPIDCoefficients(currentP, currentD);

        if (currentPhase == TuningPhase.TUNING_D) intake.setPower(0.9);
        else intake.stop();

        currentPositionIndex = 0; positionsCompleted = 0;
        totalSettlingTime = 0; totalMaxOvershoot = 0; totalOscillationCount = 0;

        prepareNewPosition();
        testInProgress = true;
    }

    private void prepareNewPosition() {
        spindexer.goToPositionForCurrentMode(currentPositionIndex);
        motionStartTime = System.currentTimeMillis();
        hasReachedTarget = false; hasCrossedTarget = false;
        errorSignChanges = 0; rapidErrorChanges = 0;
        settledStartTime = 0; lastError = 0;
    }

    private void runTest() {
        double error = spindexer.shortestError(spindexer.getTargetPosition(), spindexer.getCurrentPosition());
        double absError = Math.abs(error);

        if (hasCrossedTarget) totalMaxOvershoot = Math.max(totalMaxOvershoot, absError);

        // Oscillation tracking
        if (Math.abs(error) > 20 && Math.abs(lastError) > 20) {
            if ((error > 0 && lastError < 0) || (error < 0 && lastError > 0)) errorSignChanges++;
        }

        // Jitter tracking
        if (hasReachedTarget && Math.abs(error - lastError) > 3.0) rapidErrorChanges++;

        lastError = error;
        long now = System.currentTimeMillis();

        if (absError <= POSITION_TOLERANCE) {
            if (settledStartTime == 0) settledStartTime = now;
            if (now - settledStartTime > STEADY_STATE_TIME_MS) hasReachedTarget = true;
        } else {
            settledStartTime = 0;
            if ((error > 0 && lastError < 0) || (error < 0 && lastError > 0)) hasCrossedTarget = true;
        }

        if (hasReachedTarget || (now - motionStartTime > MAX_TEST_TIME_MS / 3)) {
            totalSettlingTime += (now - motionStartTime);
            totalOscillationCount += errorSignChanges;
            positionsCompleted++;

            if (positionsCompleted < TOTAL_POSITIONS_TO_TEST) {
                currentPositionIndex = (currentPositionIndex + 1) % 3;
                prepareNewPosition();
            } else {
                finalizeTest();
            }
        }
    }

    private void finalizeTest() {
        metrics.oscillating = totalOscillationCount > 5;
        metrics.jittering = rapidErrorChanges > 15;
        metrics.steadyStateError = Math.abs(spindexer.shortestError(spindexer.getTargetPosition(), spindexer.getCurrentPosition()));
        metrics.score = (totalSettlingTime / 1000.0) + (totalMaxOvershoot * 0.1);

        if (!metrics.oscillating && !metrics.jittering && metrics.score < bestMetrics.score) {
            bestMetrics.score = metrics.score; bestP = currentP; bestD = currentD;
        }

        testInProgress = false; testJustFinished = true;
    }

    private void resetTuning() {
        currentPhase = TuningPhase.IDLE; testInProgress = false; intake.stop();
    }

    private void updateTelemetry() {
        telemetryM.addData("Phase", currentPhase);
        telemetryM.addData("P", currentP); telemetryM.addData("D", currentD);
        telemetryM.addData("Oscillating", metrics.oscillating);
        telemetryM.addData("Jittering", metrics.jittering);
        telemetryM.addData("Best P", bestP); telemetryM.addData("Best D", bestD);
        telemetryM.update(telemetry);
    }
}