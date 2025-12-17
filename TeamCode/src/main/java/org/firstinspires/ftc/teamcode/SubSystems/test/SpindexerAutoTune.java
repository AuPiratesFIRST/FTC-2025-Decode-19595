package org.firstinspires.ftc.teamcode.SubSystems.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SubSystems.Spindexer.OldSpindexerSubsystem;

@TeleOp(name = "Spindexer Auto-Tune", group = "Test")
public class SpindexerAutoTune extends LinearOpMode {

    private OldSpindexerSubsystem spindexer;

    // ===================== TUNING PHASE =====================
    private enum TuningPhase {
        IDLE,
        TUNING_P,
        TUNING_D,
        TUNING_I,
        COMPLETE
    }

    private TuningPhase currentPhase = TuningPhase.IDLE;

    // ===================== PID VALUES =====================
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

    // ===================== METRICS =====================
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

    // ===================== TEST STATE =====================
    private boolean testInProgress = false;
    private boolean testJustFinished = false;
    private boolean spindexerIsMoving = false;

    private long motionStartTime = 0;
    private static final long MAX_TEST_TIME_MS = 8000;
    private static final int POSITION_TOLERANCE = 15;

    private double lastError = 0;
    private int errorSignChanges = 0;
    private double maxError = 0;

    // =====================================================
    @Override
    public void runOpMode() throws InterruptedException {

        spindexer = new OldSpindexerSubsystem(hardwareMap, telemetry);
        spindexer.reset();
        sleep(500);

        telemetry.addLine("Spindexer Auto PID Tuner");
        telemetry.addLine("A = Start");
        telemetry.addLine("B = Skip Phase");
        telemetry.addLine("X = Reset");
        telemetry.addLine("Y = Finish");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a && currentPhase == TuningPhase.IDLE) {
                startTuning();
                sleep(300);
            }

            if (gamepad1.b && !testInProgress && currentPhase != TuningPhase.COMPLETE) {
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
    }

    // ===================== CONTROL =====================

    private void startTuning() {
        currentPhase = TuningPhase.TUNING_P;
        currentP = P_MIN;
        currentD = 0;
        currentI = 0;
        bestMetrics.reset();
        startTest();
    }

    private void continueTuning() {
        if (currentPhase == TuningPhase.TUNING_P) {
            currentP += P_INCREMENT;
            if (currentP > P_MAX) {
                currentPhase = TuningPhase.TUNING_D;
                currentP = bestP;
                currentD = D_MIN;
            }
            startTest();
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
            currentPhase = TuningPhase.TUNING_D;
        } else if (currentPhase == TuningPhase.TUNING_D) {
            currentPhase = TuningPhase.TUNING_I;
        } else if (currentPhase == TuningPhase.TUNING_I) {
            finishTuning();
            return;
        }
        startTest();
    }

    private void resetTuning() {
        currentPhase = TuningPhase.IDLE;
        testInProgress = false;
        spindexer.reset();
    }

    private void finishTuning() {
        spindexer.setPIDCoefficients(bestP, bestI, bestD);
        currentPhase = TuningPhase.COMPLETE;
    }

    // ===================== TEST =====================

    private void startTest() {
        spindexer.reset();
        sleep(200);

        spindexer.setPIDCoefficients(currentP, currentI, currentD);
        spindexer.goToPosition(OldSpindexerSubsystem.SpindexerPosition.POSITION_2);

        spindexerIsMoving = true;
        testInProgress = true;
        testJustFinished = false;

        motionStartTime = System.currentTimeMillis();
        metrics.reset();
        lastError = 0;
        errorSignChanges = 0;
        maxError = 0;
    }

    private void runTest() {
        spindexer.update();

        int current = spindexer.getCurrentPosition();
        int target = spindexer.getTargetPosition();
        double error = current - target;

        maxError = Math.max(maxError, Math.abs(error));

        if ((error > 0 && lastError < 0) || (error < 0 && lastError > 0)) {
            errorSignChanges++;
        }
        lastError = error;

        long now = System.currentTimeMillis();

        if (spindexerIsMoving && spindexer.isAtPosition()) {
            spindexerIsMoving = false;

            metrics.settlingTime = now - motionStartTime;
            metrics.steadyStateError = Math.abs(error);
            metrics.maxOvershoot = maxError;
            metrics.oscillationCount = errorSignChanges;
            metrics.oscillating = errorSignChanges > 4;

            metrics.score =
                    metrics.steadyStateError * 100 +
                    metrics.settlingTime * 0.1 +
                    metrics.maxOvershoot * 20 +
                    (metrics.oscillating ? 2000 : 0);

            if (metrics.steadyStateError <= POSITION_TOLERANCE &&
                metrics.score < bestMetrics.score) {

                copyMetrics(metrics, bestMetrics);
                bestP = currentP;
                bestD = currentD;
                bestI = currentI;
            }

            testInProgress = false;
            testJustFinished = true;
        }

        if (now - motionStartTime > MAX_TEST_TIME_MS) {
            testInProgress = false;
            testJustFinished = true;
        }
    }

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
        telemetry.addData("Graph_Position", currentPos);
        telemetry.addData("Graph_Target", targetPos);
        telemetry.addData("Graph_Error", error);
        telemetry.addData("Graph_Time_ms", timeSinceMove);

        telemetry.addData("Graph_P", currentP);
        telemetry.addData("Graph_I", currentI);
        telemetry.addData("Graph_D", currentD);

        telemetry.addData("Phase", currentPhase);
        telemetry.addData("Moving", spindexerIsMoving);
        telemetry.addData("Settling", spindexer.isSettling());

        telemetry.update();
    }
}
