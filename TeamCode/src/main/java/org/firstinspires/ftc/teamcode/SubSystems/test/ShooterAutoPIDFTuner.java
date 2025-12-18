package org.firstinspires.ftc.teamcode.SubSystems.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.teamcode.SubSystems.Shooter.ShooterSubsystem;

@TeleOp(name = "Shooter Auto PIDF Tuner", group = "Tuning")
public class ShooterAutoPIDFTuner extends LinearOpMode {

    private ShooterSubsystem shooter;
    private DcMotorEx motor; // use ONE motor for tuning consistency

    // ================= TARGET =================
    private static final double TARGET_RPM = 5210.0;
    private static final double RPM_TOLERANCE = 50.0; // Acceptable error in RPM

    // ================= SEARCH RANGES =================
    private static final double KF_MIN = 5;
    private static final double KF_MAX = 25;
    private static final double KF_STEP = 0.5;

    private static final double KP_MIN = 0;
    private static final double KP_MAX = 80;
    private static final double KP_STEP = 2;

    private static final double KD_MIN = 0;
    private static final double KD_MAX = 30;
    private static final double KD_STEP = 1;

    // ================= BEST =================
    private double bestKf = 0;
    private double bestKp = 0;
    private double bestKd = 0;

    private enum TuningPhase {
        IDLE,
        TUNING_KF,
        TUNING_KP,
        TUNING_KD,
        COMPLETE
    }

    private TuningPhase currentPhase = TuningPhase.IDLE;

    // Current tuning values
    private double currentKf = KF_MIN;
    private double currentKp = KP_MIN;
    private double currentKd = KD_MIN;

    // Performance metrics
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
    private boolean shooterIsSettling = false;
    private boolean hasReachedSteadyState = false;
    private long motionStartTime = 0;
    private long steadyStateStartTime = 0;
    private static final long MAX_TEST_TIME_MS = 6000;
    private static final long INITIAL_SETTLING_TIME_MS = 1500; // Time to wait before measuring
    private static final long STEADY_STATE_DURATION_MS = 1000; // Time to maintain steady state
    private double lastError = 0;
    private int errorSignChanges = 0;
    private double maxError = 0;
    private double steadyStateErrorSum = 0;
    private int steadyStateSamples = 0;
    private long firstWithinToleranceTime = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        shooter = new ShooterSubsystem(hardwareMap, telemetry);

        motor = hardwareMap.get(DcMotorEx.class, "shooterL");
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("Shooter Auto PIDF Tuner");
        telemetry.addLine("NOTE: Balls NOT needed - tuning RPM only");
        telemetry.addLine("A = Start");
        telemetry.addLine("B = Skip Phase");
        telemetry.addLine("X = Reset");
        telemetry.addLine("Y = Finish");
        telemetry.addLine("");
        telemetry.addLine("If testing with balls, manually feed them");
        telemetry.addLine("during tuning to prevent them from falling out.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            shooter.updateVoltageCompensation();

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

            updateTelemetry();
            sleep(20);
        }

        shooter.stop();
    }

    // =====================================================

    // Start tuning process
    private void startTuning() {
        currentPhase = TuningPhase.TUNING_KF;
        currentKf = KF_MIN;
        currentKp = 0;
        currentKd = 0;
        bestMetrics.reset();
        startTest();
    }

    // Continue tuning based on current phase
    private void continueTuning() {
        if (currentPhase == TuningPhase.TUNING_KF) {
            currentKf += KF_STEP;
            if (currentKf > KF_MAX) {
                currentPhase = TuningPhase.TUNING_KP;
                currentKf = bestKf;
                currentKp = KP_MIN;
            }
            startTest();
        } else if (currentPhase == TuningPhase.TUNING_KP) {
            currentKp += KP_STEP;
            if (currentKp > KP_MAX) {
                currentPhase = TuningPhase.TUNING_KD;
                currentKp = bestKp;
                currentKd = KD_MIN;
            }
            startTest();
        } else if (currentPhase == TuningPhase.TUNING_KD) {
            currentKd += KD_STEP;
            if (currentKd > KD_MAX) {
                finishTuning();
                return;
            }
            startTest();
        }
    }

    private void skipPhase() {
        if (currentPhase == TuningPhase.TUNING_KF) {
            currentPhase = TuningPhase.TUNING_KP;
            currentKf = bestKf;
            currentKp = KP_MIN;
        } else if (currentPhase == TuningPhase.TUNING_KP) {
            currentPhase = TuningPhase.TUNING_KD;
            currentKp = bestKp;
            currentKd = KD_MIN;
        } else if (currentPhase == TuningPhase.TUNING_KD) {
            finishTuning();
            return;
        }
        startTest();
    }

    private void resetTuning() {
        currentPhase = TuningPhase.IDLE;
        testInProgress = false;
        shooter.stop();
    }

    private void finishTuning() {
        applyPIDF(bestKp, 0, bestKd, bestKf);
        currentPhase = TuningPhase.COMPLETE;
    }

    // =====================================================

    // Run the test for the current PIDF coefficients
    private void startTest() {
        shooter.stop();
        sleep(200);

        applyPIDF(currentKp, 0, currentKd, currentKf);
        shooter.setTargetRPM(TARGET_RPM);

        shooterIsSettling = true;
        hasReachedSteadyState = false;
        testInProgress = true;
        testJustFinished = false;

        motionStartTime = System.currentTimeMillis();
        steadyStateStartTime = 0;
        firstWithinToleranceTime = 0;
        metrics.reset();
        lastError = 0;
        errorSignChanges = 0;
        maxError = 0;
        steadyStateErrorSum = 0;
        steadyStateSamples = 0;
    }

    // Run the test to evaluate the current PIDF coefficients
    private void runTest() {
        double currentRPM = shooter.getCurrentRPM();
        double error = currentRPM - TARGET_RPM;
        double absError = Math.abs(error);

        maxError = Math.max(maxError, absError);

        // Detect oscillations (error sign changes)
        if ((error > 0 && lastError < 0) || (error < 0 && lastError > 0)) {
            errorSignChanges++;
        }
        lastError = error;

        long now = System.currentTimeMillis();
        long elapsed = now - motionStartTime;

        // Phase 1: Wait for initial settling (let motor reach target)
        if (shooterIsSettling && elapsed < INITIAL_SETTLING_TIME_MS) {
            // Still in initial settling phase
            return;
        }

        if (shooterIsSettling && elapsed >= INITIAL_SETTLING_TIME_MS) {
            shooterIsSettling = false;
            // Start measuring steady-state performance
        }

        // Phase 2: Measure steady-state performance
        if (!shooterIsSettling) {
            // Track when we first get within tolerance
            if (absError <= RPM_TOLERANCE && firstWithinToleranceTime == 0) {
                firstWithinToleranceTime = now;
                metrics.settlingTime = now - motionStartTime;
            }

            // Collect steady-state samples when within tolerance
            if (absError <= RPM_TOLERANCE) {
                if (!hasReachedSteadyState) {
                    hasReachedSteadyState = true;
                    steadyStateStartTime = now;
                }
                steadyStateErrorSum += absError;
                steadyStateSamples++;
            } else {
                // Lost steady state - reset
                hasReachedSteadyState = false;
                steadyStateStartTime = 0;
            }

            // Check if we've maintained steady state long enough
            boolean steadyStateComplete = hasReachedSteadyState &&
                    (now - steadyStateStartTime) >= STEADY_STATE_DURATION_MS;

            // Check for timeout
            boolean timeout = elapsed > MAX_TEST_TIME_MS;

            if (steadyStateComplete || timeout) {
                // Calculate final metrics
                if (steadyStateSamples > 0) {
                    metrics.steadyStateError = steadyStateErrorSum / steadyStateSamples;
                } else {
                    metrics.steadyStateError = absError;
                }

                // If we never reached steady state, use timeout as settling time
                if (metrics.settlingTime == 0) {
                    metrics.settlingTime = elapsed;
                }

                metrics.maxOvershoot = maxError;
                metrics.oscillationCount = errorSignChanges;
                metrics.oscillating = errorSignChanges > 6; // More lenient for velocity control

                metrics.score = calculateScore();

                // Update best if within tolerance and better score
                if (metrics.steadyStateError <= RPM_TOLERANCE &&
                        metrics.score < bestMetrics.score) {
                    copyMetrics(metrics, bestMetrics);
                    bestKf = currentKf;
                    bestKp = currentKp;
                    bestKd = currentKd;
                }

                testInProgress = false;
                testJustFinished = true;
                shooter.stop();
                sleep(300);
            }
        }
    }

    // Improved scoring function for velocity control (based on WPI principles)
    private double calculateScore() {
        // Lower score is better
        // For velocity control: steady-state accuracy > settling time > overshoot > oscillations
        // WPI emphasizes that feedforward (kF) handles most of the work, PID just fine-tunes
        return metrics.steadyStateError * 15.0 +           // Accuracy is most important
                metrics.settlingTime * 0.02 +              // Faster settling is better
                metrics.maxOvershoot * 3.0 +               // Overshoot penalty (less critical for velocity)
                (metrics.oscillating ? 2000.0 : 0.0);      // Heavy penalty for oscillations
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

    private void updateTelemetry() {
        double currentRPM = shooter.getCurrentRPM();
        double error = currentRPM - TARGET_RPM;
        double timeSinceStart = testInProgress
                ? (System.currentTimeMillis() - motionStartTime)
                : 0;

        telemetry.addData("Phase", currentPhase);
        telemetry.addData("Settling", shooterIsSettling);
        telemetry.addData("Steady State", hasReachedSteadyState);
        telemetry.addData("Current RPM", "%.1f", currentRPM);
        telemetry.addData("Target RPM", "%.1f", TARGET_RPM);
        telemetry.addData("Error", "%.1f", error);
        
        if (testInProgress) {
            telemetry.addData("Steady Samples", steadyStateSamples);
            telemetry.addData("Current Score", "%.2f", metrics.score);
            telemetry.addData("Best Score", "%.2f", bestMetrics.score);
        }

        telemetry.addData("Current kF", "%.2f", currentKf);
        telemetry.addData("Current kP", "%.2f", currentKp);
        telemetry.addData("Current kD", "%.2f", currentKd);

        telemetry.addData("Best kF", "%.2f", bestKf);
        telemetry.addData("Best kP", "%.2f", bestKp);
        telemetry.addData("Best kD", "%.2f", bestKd);

        // Graph data - Panels Graph plugin reads from telemetry
        telemetry.addData("RPM", currentRPM);
        telemetry.addData("Target", TARGET_RPM);
        telemetry.addData("Error", error);
        telemetry.addData("kF", currentKf);
        telemetry.addData("kP", currentKp);
        telemetry.addData("kD", currentKd);
        telemetry.addData("Time", timeSinceStart);

        telemetry.update();
    }

    private void applyPIDF(double p, double i, double d, double f) {
        PIDFCoefficients pidf = new PIDFCoefficients(p, i, d, f);
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
    }
}
