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
    private long motionStartTime = 0;
    private static final long MAX_TEST_TIME_MS = 5000;
    private static final long SETTLING_TIME_MS = 1000; // Time to wait for shooter to settle
    private double lastError = 0;
    private int errorSignChanges = 0;
    private double maxError = 0;

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
        testInProgress = true;
        testJustFinished = false;

        motionStartTime = System.currentTimeMillis();
        metrics.reset();
        lastError = 0;
        errorSignChanges = 0;
        maxError = 0;
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

        // Wait for shooter to settle before measuring
        if (shooterIsSettling && elapsed > SETTLING_TIME_MS) {
            shooterIsSettling = false;
            motionStartTime = now; // Reset timer for actual measurement
        }

        // Measure performance after settling
        if (!shooterIsSettling) {
            // Check if we've reached steady state (within tolerance for a period)
            if (absError <= RPM_TOLERANCE) {
                if (metrics.settlingTime == 0) {
                    metrics.settlingTime = now - motionStartTime;
                }
                metrics.steadyStateError = absError;
            }

            // Check for timeout
            if (elapsed > MAX_TEST_TIME_MS) {
                metrics.settlingTime = elapsed;
                metrics.steadyStateError = absError;
                metrics.maxOvershoot = maxError;
                metrics.oscillationCount = errorSignChanges;
                metrics.oscillating = errorSignChanges > 4;

                metrics.score = calculateScore();

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

    private double calculateScore() {
        return metrics.steadyStateError * 10 +
                metrics.settlingTime * 0.01 +
                metrics.maxOvershoot * 5 +
                (metrics.oscillating ? 1000 : 0);
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
        telemetry.addData("Current RPM", "%.1f", currentRPM);
        telemetry.addData("Target RPM", "%.1f", TARGET_RPM);
        telemetry.addData("Error", "%.1f", error);

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
