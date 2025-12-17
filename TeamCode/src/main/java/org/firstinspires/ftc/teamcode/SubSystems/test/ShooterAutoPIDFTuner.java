package org.firstinspires.ftc.teamcode.SubSystems.Shooter;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp(name = "Shooter Auto PIDF Tuner", group = "Tuning")
public class ShooterAutoPIDFTuner extends LinearOpMode {

    private ShooterSubsystem shooter;
    private DcMotorEx motor; // use ONE motor for tuning consistency

    // ================= TARGET =================
    private static final double TARGET_RPM = 5210.0;

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
    private double bestError = Double.MAX_VALUE;

    private enum Phase {
        KF,
        KP,
        KD,
        DONE
    }

    private Phase phase = Phase.KF;

    @Override
    public void runOpMode() {
        shooter = new ShooterSubsystem(hardwareMap, telemetry);

        motor = hardwareMap.get(DcMotorEx.class, "shooterL");
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("Shooter Auto PIDF Tuner");
        telemetry.addLine("A = Start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            shooter.updateVoltageCompensation();

            if (gamepad1.a) {
                runTuning();
                break;
            }

            idle();
        }

        telemetry.addLine("TUNING COMPLETE");
        telemetry.addData("BEST kF", bestKf);
        telemetry.addData("BEST kP", bestKp);
        telemetry.addData("BEST kD", bestKd);
        telemetry.update();

        while (opModeIsActive()) idle();
    }

    // =====================================================

    private void runTuning() {

        // ---------- KF ----------
        for (double kf = KF_MIN; kf <= KF_MAX; kf += KF_STEP) {
            applyPIDF(0, 0, 0, kf);
            double error = runTest();

            if (error < bestError) {
                bestError = error;
                bestKf = kf;
            }
        }

        // ---------- KP ----------
        for (double kp = KP_MIN; kp <= KP_MAX; kp += KP_STEP) {
            applyPIDF(kp, 0, 0, bestKf);
            double error = runTest();

            if (error < bestError) {
                bestError = error;
                bestKp = kp;
            }
        }

        // ---------- KD ----------
        for (double kd = KD_MIN; kd <= KD_MAX; kd += KD_STEP) {
            applyPIDF(bestKp, 0, kd, bestKf);
            double error = runTest();

            if (error < bestError) {
                bestError = error;
                bestKd = kd;
            }
        }

        phase = Phase.DONE;
        applyPIDF(bestKp, 0, bestKd, bestKf);
    }

    // =====================================================

    private double runTest() {

        shooter.setTargetRPM(TARGET_RPM);

        long start = System.currentTimeMillis();
        double totalError = 0;
        int samples = 0;

        while (opModeIsActive() && System.currentTimeMillis() - start < 2000) {

            double rpm = shooter.getCurrentRPM();
            double error = Math.abs(TARGET_RPM - rpm);

            totalError += error;
            samples++;

            // ===== GRAPHABLE =====
            telemetry.addData("Graph_RPM", rpm);
            telemetry.addData("Graph_Target", TARGET_RPM);
            telemetry.addData("Graph_Error", error);

            telemetry.addData("Phase", phase);
            telemetry.addData("kF", bestKf);
            telemetry.addData("kP", bestKp);
            telemetry.addData("kD", bestKd);
            telemetry.update();

            sleep(20);
        }

        shooter.stop();
        sleep(300);

        return totalError / samples;
    }

    private void applyPIDF(double p, double i, double d, double f) {
        PIDFCoefficients pidf = new PIDFCoefficients(p, i, d, f);
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
    }
}
