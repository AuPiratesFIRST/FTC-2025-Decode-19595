package org.firstinspires.ftc.teamcode.Auto;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.SubSystems.Control.HeadingAutoTuner;
import org.firstinspires.ftc.teamcode.SubSystems.Drive.DriveSubsystem;

@Autonomous(name = "PID Auto-Tuner", group = "Test")
public class HeadingPIDAutoTuneTest extends LinearOpMode {

    private DriveSubsystem drive;
    private HeadingAutoTuner tuner;
    private TelemetryManager telemetryM;

    private static final double RELAY_POWER = 0.35;
    private static final double TIMEOUT_SECONDS = 15.0;

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new DriveSubsystem(hardwareMap, telemetry);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        telemetry.addLine("=== PID AUTO-TUNER READY ===");
        telemetry.addLine();
        telemetry.addLine("⚠️ CRITICAL REQUIREMENTS:");
        telemetry.addLine("  ✅ Robot ON THE FLOOR (not lifted!)");
        telemetry.addLine("  ✅ Wheels can touch ground & rotate");
        telemetry.addLine("  ✅ Clear 360° space (~30° rotation)");
        telemetry.addLine("  ✅ Hold robot to prevent drift");
        telemetry.addLine();
        telemetry.addLine("❌ DO NOT run if wheels are lifted!");
        telemetry.addLine("❌ IMU will not change → tuning fails");
        telemetry.addLine();
        telemetry.addLine("📊 panels telemetry ENABLED");
        telemetry.addLine("Press START to begin tuning...");
        telemetry.update();

        waitForStart();
        if (!opModeIsActive()) return;

        drive.resetHeading();
        sleep(100);

        double targetHeading = drive.getHeading();
        tuner = new HeadingAutoTuner(targetHeading, RELAY_POWER);

        double startTime = getRuntime();

        while (opModeIsActive() && !tuner.isFinished()) {

            // Early detection: Robot not rotating
            if (tuner.isStuck()) {
                telemetry.clearAll();
                telemetry.addLine("❌ TUNING STUCK - NO ROTATION DETECTED");
                telemetry.addLine();
                telemetry.addLine("Likely causes:");
                telemetry.addLine("  🚨 Robot wheels are LIFTED");
                telemetry.addLine("  🚨 Wheels blocked/can't rotate");
                telemetry.addLine("  🚨 IMU heading not changing");
                telemetry.addLine();
                telemetry.addLine("✅ Put robot ON THE FLOOR and retry");
                telemetry.update();
                break;
            }

            // Safety timeout
            if (getRuntime() - startTime > TIMEOUT_SECONDS) {
                telemetry.clearAll();
                telemetry.addLine("❌ TIMEOUT - Tuning incomplete");
                telemetry.addLine("Possible issues:");
                telemetry.addLine("- Oscillation too slow");
                telemetry.addLine("- Relay power too low");
                telemetry.addLine("- Try increasing RELAY_POWER");
                telemetry.update();
                break;
            }

            // Get motor turn power
            double currentHeading = drive.getHeading();
            double turn = tuner.update(currentHeading);
            drive.drive(0, 0, turn);

            // Calculate error for graphing
            double errorDeg = Math.toDegrees(
                AngleUnit.normalizeRadians(targetHeading - currentHeading)
            );

            // Display telemetry
            double[] stats = tuner.getStats();
            int zeroCrossings = (int) stats[0];
            double progress = (zeroCrossings / 8.0) * 100;

            // === PEDRO PATHING TELEMETRY (Live Graphs) ===
            telemetryM.addData("Graph_TargetHeading", Math.toDegrees(targetHeading));
            telemetryM.addData("Graph_CurrentHeading", Math.toDegrees(currentHeading));
            telemetryM.addData("Graph_Error", errorDeg);
            telemetryM.addData("Graph_TurnPower", turn);
            telemetryM.addData("Graph_ZeroCrossings", zeroCrossings);

            telemetry.clearAll();
            telemetry.addLine("🔄 TUNING IN PROGRESS...");
            telemetry.addData("Zero Crossings", "%d / 8", zeroCrossings);
            telemetry.addData("Progress", "%.0f%%", progress);
            telemetry.addData("Heading", "%.1f°", drive.getHeadingDegrees()); // Use getHeadingDegrees() for telemetry
            telemetry.addData("Turn Power", "%.2f", turn);
            telemetry.addData("Max Error", "%.2f°", stats[1]);
            telemetry.addData("Min Error", "%.2f°", stats[2]);
            
            telemetryM.update(telemetry);

            sleep(25); // 40Hz update rate
        }

        drive.stop();

        if (tuner.isFinished()) {
            HeadingAutoTuner.PIDGains gains = tuner.getGains();

            telemetry.clearAll();
            telemetry.addLine("=== ✅ TUNING COMPLETE ===");
            telemetry.addLine();
            telemetry.addLine("📊 TUNED PID GAINS:");
            telemetry.addData("kP", "%.4f", gains.kp);
            telemetry.addData("kI", "%.4f", gains.ki);
            telemetry.addData("kD", "%.4f", gains.kd);
            telemetry.addLine();
            telemetry.addLine("-----------------------------------");
            telemetry.addLine("📝 APPLY TO YOUR CODE:");
            telemetry.addLine(String.format("new HeadingPID(%.4f, %.4f, %.4f);",
                    gains.kp, gains.ki, gains.kd));
            telemetry.addLine("-----------------------------------");
            telemetry.addLine("💾 SAVE THESE GAINS!");
            telemetry.update();

            // Keep results on screen
            while (opModeIsActive()) sleep(100);
        }
    }
}
