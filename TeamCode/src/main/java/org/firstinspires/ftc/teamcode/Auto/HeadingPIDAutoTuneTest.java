package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.SubSystems.Control.HeadingAutoTuner;
import org.firstinspires.ftc.teamcode.SubSystems.Drive.DriveSubsystem;

@Autonomous(name = "PID Auto-Tuner", group = "Test")
public class HeadingPIDAutoTuneTest extends LinearOpMode {

    private DriveSubsystem drive;
    private HeadingAutoTuner tuner;

    private static final double RELAY_POWER = 0.35;
    private static final double TIMEOUT_SECONDS = 15.0;

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new DriveSubsystem(hardwareMap, telemetry);

        telemetry.addLine("=== PID AUTO-TUNER READY ===");
        telemetry.addLine();
        telemetry.addLine("⚠️ SAFETY CHECKLIST:");
        telemetry.addLine("  ✓ Wheels free or lifted?");
        telemetry.addLine("  ✓ Clear space around robot?");
        telemetry.addLine("  ✓ Ready to press STOP if needed?");
        telemetry.addLine();
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

            // Safety timeout
            if (getRuntime() - startTime > TIMEOUT_SECONDS) {
                telemetry.clearAll();
                telemetry.addLine("❌ TIMEOUT - Tuning aborted");
                telemetry.addLine("Possible issues:");
                telemetry.addLine("- Robot wheels blocked");
                telemetry.addLine("- IMU not responding");
                telemetry.addLine("- Relay power too low");
                telemetry.update();
                break;
            }

            // Get motor turn power
            double turn = tuner.update(drive.getHeading());
            drive.drive(0, 0, turn);

            // Display telemetry
            double[] stats = tuner.getStats();
            int zeroCrossings = (int) stats[0];
            double progress = (zeroCrossings / 8.0) * 100;

            telemetry.clearAll();
            telemetry.addLine("🔄 TUNING IN PROGRESS...");
            telemetry.addData("Zero Crossings", "%d / 8", zeroCrossings);
            telemetry.addData("Progress", "%.0f%%", progress);
            telemetry.addData("Heading", "%.1f°", Math.toDegrees(drive.getHeading()));
            telemetry.addData("Turn Power", "%.2f", turn);
            telemetry.addData("Max Error", "%.2f°", stats[1]);
            telemetry.addData("Min Error", "%.2f°", stats[2]);
            telemetry.update();

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
