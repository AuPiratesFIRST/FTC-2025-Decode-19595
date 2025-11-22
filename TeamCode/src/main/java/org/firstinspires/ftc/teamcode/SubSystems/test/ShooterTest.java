package org.firstinspires.ftc.teamcode.SubSystems.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SubSystems.Shooter.ShooterSubsystem;

/**
 * Test OpMode for Shooter subsystem.
 * Allows testing and tuning of shooter speeds.
 * 
 * Controls:
 * - Right Trigger: Increase power
 * - Left Trigger: Decrease power
 * - A Button: Set to LOW speed
 * - B Button: Set to MEDIUM speed
 * - X Button: Set to HIGH speed
 * - Y Button: Set to MAX speed
 * - Right Bumper: Start/Stop toggle
 */
@TeleOp(name = "Shooter Test", group = "Test")
public class ShooterTest extends LinearOpMode {

    private ShooterSubsystem shooter;
    private boolean isRunning = false;
    private double currentPower = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initializing Shooter Test...");
        telemetry.update();

        // Initialize shooter subsystem
        shooter = new ShooterSubsystem(hardwareMap, telemetry);

        telemetry.addData("Status", "Ready");
        telemetry.addLine();
        telemetry.addLine("Controls:");
        telemetry.addLine("Right Trigger: Increase power");
        telemetry.addLine("Left Trigger: Decrease power");
        telemetry.addLine("A: LOW speed");
        telemetry.addLine("B: MEDIUM speed");
        telemetry.addLine("X: HIGH speed");
        telemetry.addLine("Y: MAX speed");
        telemetry.addLine("Right Bumper: Toggle on/off");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Power adjustment with triggers
            if (gamepad1.right_trigger > 0.1) {
                currentPower += 0.01;
                if (currentPower > 1.0)
                    currentPower = 1.0;
            }
            if (gamepad1.left_trigger > 0.1) {
                currentPower -= 0.01;
                if (currentPower < 0.0)
                    currentPower = 0.0;
            }

            // Speed presets
            if (gamepad1.a) {
                shooter.setSpeed(ShooterSubsystem.ShooterSpeed.LOW);
                currentPower = ShooterSubsystem.ShooterSpeed.LOW.getPower();
                isRunning = true;
            }
            if (gamepad1.b) {
                shooter.setSpeed(ShooterSubsystem.ShooterSpeed.MEDIUM);
                currentPower = ShooterSubsystem.ShooterSpeed.MEDIUM.getPower();
                isRunning = true;
            }
            if (gamepad1.x) {
                shooter.setSpeed(ShooterSubsystem.ShooterSpeed.HIGH);
                currentPower = ShooterSubsystem.ShooterSpeed.HIGH.getPower();
                isRunning = true;
            }
            if (gamepad1.y) {
                shooter.setSpeed(ShooterSubsystem.ShooterSpeed.MAX);
                currentPower = ShooterSubsystem.ShooterSpeed.MAX.getPower();
                isRunning = true;
            }

            // Toggle on/off
            if (gamepad1.right_bumper) {
                isRunning = !isRunning;
                sleep(200); // Debounce
            }

            // Apply power
            if (isRunning) {
                shooter.setPower(currentPower);
            } else {
                shooter.stop();
            }

            // Update telemetry
            telemetry.addData("Status", isRunning ? "RUNNING" : "STOPPED");
            telemetry.addData("Current Power", "%.2f", currentPower);
            shooter.updateTelemetry();
            telemetry.update();

            sleep(20);
        }

        // Stop shooter when OpMode ends
        shooter.stop();
    }
}

