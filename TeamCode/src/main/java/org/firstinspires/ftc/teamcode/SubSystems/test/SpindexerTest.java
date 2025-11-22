package org.firstinspires.ftc.teamcode.SubSystems.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SubSystems.Spindexer.SpindexerSubsystem;

/**
 * Test OpMode for Spindexer subsystem.
 * Allows testing of position control and PID tuning.
 * 
 * Controls:
 * - A Button: Go to Position 1
 * - B Button: Go to Position 2
 * - X Button: Go to Position 3
 * - D-Pad Up: Increase P gain
 * - D-Pad Down: Decrease P gain
 * - D-Pad Left: Decrease I gain
 * - D-Pad Right: Increase I gain
 * - Left Bumper: Decrease D gain
 * - Right Bumper: Increase D gain
 * - Y Button: Reset encoder
 * - Left Stick Y: Manual control (overrides PID)
 */
@TeleOp(name = "Spindexer Test", group = "Test")
public class SpindexerTest extends LinearOpMode {

    private SpindexerSubsystem spindexer;
    private boolean manualMode = false;
    private double[] pidCoefficients = new double[3];

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initializing Spindexer Test...");
        telemetry.update();

        // Initialize spindexer subsystem
        spindexer = new SpindexerSubsystem(hardwareMap, telemetry);

        // Get initial PID coefficients
        pidCoefficients = spindexer.getPIDCoefficients();

        telemetry.addData("Status", "Ready");
        telemetry.addLine();
        telemetry.addLine("Controls:");
        telemetry.addLine("A: Position 1");
        telemetry.addLine("B: Position 2");
        telemetry.addLine("X: Position 3");
        telemetry.addLine("D-Pad: Adjust PID (Up/Down: P, Left/Right: I)");
        telemetry.addLine("Bumpers: Adjust D gain");
        telemetry.addLine("Y: Reset encoder");
        telemetry.addLine("Left Stick Y: Manual control");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Position selection
            if (gamepad1.a) {
                spindexer.goToPosition(SpindexerSubsystem.SpindexerPosition.POSITION_1);
                manualMode = false;
                sleep(200);
            }
            if (gamepad1.b) {
                spindexer.goToPosition(SpindexerSubsystem.SpindexerPosition.POSITION_2);
                manualMode = false;
                sleep(200);
            }
            if (gamepad1.x) {
                spindexer.goToPosition(SpindexerSubsystem.SpindexerPosition.POSITION_3);
                manualMode = false;
                sleep(200);
            }

            // PID tuning
            if (gamepad1.dpad_up) {
                pidCoefficients[0] += 0.001; // Increase P
                spindexer.setPIDCoefficients(pidCoefficients[0], pidCoefficients[1], pidCoefficients[2]);
                sleep(100);
            }
            if (gamepad1.dpad_down) {
                pidCoefficients[0] -= 0.001; // Decrease P
                if (pidCoefficients[0] < 0)
                    pidCoefficients[0] = 0;
                spindexer.setPIDCoefficients(pidCoefficients[0], pidCoefficients[1], pidCoefficients[2]);
                sleep(100);
            }
            if (gamepad1.dpad_right) {
                pidCoefficients[1] += 0.0001; // Increase I
                spindexer.setPIDCoefficients(pidCoefficients[0], pidCoefficients[1], pidCoefficients[2]);
                sleep(100);
            }
            if (gamepad1.dpad_left) {
                pidCoefficients[1] -= 0.0001; // Decrease I
                if (pidCoefficients[1] < 0)
                    pidCoefficients[1] = 0;
                spindexer.setPIDCoefficients(pidCoefficients[0], pidCoefficients[1], pidCoefficients[2]);
                sleep(100);
            }
            if (gamepad1.right_bumper) {
                pidCoefficients[2] += 0.001; // Increase D
                spindexer.setPIDCoefficients(pidCoefficients[0], pidCoefficients[1], pidCoefficients[2]);
                sleep(100);
            }
            if (gamepad1.left_bumper) {
                pidCoefficients[2] -= 0.001; // Decrease D
                if (pidCoefficients[2] < 0)
                    pidCoefficients[2] = 0;
                spindexer.setPIDCoefficients(pidCoefficients[0], pidCoefficients[1], pidCoefficients[2]);
                sleep(100);
            }

            // Reset encoder
            if (gamepad1.y) {
                spindexer.reset();
                sleep(200);
            }

            // Manual control
            double manualPower = -gamepad1.left_stick_y;
            if (Math.abs(manualPower) > 0.1) {
                spindexer.setManualPower(manualPower * 0.5); // Scale down for safety
                manualMode = true;
            } else if (!manualMode) {
                // Update PID control
                spindexer.update();
            }

            // Update telemetry
            telemetry.addData("Status", manualMode ? "MANUAL MODE" : "PID MODE");
            telemetry.addData("PID Coefficients", "P: %.3f, I: %.3f, D: %.3f",
                    pidCoefficients[0], pidCoefficients[1], pidCoefficients[2]);
            spindexer.updateTelemetry();
            telemetry.update();

            sleep(20);
        }

        // Stop spindexer when OpMode ends
        spindexer.setManualPower(0);
    }
}

