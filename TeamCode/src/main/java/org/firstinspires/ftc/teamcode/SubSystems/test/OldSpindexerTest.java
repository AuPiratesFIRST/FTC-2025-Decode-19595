package org.firstinspires.ftc.teamcode.SubSystems.test;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.SubSystems.Spindexer.OldSpindexerSubsystem;

/**
 * Updated Test OpMode for Spindexer.
 * Tuning Sequence:
 * 1. Use D-Pad Up/Down for P (Reach the target)
 * 2. Use Bumpers for D (Stop the bouncing/overshoot)
 * 3. Use D-Pad Left/Right for I (The "Rock Solid" holding force)
 */
@TeleOp(name = "Spindexer Test with Active Hold", group = "Test")
public class OldSpindexerTest extends LinearOpMode {

    private OldSpindexerSubsystem spindexer;
    private boolean manualMode = false;

    // Array to store [kP, kI, kD]
    private double[] pidCoefficients = new double[3];

    private TelemetryManager telemetryM;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        telemetryM.addLine("Status: Initializing Spindexer Test...");
        telemetryM.update(telemetry);

        spindexer = new OldSpindexerSubsystem(hardwareMap, telemetry);

        // Get initial [kP, kI, kD] from subsystem
        pidCoefficients = spindexer.getPIDCoefficients();

        telemetryM.addLine("Status: Ready");
        telemetryM.addLine("--- CONTROLS ---");
        telemetryM.addLine("Buttons: A(Pos1), B(Pos2), X(Pos3)");
        telemetryM.addLine("D-Pad Up/Down: Tune P (Power)");
        telemetryM.addLine("D-Pad Left/Right: Tune I (Hold Force)");
        telemetryM.addLine("Bumpers: Tune D (Damping)");
        telemetryM.addLine("Y: Reset Encoder | Stick: Manual");
        telemetryM.update(telemetry);

        waitForStart();

        while (opModeIsActive()) {
            // Position Selection
            if (gamepad1.a) {
                spindexer.goToPosition(OldSpindexerSubsystem.SpindexerPosition.POSITION_1);
                manualMode = false;
                sleep(200);
            }
            if (gamepad1.b) {
                spindexer.goToPosition(OldSpindexerSubsystem.SpindexerPosition.POSITION_2);
                manualMode = false;
                sleep(200);
            }
            if (gamepad1.x) {
                spindexer.goToPosition(OldSpindexerSubsystem.SpindexerPosition.POSITION_3);
                manualMode = false;
                sleep(200);
            }

            // --- LIVE PID TUNING ---

            // Tune P (Up/Down)
            if (gamepad1.dpad_up) {
                pidCoefficients[0] += 0.0005;
                updateSubsystemPID();
                sleep(100);
            }
            if (gamepad1.dpad_down) {
                pidCoefficients[0] -= 0.0005;
                if (pidCoefficients[0] < 0) pidCoefficients[0] = 0;
                updateSubsystemPID();
                sleep(100);
            }

            // Tune I (Left/Right) - The Holding Force
            if (gamepad1.dpad_right) {
                pidCoefficients[1] += 0.00001;
                updateSubsystemPID();
                sleep(100);
            }
            if (gamepad1.dpad_left) {
                pidCoefficients[1] -= 0.00001;
                if (pidCoefficients[1] < 0) pidCoefficients[1] = 0;
                updateSubsystemPID();
                sleep(100);
            }

            // Tune D (Bumpers)
            if (gamepad1.right_bumper) {
                pidCoefficients[2] += 0.0001;
                updateSubsystemPID();
                sleep(100);
            }
            if (gamepad1.left_bumper) {
                pidCoefficients[2] -= 0.0001;
                if (pidCoefficients[2] < 0) pidCoefficients[2] = 0;
                updateSubsystemPID();
                sleep(100);
            }

            // Reset encoder
            if (gamepad1.y) {
                spindexer.reset();
                manualMode = false;
                sleep(200);
            }

            // Manual control vs PID Update
            double manualPower = -gamepad1.left_stick_y;
            if (Math.abs(manualPower) > 0.1) {
                spindexer.setManualPower(manualPower * 0.5);
                manualMode = true;
            } else {
                if (manualMode) {
                    // If we just released the stick, lock the position
                    spindexer.lockCurrentPosition();
                    manualMode = false;
                }
                spindexer.update();
            }

            updateTelemetryDisplay();
            sleep(20);
        }

        spindexer.setManualPower(0);
    }

    private void updateSubsystemPID() {
        // Sends P, I, D to the subsystem
        spindexer.setPIDCoefficients(pidCoefficients[0], pidCoefficients[1], pidCoefficients[2]);
    }

    private void updateTelemetryDisplay() {
        int currentPosition = spindexer.getCurrentPosition();
        int targetPosition = spindexer.getTargetPosition();
        // Use shortestError for the graph so the wrap-around (0/2150) doesn't look like a spike
        double error = spindexer.shortestError(targetPosition, currentPosition);

        telemetryM.addData("Mode", manualMode ? "MANUAL" : "ACTIVE HOLD");
        telemetryM.addData("Graph_Current", currentPosition);
        telemetryM.addData("Graph_Target", targetPosition);
        telemetryM.addData("Graph_Error", error);

        // Track the PID values on the graph to see how they affect stability
        telemetryM.addData("Graph_kP", pidCoefficients[0]);
        telemetryM.addData("Graph_kI", pidCoefficients[1]);
        telemetryM.addData("Graph_kD", pidCoefficients[2]);

        telemetryM.update(telemetry);
    }
}