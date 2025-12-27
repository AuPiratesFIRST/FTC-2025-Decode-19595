package org.firstinspires.ftc.teamcode.SubSystems.test;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.SubSystems.Spindexer.OldSpindexerSubsystem;

/**
 * Test OpMode for Spindexer subsystem.
 * Allows testing of position control and PID tuning with enhanced telemetry for graphing.
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
@TeleOp(name = "Spindexer Test with Graphs", group = "Test")
public class OldSpindexerTest extends LinearOpMode {

    private OldSpindexerSubsystem spindexer;
    private boolean manualMode = false;
    private double[] pidCoefficients = new double[2]; // [kP, kD] - I removed

    // PanelsTelemetry manager
    private TelemetryManager telemetryM;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        telemetryM.addLine("Status: Initializing Spindexer Test...");
        telemetryM.update(telemetry);

        // Initialize spindexer subsystem
        spindexer = new OldSpindexerSubsystem(hardwareMap, telemetry);

        // Get initial PID coefficients
        pidCoefficients = spindexer.getPIDCoefficients();

        telemetryM.addLine("Status: Ready");
        telemetryM.addLine("Controls:");
        telemetryM.addLine("A: Position 1");
        telemetryM.addLine("B: Position 2");
        telemetryM.addLine("X: Position 3");
        telemetryM.addLine("D-Pad: Adjust PID (Up/Down: P, Left/Right: I)");
        telemetryM.addLine("Bumpers: Adjust D gain");
        telemetryM.addLine("Y: Reset encoder");
        telemetryM.addLine("Left Stick Y: Manual control");
        telemetryM.update(telemetry);

        waitForStart();

        while (opModeIsActive()) {
            // Position selection
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

            // PID tuning (PD only - I removed)
            if (gamepad1.dpad_up) {
                pidCoefficients[0] += 0.001; // Increase P
                spindexer.setPIDCoefficients(pidCoefficients[0], pidCoefficients[1]);
                sleep(100);
            }
            if (gamepad1.dpad_down) {
                pidCoefficients[0] -= 0.001; // Decrease P
                if (pidCoefficients[0] < 0)
                    pidCoefficients[0] = 0;
                spindexer.setPIDCoefficients(pidCoefficients[0], pidCoefficients[1]);
                sleep(100);
            }
            if (gamepad1.right_bumper) {
                pidCoefficients[1] += 0.001; // Increase D
                spindexer.setPIDCoefficients(pidCoefficients[0], pidCoefficients[1]);
                sleep(100);
            }
            if (gamepad1.left_bumper) {
                pidCoefficients[1] -= 0.001; // Decrease D
                if (pidCoefficients[1] < 0)
                    pidCoefficients[1] = 0;
                spindexer.setPIDCoefficients(pidCoefficients[0], pidCoefficients[1]);
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

            // Enhanced telemetry for graphing
            updateTelemetry();
            sleep(20);
        }

        // Stop spindexer when OpMode ends
        spindexer.setManualPower(0);
    }

    /**
     * Updates the telemetry for graphing PID values, errors, and target positions.
     */
    private void updateTelemetry() {
        int currentPosition = spindexer.getCurrentPosition();
        int targetPosition = spindexer.getTargetPosition();
        double error = currentPosition - targetPosition;

        // Panels telemetry graph data
        telemetryM.addData("Mode", manualMode ? "MANUAL" : "PID");
        telemetryM.addData("Graph_Current_Position", currentPosition);
        telemetryM.addData("Graph_Target_Position", targetPosition);
        telemetryM.addData("Graph_Error", error);

        telemetryM.addData("Graph_PID_P", pidCoefficients[0]);
        telemetryM.addData("Graph_PID_D", pidCoefficients[1]);

        telemetryM.update(telemetry);
    }
}