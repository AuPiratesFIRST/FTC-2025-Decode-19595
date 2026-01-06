package org.firstinspires.ftc.teamcode.SubSystems.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SubSystems.Funnel.FunnelSubsystem;

/**
 * Test OpMode for Funnel subsystem.
 * Allows testing of funnel servo positions and synchronization.
 * 
 * Controls:
 * - A Button: Extend funnels
 * - B Button: Retract funnels
 * - X Button: Toggle extend/retract
 * - D-Pad Up: Increase position (both servos)
 * - D-Pad Down: Decrease position (both servos)
 * - Left Stick Y: Manual position control (both servos)
 * - Right Stick Y: Manual left servo position control
 * - Right Stick X: Manual right servo position control
 */
@TeleOp(name = "Funnel Test", group = "Test")
public class FunnelTest extends LinearOpMode {

    private FunnelSubsystem funnel;
    private boolean lastX = false;
    private boolean lastA = false;
    private boolean lastB = false;
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initializing Funnel Test...");
        telemetry.update();

        // Initialize funnel subsystem
        funnel = new FunnelSubsystem(hardwareMap, telemetry);

        telemetry.addData("Status", "Ready");
        telemetry.addLine();
        telemetry.addLine("Controls:");
        telemetry.addLine("A: Extend funnels");
        telemetry.addLine("B: Retract funnels");
        telemetry.addLine("X: Toggle extend/retract");
        telemetry.addLine("D-Pad Up: Increase position");
        telemetry.addLine("D-Pad Down: Decrease position");
        telemetry.addLine("Left Stick Y: Manual position (both)");
        telemetry.addLine("Right Stick Y: Manual left servo");
        telemetry.addLine("Right Stick X: Manual right servo");
        telemetry.update();

        waitForStart();

        double manualPosition = 0.5; // Start at middle position

        while (opModeIsActive()) {
            // Button controls
            boolean a = gamepad1.a;
            boolean b = gamepad1.b;
            boolean x = gamepad1.x;
            boolean dpadUp = gamepad1.dpad_up;
            boolean dpadDown = gamepad1.dpad_down;

            // Extend button
            if (a && !lastA) {
                funnel.extend();
                manualPosition = 1.0;
                sleep(200);
            }
            lastA = a;

            // Retract button
            if (b && !lastB) {
                funnel.retract();
                manualPosition = 0.0;
                sleep(200);
            }
            lastB = b;

            // Toggle button
            if (x && !lastX) {
                funnel.toggle();
                manualPosition = funnel.isExtended() ? 1.0 : 0.0;
                sleep(200);
            }
            lastX = x;

            // D-Pad position adjustment (both servos synchronized)
            if (dpadUp && !lastDpadUp) {
                manualPosition = Math.min(1.0, manualPosition + 0.05);
                funnel.setPosition(manualPosition);
                sleep(100);
            }
            lastDpadUp = dpadUp;

            if (dpadDown && !lastDpadDown) {
                manualPosition = Math.max(0.0, manualPosition - 0.05);
                funnel.setPosition(manualPosition);
                sleep(100);
            }
            lastDpadDown = dpadDown;

            // Manual position control with left stick (both servos)
            double leftStickY = -gamepad1.left_stick_y;
            if (Math.abs(leftStickY) > 0.1) {
                // Map stick position (-1 to 1) to servo position (0 to 1)
                manualPosition = (leftStickY + 1.0) / 2.0;
                funnel.setSyncPosition(manualPosition);
            }

            // Individual servo control with right stick
            double rightStickY = -gamepad1.right_stick_y;
            double rightStickX = gamepad1.right_stick_x;
            if (Math.abs(rightStickY) > 0.1 || Math.abs(rightStickX) > 0.1) {
                // Map stick positions to servo positions
                double leftPos = (rightStickY + 1.0) / 2.0;
                double rightPos = (rightStickX + 1.0) / 2.0;
                funnel.setSyncPosition(leftPos, rightPos);
                manualPosition = (leftPos + rightPos) / 2.0;
            }

            // Update telemetry
            telemetry.addData("=== FUNNEL TEST ===", "");
            telemetry.addData("Status", funnel.isExtended() ? "EXTENDED" : "RETRACTED");
            telemetry.addData("Left Servo Position", "%.3f", funnel.getLeftPosition());
            telemetry.addData("Right Servo Position", "%.3f", funnel.getRightPosition());
            telemetry.addData("Manual Position", "%.3f", manualPosition);
            telemetry.addLine();
            telemetry.addLine("Controls:");
            telemetry.addData("A", "Extend");
            telemetry.addData("B", "Retract");
            telemetry.addData("X", "Toggle");
            telemetry.addData("D-Pad Up/Down", "Adjust position");
            telemetry.addData("Left Stick Y", "Manual position (both)");
            telemetry.addData("Right Stick Y/X", "Individual control");
            
            funnel.updateTelemetry();
            telemetry.update();

            sleep(20);
        }

        // Retract funnels when OpMode ends
        funnel.retract();
    }
}

