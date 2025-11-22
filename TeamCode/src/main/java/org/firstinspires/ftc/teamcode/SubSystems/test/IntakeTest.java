package org.firstinspires.ftc.teamcode.SubSystems.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SubSystems.Intake.IntakeSubsystem;

/**
 * Test OpMode for Intake subsystem.
 * Allows testing of forward and reverse intake operation.
 * 
 * Controls:
 * - Right Trigger: Forward intake
 * - Left Trigger: Reverse/outtake
 * - A Button: Toggle forward intake
 * - B Button: Toggle reverse intake
 * - X Button: Stop intake
 * - Right Stick Y: Manual power control
 */
@TeleOp(name = "Intake Test", group = "Test")
public class IntakeTest extends LinearOpMode {

    private IntakeSubsystem intake;
    private boolean forwardToggle = false;
    private boolean reverseToggle = false;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initializing Intake Test...");
        telemetry.update();

        // Initialize intake subsystem
        intake = new IntakeSubsystem(hardwareMap, telemetry);

        telemetry.addData("Status", "Ready");
        telemetry.addLine();
        telemetry.addLine("Controls:");
        telemetry.addLine("Right Trigger: Forward intake");
        telemetry.addLine("Left Trigger: Reverse/outtake");
        telemetry.addLine("A: Toggle forward");
        telemetry.addLine("B: Toggle reverse");
        telemetry.addLine("X: Stop");
        telemetry.addLine("Right Stick Y: Manual power");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Trigger controls (hold to run)
            if (gamepad1.right_trigger > 0.1) {
                intake.start();
                forwardToggle = false;
                reverseToggle = false;
            } else if (gamepad1.left_trigger > 0.1) {
                intake.reverse();
                forwardToggle = false;
                reverseToggle = false;
            } else {
                // Toggle controls
                if (gamepad1.a) {
                    forwardToggle = !forwardToggle;
                    reverseToggle = false;
                    if (forwardToggle) {
                        intake.start();
                    } else {
                        intake.stop();
                    }
                    sleep(200);
                }
                if (gamepad1.b) {
                    reverseToggle = !reverseToggle;
                    forwardToggle = false;
                    if (reverseToggle) {
                        intake.reverse();
                    } else {
                        intake.stop();
                    }
                    sleep(200);
                }
                if (gamepad1.x) {
                    intake.stop();
                    forwardToggle = false;
                    reverseToggle = false;
                    sleep(200);
                }

                // Manual power control with right stick
                double manualPower = -gamepad1.right_stick_y;
                if (Math.abs(manualPower) > 0.1) {
                    intake.setPower(manualPower);
                    forwardToggle = false;
                    reverseToggle = false;
                } else if (!forwardToggle && !reverseToggle && Math.abs(gamepad1.right_trigger) < 0.1
                        && Math.abs(gamepad1.left_trigger) < 0.1) {
                    // Only stop if not using toggles or triggers
                    intake.stop();
                }
            }

            // Update telemetry
            telemetry.addData("Status", intake.isRunning() ? "ACTIVE" : "STOPPED");
            intake.updateTelemetry();
            telemetry.update();

            sleep(20);
        }

        // Stop intake when OpMode ends
        intake.stop();
    }
}

