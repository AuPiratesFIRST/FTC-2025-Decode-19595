package org.firstinspires.ftc.teamcode.SubSystems.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SubSystems.Drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Vision.AprilTagNavigator;
import org.firstinspires.ftc.teamcode.SubSystems.Shooter.IntegratedShooterSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Spindexer.OldSpindexerSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Drive.TileCoordinate;

/**
 * Integrated test for shooter with AprilTag localization and spindexer.
 * 
 * Tests the complete shooting system:
 * - AprilTag localization to get robot position
 * - Distance calculation from robot to goal
 * - Shot profile power calculation based on distance
 * - Shooter velocity control with RPM targeting
 * - Spindexer integration for artifact positioning
 * 
 * Controls:
 * - Right Trigger: Shoot (calculates distance and sets power automatically)
 * - Left Trigger: Manual power adjustment
 * - A Button: Toggle spindexer position 1
 * - B Button: Toggle spindexer position 2
 * - X Button: Toggle spindexer position 3
 * - Y Button: Toggle alliance (Blue/Red)
 * - D-Pad Up: Increase manual power
 * - D-Pad Down: Decrease manual power
 * - Right Bumper: Stop shooter
 * - Left Bumper: Update localization from AprilTag
 */
@TeleOp(name = "Integrated Shooter Test", group = "Test")
public class IntegratedShooterTest extends LinearOpMode {

    private DriveSubsystem driveSubsystem;
    private AprilTagNavigator aprilTagNavigator;
    private IntegratedShooterSubsystem shooter;
    private OldSpindexerSubsystem spindexer;

    private boolean isBlueAlliance = true;
    private double manualPower = 0.7;
    private boolean lastA = false;
    private boolean lastB = false;
    private boolean lastX = false;
    private boolean lastY = false;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initializing Integrated Shooter Test...");
        telemetry.update();

        // Initialize subsystems
        driveSubsystem = new DriveSubsystem(hardwareMap, telemetry);
        aprilTagNavigator = new AprilTagNavigator(driveSubsystem, hardwareMap, telemetry);
        shooter = new IntegratedShooterSubsystem(hardwareMap, telemetry);
        spindexer = new OldSpindexerSubsystem(hardwareMap, telemetry);

        telemetry.addData("Status", "Ready");
        telemetry.addLine();
        telemetry.addLine("=== CONTROLS ===");
        telemetry.addLine("Right Trigger: Auto-shoot (distance-based)");
        telemetry.addLine("Left Trigger: Manual power adjustment");
        telemetry.addLine("A: Spindexer Position 1");
        telemetry.addLine("B: Spindexer Position 2");
        telemetry.addLine("X: Spindexer Position 3");
        telemetry.addLine("Y: Toggle Alliance (Blue/Red)");
        telemetry.addLine("D-Pad Up: Increase manual power");
        telemetry.addLine("D-Pad Down: Decrease manual power");
        telemetry.addLine("Right Bumper: Stop shooter");
        telemetry.addLine("Left Bumper: Update localization");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Update spindexer PID control
            spindexer.update();

            // Update AprilTag localization (continuous)
            aprilTagNavigator.updateRobotPositionFromTriangulation();

            // Get robot position from localization
            TileCoordinate robotPosition = driveSubsystem.getCurrentPosition();
            TileCoordinate goalPosition = IntegratedShooterSubsystem.getGoalPosition(isBlueAlliance);

            // Calculate distance to goal
            double distanceToGoal = robotPosition != null && goalPosition != null
                    ? robotPosition.distanceTo(goalPosition)
                    : 0.0;

            // Calculate power from shot profile based on distance
            double calculatedPower = shooter.calculateMotorPower(distanceToGoal,
                    IntegratedShooterSubsystem.GOAL_HEIGHT_INCHES);

            // ==================== CONTROLS ====================

            // Right Trigger: Auto-shoot using distance-based power
            if (gamepad1.right_trigger > 0.1) {
                if (robotPosition != null) {
                    shooter.shootArtifact(robotPosition, isBlueAlliance);
                } else {
                    telemetry.addData("Error", "Robot position not localized!");
                }
            }

            // Left Trigger: Manual power adjustment
            if (gamepad1.left_trigger > 0.1) {
                shooter.setPower(manualPower);
            }

            // D-Pad Up: Increase manual power
            if (gamepad1.dpad_up) {
                manualPower = Math.min(1.0, manualPower + 0.01);
                sleep(50); // Debounce
            }

            // D-Pad Down: Decrease manual power
            if (gamepad1.dpad_down) {
                manualPower = Math.max(0.0, manualPower - 0.01);
                sleep(50); // Debounce
            }

            // Right Bumper: Stop shooter
            if (gamepad1.right_bumper) {
                shooter.stop();
                sleep(200); // Debounce
            }

            // Left Bumper: Force localization update
            if (gamepad1.left_bumper) {
                aprilTagNavigator.updateRobotPositionFromTriangulation();
                sleep(200); // Debounce
            }

            // Spindexer Position Controls
            if (gamepad1.a && !lastA) {
                spindexer.goToPosition(OldSpindexerSubsystem.SpindexerPosition.POSITION_1);
            }
            lastA = gamepad1.a;

            if (gamepad1.b && !lastB) {
                spindexer.goToPosition(OldSpindexerSubsystem.SpindexerPosition.POSITION_2);
            }
            lastB = gamepad1.b;

            if (gamepad1.x && !lastX) {
                spindexer.goToPosition(OldSpindexerSubsystem.SpindexerPosition.POSITION_3);
            }
            lastX = gamepad1.x;

            // Toggle Alliance
            if (gamepad1.y && !lastY) {
                isBlueAlliance = !isBlueAlliance;
                sleep(200); // Debounce
            }
            lastY = gamepad1.y;

            // ==================== TELEMETRY ====================

            telemetry.addData("=== INTEGRATED SHOOTER TEST ===", "");
            telemetry.addLine();

            // Localization Info
            telemetry.addData("=== LOCALIZATION ===", "");
            boolean isLocalized = aprilTagNavigator.isLocalized();
            telemetry.addData("Localized", isLocalized ? "YES ✓" : "NO ✗");
            if (robotPosition != null) {
                telemetry.addData("Robot Position", robotPosition.getTilePosition());
                telemetry.addData("Robot (inches)", "X: %.1f, Y: %.1f",
                        robotPosition.getX(), robotPosition.getY());
            } else {
                telemetry.addData("Robot Position", "NOT LOCALIZED");
            }

            // Goal Info
            telemetry.addData("Alliance", isBlueAlliance ? "BLUE" : "RED");
            if (goalPosition != null) {
                telemetry.addData("Goal Position", "X: %.1f, Y: %.1f",
                        goalPosition.getX(), goalPosition.getY());
            }
            telemetry.addData("Distance to Goal", "%.1f inches", distanceToGoal);
            telemetry.addLine();

            // Shot Profile Info
            telemetry.addData("=== SHOT PROFILE ===", "");
            telemetry.addData("Calculated Power", "%.2f (from distance)", calculatedPower);
            telemetry.addData("Manual Power", "%.2f", manualPower);
            telemetry.addLine();

            // Simulation Comparison (Distance-based vs Physics-based)
            shooter.updateSimulationTelemetry(distanceToGoal, IntegratedShooterSubsystem.GOAL_HEIGHT_INCHES);
            telemetry.addLine();

            // Shooter Info
            telemetry.addData("=== SHOOTER ===", "");
            shooter.updateTelemetry();
            telemetry.addLine();

            // Spindexer Info
            telemetry.addData("=== SPINDEXER ===", "");
            telemetry.addData("Current Position", spindexer.getCurrentPosition());
            telemetry.addData("Target Position", spindexer.getTargetPosition());
            telemetry.addData("At Position", spindexer.isAtPosition() ? "YES ✓" : "NO ✗");
            telemetry.addLine();

            // Controls Status
            telemetry.addData("=== CONTROLS ===", "");
            telemetry.addData("Right Trigger", gamepad1.right_trigger > 0.1 ? "ACTIVE (Auto-shoot)" : "Inactive");
            telemetry.addData("Left Trigger", gamepad1.left_trigger > 0.1 ? "ACTIVE (Manual)" : "Inactive");
            telemetry.addData("D-Pad", gamepad1.dpad_up ? "UP" : gamepad1.dpad_down ? "DOWN" : "None");

            telemetry.update();
            sleep(20);
        }

        // Cleanup
        shooter.stop();
        spindexer.update(); // Final update
    }
}
