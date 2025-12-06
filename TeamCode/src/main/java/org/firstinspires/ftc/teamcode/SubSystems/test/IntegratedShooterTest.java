package org.firstinspires.ftc.teamcode.SubSystems.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SubSystems.Drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Vision.AprilTagNavigator;
import org.firstinspires.ftc.teamcode.SubSystems.Shooter.IntegratedShooterSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Spindexer.OldSpindexerSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Drive.TileCoordinate;

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

        driveSubsystem = new DriveSubsystem(hardwareMap, telemetry);
        aprilTagNavigator = new AprilTagNavigator(driveSubsystem, hardwareMap, telemetry);
        shooter = new IntegratedShooterSubsystem(hardwareMap, telemetry);
        spindexer = new OldSpindexerSubsystem(hardwareMap, telemetry);

        telemetry.addData("Status", "Ready");
        telemetry.addLine();
        telemetry.addLine("=== CONTROLS ===");
        telemetry.addLine("Right Trigger: Auto-shoot");
        telemetry.addLine("Left Trigger: Manual power");
        telemetry.addLine("A/B/X: Spindexer positions");
        telemetry.addLine("Y: Toggle Alliance");
        telemetry.addLine("D-Pad Up/Down: Adjust manual power");
        telemetry.addLine("RB: Stop shooter");
        telemetry.addLine("LB: Update localization");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Update subsystems
            spindexer.update();
            aprilTagNavigator.updateRobotPositionFromTriangulation();

            TileCoordinate robotPosition = driveSubsystem.getCurrentPosition();
            TileCoordinate goalPosition = IntegratedShooterSubsystem.getGoalPosition(isBlueAlliance);

            // Get distance to goal (safe)
            double distanceToGoal = (robotPosition != null && goalPosition != null)
                    ? robotPosition.distanceTo(goalPosition)
                    : 0.0;

            // FIXED SYNTAX ERROR (added closing parenthesis)
            double calculatedPower = shooter.calculateMotorPower(distanceToGoal);

            // ===== CONTROLS =====

            // Auto shoot
            if (gamepad1.right_trigger > 0.1) {
                if (robotPosition != null) {
                    shooter.shootArtifact(robotPosition, isBlueAlliance);
                } else {
                    telemetry.addData("Error", "Robot not localized!");
                }
            }

            // Manual shoot power
            if (gamepad1.left_trigger > 0.1) {
                shooter.setPower(manualPower);
            }

            if (gamepad1.dpad_up) {
                manualPower = Math.min(1.0, manualPower + 0.01);
                sleep(50);
            }
            if (gamepad1.dpad_down) {
                manualPower = Math.max(0.0, manualPower - 0.01);
                sleep(50);
            }

            if (gamepad1.right_bumper) {
                shooter.stop();
                sleep(150);
            }

            // Force AprilTag update
            if (gamepad1.left_bumper) {
                aprilTagNavigator.updateRobotPositionFromTriangulation();
                sleep(150);
            }

            // Spindexer position A
            if (gamepad1.a && !lastA) {
                spindexer.goToPosition(OldSpindexerSubsystem.SpindexerPosition.POSITION_1);
            }
            lastA = gamepad1.a;

            // Spindexer position B
            if (gamepad1.b && !lastB) {
                spindexer.goToPosition(OldSpindexerSubsystem.SpindexerPosition.POSITION_2);
            }
            lastB = gamepad1.b;

            // Spindexer position X
            if (gamepad1.x && !lastX) {
                spindexer.goToPosition(OldSpindexerSubsystem.SpindexerPosition.POSITION_3);
            }
            lastX = gamepad1.x;

            // Alliance toggle
            if (gamepad1.y && !lastY) {
                isBlueAlliance = !isBlueAlliance;
                sleep(200);
            }
            lastY = gamepad1.y;

            // ========== TELEMETRY ==========

            telemetry.addLine("=== INTEGRATED SHOOTER TEST ===");

            telemetry.addLine("\n--- Localizaton ---");
            telemetry.addData("Localized", aprilTagNavigator.isLocalized());
            if (robotPosition != null) {
                telemetry.addData("Robot Tile", robotPosition.getTilePosition());
                telemetry.addData("Robot Inches", "X: %.1f, Y: %.1f",
                        robotPosition.getX(), robotPosition.getY());
            } else {
                telemetry.addData("Robot Position", "NO DATA");
            }

            telemetry.addLine("\n--- Goal ---");
            telemetry.addData("Alliance", isBlueAlliance ? "BLUE" : "RED");
            if (goalPosition != null) {
                telemetry.addData("Goal", "X: %.1f, Y: %.1f",
                        goalPosition.getX(), goalPosition.getY());
            }
            telemetry.addData("Distance to Goal", "%.1f in", distanceToGoal);

            telemetry.addLine("\n--- Shooter ---");
            telemetry.addData("Auto Power", "%.2f", calculatedPower);
            telemetry.addData("Manual Power", "%.2f", manualPower);
            shooter.updateTelemetry();

            telemetry.addLine("\n--- Spindexer ---");
            telemetry.addData("Current", spindexer.getCurrentPosition());
            telemetry.addData("Target", spindexer.getTargetPosition());
            telemetry.addData("At Target", spindexer.isAtPosition());

            telemetry.update();

            sleep(20);
        }

        shooter.stop();
        spindexer.update();
    }
}
