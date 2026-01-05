package org.firstinspires.ftc.teamcode.SubSystems.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SubSystems.Shooter.IntegratedShooterSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Drive.TileCoordinate;

/**
 * Test OpMode for IntegratedShooterSubsystem
 * 
 * Controls:
 * - Left Trigger (LT): Spin shooter at interpolated RPM based on test distance
 * - Right Trigger (RT): Stop shooter
 * - D-Pad Up: Increase test distance
 * - D-Pad Down: Decrease test distance
 * - A Button: Test auto aim for Blue alliance
 * - B Button: Test auto aim for Red alliance
 */
@TeleOp(name = "Integrated Shooter Test", group = "Test")
public class IntegratedShooterSubsystemTest extends LinearOpMode {

    private IntegratedShooterSubsystem shooter;
    private double testDistance = 60.0; // Starting test distance in inches
    private static final double DISTANCE_STEP = 5.0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initializing Shooter...");
        telemetry.update();

        // Initialize the shooter subsystem
        shooter = new IntegratedShooterSubsystem(hardwareMap, telemetry);

        telemetry.addData("Status", "Shooter initialized successfully!");
        telemetry.addData("Instructions", "Ready for testing");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // ========== MANUAL RPM CONTROL ==========
            if (gamepad1.left_trigger > 0.1) {
                // LT: Spin at RPM based on test distance
                double rpmToSet = calculateRPMForDistance(testDistance);
                shooter.setTargetRPM(rpmToSet);
            } else if (gamepad1.right_trigger > 0.1) {
                // RT: Stop shooter
                shooter.stop();
            }

            // ========== DISTANCE ADJUSTMENT ==========
            if (gamepad1.dpad_up) {
                testDistance += DISTANCE_STEP;
                sleep(100); // Debounce
            }
            if (gamepad1.dpad_down) {
                testDistance -= DISTANCE_STEP;
                testDistance = Math.max(testDistance, 0); // Prevent negative distance
                sleep(100); // Debounce
            }

            // ========== AUTO AIM TESTS ==========
            if (gamepad1.a) {
                // Test Blue alliance auto aim
                TileCoordinate testRobotPos = new TileCoordinate(0, 0); // Robot at origin
                shooter.autoAimAndSpin(testRobotPos, true); // true = Blue alliance
                sleep(200); // Debounce
            }

            if (gamepad1.b) {
                // Test Red alliance auto aim
                TileCoordinate testRobotPos = new TileCoordinate(0, 0); // Robot at origin
                shooter.autoAimAndSpin(testRobotPos, false); // false = Red alliance
                sleep(200); // Debounce
            }

            // ========== VOLTAGE COMPENSATION UPDATE ==========
            shooter.updateVoltageCompensation();

            // ========== TELEMETRY UPDATES ==========
            updateTestTelemetry();

            telemetry.update();
        }

        shooter.stop();
    }

    /**
     * Calculates RPM for a given distance using linear interpolation.
     * Mirrors the logic in the shooter subsystem.
     */
    private double calculateRPMForDistance(double distance) {
        double[][] shotProfile = new double[][]{
                {40, 3200},
                {72, 3800},
                {100, 4400},
                {125, 4850},
                {140, 5200}
        };

        if (distance <= shotProfile[0][0]) return shotProfile[0][1];
        if (distance >= shotProfile[shotProfile.length - 1][0]) {
            return shotProfile[shotProfile.length - 1][1];
        }

        for (int i = 0; i < shotProfile.length - 1; i++) {
            if (distance < shotProfile[i + 1][0]) {
                double d0 = shotProfile[i][0];
                double d1 = shotProfile[i + 1][0];
                double r0 = shotProfile[i][1];
                double r1 = shotProfile[i + 1][1];

                return r0 + (distance - d0) * ((r1 - r0) / (d1 - d0));
            }
        }
        return shotProfile[shotProfile.length - 1][1];
    }

    private void updateTestTelemetry() {
        telemetry.addLine("========== SHOOTER TEST ==========");
        telemetry.addData("Test Distance", "%.1f inches", testDistance);
        telemetry.addData("Calculated RPM", "%.0f", calculateRPMForDistance(testDistance));
        telemetry.addLine();

        shooter.updateTelemetry();

        telemetry.addLine();
        telemetry.addLine("========== CONTROLS ==========");
        telemetry.addData("LT", "Spin at test distance RPM");
        telemetry.addData("RT", "Stop shooter");
        telemetry.addData("D-Pad Up/Down", "Adjust test distance");
        telemetry.addData("A Button", "Auto aim Blue alliance");
        telemetry.addData("B Button", "Auto aim Red alliance");
        telemetry.addLine();

        if (shooter.isAtTargetRPM()) {
            telemetry.addData("Status", "READY TO FIRE");
        } else {
            telemetry.addData("Status", "WARMING UP...");
        }
    }
}
