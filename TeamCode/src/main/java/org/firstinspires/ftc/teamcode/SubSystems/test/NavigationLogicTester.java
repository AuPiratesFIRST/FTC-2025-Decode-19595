package org.firstinspires.ftc.teamcode.SubSystems.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.SubSystems.Drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Drive.TileCoordinate;
import org.firstinspires.ftc.teamcode.SubSystems.Drive.TileNavigator;
import org.firstinspires.ftc.teamcode.SubSystems.Vision.AprilTagNavigator;

/**
 * Navigation Logic Tester for DECODE Tile-Based Navigation
 * 
 * Tests square-by-square accuracy of tile navigation system using AprilTag localization.
 * 
 * Controls:
 * - Gamepad1 Y: Update robot position from AprilTag alliance goals
 * - Gamepad1 D-Pad Up: Move exactly 1 tile (24") forward
 * - Gamepad1 D-Pad Right: Move exactly 1 tile (24") right
 * - Gamepad1 D-Pad Down: Move exactly 1 tile (24") backward
 * - Gamepad1 D-Pad Left: Move exactly 1 tile (24") left
 * 
 * Verification:
 * - Place robot at Tab V1 (field corner 0,0) facing right
 * - Press Y to calibrate with AprilTags
 * - Use D-Pad to move square-by-square
 * - Telemetry shows Tile Position (A1, B1, etc.), Tab Position (V1, W1, etc.), and offsets
 * 
 * Expected Behavior:
 * - Origin Point (V1): Tab: V1, Tile: A1, X: 0.0, Y: 0.0
 * - Center of B1: Tile: B1, Offset: 12, 12, X: 36.0, Y: 12.0
 * - Tab W1: X: 24.0, Y: 0.0
 */
@TeleOp(name = "Navigation Logic Tester", group = "Test")
public class NavigationLogicTester extends LinearOpMode {
    DriveSubsystem drive;
    TileNavigator navigator;
    AprilTagNavigator aprilTag;

    @Override
    public void runOpMode() {
        drive = new DriveSubsystem(hardwareMap, telemetry);
        navigator = new TileNavigator(drive, telemetry);
        aprilTag = new AprilTagNavigator(drive, hardwareMap, telemetry);

        telemetry.addData("Status", "Place robot at Tab V1 (0,0) facing Right");
        telemetry.addData("Controls", "Y = Localize with AprilTags");
        telemetry.addData("Controls", "D-Pad = Move 1 tile (24\")");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // STEP 1: TEST TILE LOCALIZATION
            // Point the camera at a Goal Tag (ID 20 or 24)
            if (gamepad1.y) {
                aprilTag.updateRobotPositionFromAllianceGoals();
                telemetry.addData("Localization", "Updated from AprilTags");
                telemetry.update();
                sleep(500);
            }

            // STEP 2: TEST SQUARE-BY-SQUARE MOVEMENT
            // Move exactly 1 tile (24 inches) in each direction
            if (gamepad1.dpad_up) {
                moveOneSquare('U'); 
            }
            if (gamepad1.dpad_right) {
                moveOneSquare('R');
            }
            if (gamepad1.dpad_down) {
                moveOneSquare('D');
            }
            if (gamepad1.dpad_left) {
                moveOneSquare('L');
            }

            // Telemetry Verification
            displayNavigationDebug();
            sleep(100);
        }
    }

    /**
     * Move exactly one tile (24 inches) in the specified direction.
     * After moving, re-verify position with AprilTags to correct any drift.
     * 
     * @param dir Direction: 'U'=Up, 'R'=Right, 'D'=Down, 'L'=Left
     */
    private void moveOneSquare(char dir) {
        double dist = 24.0; // Standard Tile Size (DECODE field)
        
        telemetry.addData("Moving", "Direction: %c, Distance: %.1f\"", dir, dist);
        telemetry.update();
        
        if (dir == 'U') drive.moveInches(dist, 0.5);    // Forward
        if (dir == 'R') drive.strafeInches(dist, 0.5);  // Right
        if (dir == 'D') drive.moveInches(-dist, 0.5);   // Backward
        if (dir == 'L') drive.strafeInches(-dist, 0.5); // Left
        
        // IMPORTANT: After moving, we re-verify with AprilTags
        // This corrects any encoder drift and provides "ground truth"
        sleep(500); // Wait for robot to settle
        aprilTag.updateRobotPositionFromAllianceGoals();
        
        telemetry.addData("Move Complete", "Position updated from AprilTags");
        telemetry.update();
        sleep(300);
    }

    /**
     * Display comprehensive navigation debug information.
     * Shows tile position, tab intersection, raw coordinates, and physical verification hints.
     */
    private void displayNavigationDebug() {
        TileCoordinate pos = navigator.getCurrentPosition();
        telemetry.addLine("=== LOCALIZATION CHECK ===");
        telemetry.addData("Grid Position", pos.getTilePosition()); // Should be A1, B1, etc.
        telemetry.addData("Tab Intersection", pos.getTabPosition()); // Should be V1, W1, etc.
        telemetry.addData("Raw X (Inches)", "%.2f", pos.getX());
        telemetry.addData("Raw Y (Inches)", "%.2f", pos.getY());
        
        double[] offset = pos.getTileOffset();
        telemetry.addData("Offset in Tile", "X:%.1f Y:%.1f", offset[0], offset[1]);
        
        telemetry.addData("Current Heading", "%.1f degrees", Math.toDegrees(drive.getHeading()));
        
        telemetry.addLine("\n=== PHYSICAL VERIFICATION ===");
        telemetry.addLine("If Robot is on Tab W1, Telemetry should say: W1");
        telemetry.addLine("If Robot is in center of Tile B2, Telemetry should say: B2, Offset 12,12");
        telemetry.addLine("\n=== EXPECTED VALUES ===");
        telemetry.addLine("Origin (V1/A1): X=0.0, Y=0.0");
        telemetry.addLine("Center of B1: X=36.0, Y=12.0, Offset 12,12");
        telemetry.addLine("Tab W1: X=24.0, Y=0.0");
        telemetry.update();
    }
}
