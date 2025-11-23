package org.firstinspires.ftc.teamcode.SubSystems.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;

import org.firstinspires.ftc.teamcode.SubSystems.Drive.TileCoordinate;
import org.firstinspires.ftc.teamcode.SubSystems.Drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Vision.AprilTagNavigator;
import com.acmerobotics.dashboard.canvas.Canvas;

@Configurable
@TeleOp(name = "Move To Tile Test", group = "Test")
public class MoveToTile extends LinearOpMode {

    // Configurable target tile via Panels
    public char targetColumn = 'C';
    public int targetRow = 3;
    public double targetOffsetX = 12.0; // Offset within tile (0-24 inches)
    public double targetOffsetY = 12.0; // Offset within tile (0-24 inches)
    public double movePower = 0.5; // Motor power for movement (0-1)
    public boolean autoMove = false; // Automatically move when target changes
    public double positionTolerance = 6.0; // Distance tolerance in inches

    @IgnoreConfigurable
    private DriveSubsystem driveSubsystem;
    
    @IgnoreConfigurable
    private AprilTagNavigator navigator;
    
    @IgnoreConfigurable
    private FtcDashboard dashboard;
    
    @IgnoreConfigurable
    private TileCoordinate currentTarget;
    
    @IgnoreConfigurable
    private boolean isMoving = false;
    
    @IgnoreConfigurable
    private TileCoordinate lastTarget = null;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize subsystems
        driveSubsystem = null;
        try {
            driveSubsystem = new DriveSubsystem(hardwareMap, telemetry);
        } catch (Throwable t) {
            telemetry.addData("DriveSubsystem", "Not available: %s", t.getMessage());
        }

        navigator = new AprilTagNavigator(driveSubsystem, hardwareMap, telemetry);
        dashboard = FtcDashboard.getInstance();

        // Register with Panels configurables
        PanelsConfigurables.INSTANCE.refreshClass(this);

        if (driveSubsystem != null) {
            // Set a reasonable starting pose (center of A1, facing +X)
            driveSubsystem.setPosition(new TileCoordinate('A', 1, 12, 12));
            driveSubsystem.setHeading(0.0);
        }

        telemetry.addLine("Move To Tile Test");
        telemetry.addLine("Configure target tile in Panels Dashboard");
        telemetry.addLine("Press A to move to target tile");
        telemetry.addLine("Press START to begin");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Update localization from vision
            boolean updated = false;
            if (!updated) updated = navigator.updateRobotPositionFromTriangulation();
            if (!updated) updated = navigator.updateRobotPositionFromAllianceGoals();
            if (!updated) updated = navigator.updateRobotPosition();

            navigator.updateDECODELocalizationTelemetry();

            // Update target tile from configurables
            updateTargetTile();

            // Check if target changed and auto-move is enabled
            if (autoMove && currentTarget != null && !currentTarget.equals(lastTarget)) {
                startMovingToTarget();
                lastTarget = currentTarget;
            }

            // Manual move trigger (gamepad A button)
            if (gamepad1.a && !isMoving && currentTarget != null) {
                startMovingToTarget();
            }

            // Update movement if currently moving
            if (isMoving && currentTarget != null) {
                updateMovement();
            }

            // Manual drive override (gamepad left stick)
            if (Math.abs(gamepad1.left_stick_y) > 0.1 || Math.abs(gamepad1.left_stick_x) > 0.1 || Math.abs(gamepad1.right_stick_x) > 0.1) {
                // Cancel movement if manually driving
                isMoving = false;
                if (driveSubsystem != null) {
                    double forward = -gamepad1.left_stick_y;
                    double strafe = gamepad1.left_stick_x;
                    double turn = gamepad1.right_stick_x;
                    driveSubsystem.drive(forward, strafe, turn);
                }
            } else if (!isMoving && driveSubsystem != null) {
                // Stop motors if not moving
                driveSubsystem.stop();
            }

            // Draw on dashboard
            drawDashboard();

            // Update telemetry
            updateTelemetry();

            telemetry.update();
            sleep(50);
        }

        if (driveSubsystem != null) {
            driveSubsystem.stop();
        }
        navigator.closeVision();
    }

    private void updateTargetTile() {
        // Clamp column to valid range (A-F)
        if (targetColumn < 'A' || targetColumn > 'F') {
            targetColumn = 'C';
        }
        
        // Clamp row to valid range (1-6)
        if (targetRow < 1 || targetRow > 6) {
            targetRow = 3;
        }
        
        // Clamp offsets to valid range (0-24)
        targetOffsetX = Math.max(0, Math.min(24, targetOffsetX));
        targetOffsetY = Math.max(0, Math.min(24, targetOffsetY));
        
        // Clamp power to valid range (0-1)
        movePower = Math.max(0, Math.min(1, movePower));
        
        // Create target coordinate
        currentTarget = new TileCoordinate(targetColumn, targetRow, targetOffsetX, targetOffsetY);
    }

    private void startMovingToTarget() {
        if (driveSubsystem == null || currentTarget == null) {
            return;
        }
        
        isMoving = true;
        telemetry.addLine("Starting movement to " + currentTarget.getTilePosition());
    }

    private void updateMovement() {
        if (driveSubsystem == null || currentTarget == null) {
            isMoving = false;
            return;
        }

        TileCoordinate currentPos = driveSubsystem.getCurrentPosition();
        if (currentPos == null) {
            return;
        }

        // Check if we've reached the target
        double distance = currentPos.distanceTo(currentTarget);
        if (distance <= positionTolerance) {
            isMoving = false;
            driveSubsystem.stop();
            telemetry.addLine("Reached target tile!");
            return;
        }

        // Calculate movement needed
        double angle = currentPos.angleTo(currentTarget);
        double currentHeading = driveSubsystem.getCurrentHeading();
        
        // Calculate turn needed
        double turnAngle = angle - currentHeading;
        // Normalize to [-π, π]
        while (turnAngle > Math.PI) turnAngle -= 2 * Math.PI;
        while (turnAngle < -Math.PI) turnAngle += 2 * Math.PI;
        
        // Calculate forward and strafe components
        double forward = Math.cos(angle) * movePower;
        double strafe = Math.sin(angle) * movePower;
        
        // Scale turn based on angle error
        double turn = Math.signum(turnAngle) * Math.min(Math.abs(turnAngle) / Math.PI, 1.0) * movePower * 0.5;
        
        // Apply movement
        driveSubsystem.drive(forward, strafe, turn);
    }

    private void drawDashboard() {
        if (driveSubsystem == null) {
            return;
        }

        TileCoordinate pos = driveSubsystem.getCurrentPosition();
        double heading = driveSubsystem.getCurrentHeading();
        
        if (pos != null) {
            // Convert TileCoordinate (0..144, bottom-left origin) -> Field View frame (origin at center)
            double x = pos.getX();
            double y = pos.getY();
            double fx = x - 72.0;
            double fy = y - 72.0;
            
            TelemetryPacket packet = new TelemetryPacket();
            Canvas overlay = packet.fieldOverlay();
            
            // Draw grid
            overlay.drawGrid(0, 0, 144, 144, 7, 7);
            
            // Draw robot position
            overlay.setStroke("#00AA00");
            overlay.strokeCircle(fx, fy, 2.0);
            
            // Draw heading arrow
            double arrowLen = 8.0;
            double x2 = fx + Math.cos(heading) * arrowLen;
            double y2 = fy + Math.sin(heading) * arrowLen;
            overlay.strokeLine(fx, fy, x2, y2);
            
            // Draw target tile if set
            if (currentTarget != null) {
                double tx = currentTarget.getX() - 72.0;
                double ty = currentTarget.getY() - 72.0;
                
                // Draw target as a rectangle
                overlay.setStroke("#FF0000");
                overlay.setFill("#FF0000");
                overlay.setAlpha(0.3);
                overlay.fillRect(tx - 12, ty - 12, 24, 24);
                overlay.setAlpha(1.0);
                overlay.strokeRect(tx - 12, ty - 12, 24, 24);
                
                // Draw line from robot to target
                overlay.setStroke("#FFFF00");
                overlay.strokeLine(fx, fy, tx, ty);
                
                // Draw target label
                overlay.setFill("#FFFFFF");
                overlay.fillText(currentTarget.getTilePosition(), tx + 2.5, ty + 2.5, "12px Arial", 0.0, false);
            }
            
            // Draw robot label
            overlay.setFill("#FFFFFF");
            overlay.fillText(pos.getTilePosition(), fx + 2.5, fy - 5.5, "10px Arial", 0.0, false);
            
            // Draw status
            String status = isMoving ? "MOVING" : "IDLE";
            overlay.fillText(status, fx - 15, fy + 5.5, "10px Arial", 0.0, false);
            
            dashboard.sendTelemetryPacket(packet);
        }
    }

    private void updateTelemetry() {
        telemetry.addLine("=== Move To Tile Test ===");
        
        if (currentTarget != null) {
            telemetry.addData("Target Tile", currentTarget.getTilePosition());
            telemetry.addData("Target Position", "X: %.1f, Y: %.1f", currentTarget.getX(), currentTarget.getY());
        }
        
        if (driveSubsystem != null) {
            TileCoordinate currentPos = driveSubsystem.getCurrentPosition();
            if (currentPos != null) {
                telemetry.addData("Current Tile", currentPos.getTilePosition());
                telemetry.addData("Current Position", "X: %.1f, Y: %.1f", currentPos.getX(), currentPos.getY());
                
                if (currentTarget != null) {
                    double distance = currentPos.distanceTo(currentTarget);
                    telemetry.addData("Distance to Target", "%.1f inches", distance);
                    telemetry.addData("Status", isMoving ? "Moving..." : "Ready");
                }
            }
        }
        
        telemetry.addLine("");
        telemetry.addLine("Controls:");
        telemetry.addLine("A - Move to target tile");
        telemetry.addLine("Left Stick - Manual drive (cancels auto)");
        telemetry.addLine("Configure target in Panels Dashboard");
    }
}

