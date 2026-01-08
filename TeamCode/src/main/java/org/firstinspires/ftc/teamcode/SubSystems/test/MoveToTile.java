package org.firstinspires.ftc.teamcode.SubSystems.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

// Panels Imports
import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.field.PanelsField;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;

import org.firstinspires.ftc.teamcode.SubSystems.Drive.TileCoordinate;
import org.firstinspires.ftc.teamcode.SubSystems.Drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Vision.AprilTagNavigator;

@Configurable
@TeleOp(name = "Move To Tile Test (Panels)", group = "Test")
public class MoveToTile extends LinearOpMode {

    // === CONFIGURABLES (Editable live in Panels) ===
    public char targetColumn = 'C';
    public int targetRow = 3;
    public double targetOffsetX = 12.0; 
    public double targetOffsetY = 12.0; 
    public double movePower = 0.5; 
    public boolean autoMove = false; 
    public double positionTolerance = 6.0; 

    // === SUBSYSTEMS ===
    @IgnoreConfigurable
    private DriveSubsystem driveSubsystem;
    
    @IgnoreConfigurable
    private AprilTagNavigator navigator;
    
    // === PANELS OBJECTS ===
    @IgnoreConfigurable
    private PanelsField panelsField;
    
    @IgnoreConfigurable
    private JoinedTelemetry telemetryM; // "M" for Multi/Manager

    // === STATE ===
    @IgnoreConfigurable
    private TileCoordinate currentTarget;
    
    @IgnoreConfigurable
    private boolean isMoving = false;
    
    @IgnoreConfigurable
    private TileCoordinate lastTarget = null;

    @Override
    public void runOpMode() throws InterruptedException {
        // 1. Initialize Joined Telemetry (Sends to both Driver Station & Panels)
        // Accessing the singleton instance via INSTANCE.getFtcTelemetry()
        telemetryM = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);

        // 2. Initialize Subsystems
        driveSubsystem = null;
        try {
            driveSubsystem = new DriveSubsystem(hardwareMap, telemetry);
        } catch (Throwable t) {
            telemetryM.addData("Error", "DriveSubsystem failed: " + t.getMessage());
        }

        navigator = new AprilTagNavigator(driveSubsystem, hardwareMap, telemetry);
        
        // 3. Initialize Panels Field
        panelsField = PanelsField.INSTANCE.getField();
        // Set coordinate system to Standard FTC (Center is 0,0)
        panelsField.setOffsets(PanelsField.presets.DEFAULT_FTC);

        // 4. Register Configurables
        PanelsConfigurables.INSTANCE.refreshClass(this);

        if (driveSubsystem != null) {
            // Set starting pose (center of A1, facing +X)
            driveSubsystem.setPosition(new TileCoordinate('A', 1, 12, 12));
            driveSubsystem.setHeading(0.0);
        }

        telemetryM.addLine("Move To Tile Test");
        telemetryM.addLine("1. Open Panels Dashboard");
        telemetryM.addLine("2. Set Target in Configurables");
        telemetryM.addLine("3. Press Start");
        telemetryM.update();

        waitForStart();

        while (opModeIsActive()) {
            // --- LOGIC ---
            
            // Localization
            boolean updated = false;
            if (!updated) updated = navigator.updateRobotPositionFromTriangulation();
            if (!updated) updated = navigator.updateRobotPositionFromAllianceGoals();
            if (!updated) updated = navigator.updateRobotPosition();

            // Config Updates
            updateTargetTile();

            // Auto-Trigger
            if (autoMove && currentTarget != null && !currentTarget.equals(lastTarget)) {
                startMovingToTarget();
                lastTarget = currentTarget;
            }

            // Manual Trigger
            if (gamepad1.a && !isMoving && currentTarget != null) {
                startMovingToTarget();
            }

            // Movement Execution
            if (isMoving && currentTarget != null) {
                updateMovement();
            }

            // Manual Override
            if (Math.abs(gamepad1.left_stick_y) > 0.1 || Math.abs(gamepad1.left_stick_x) > 0.1 || Math.abs(gamepad1.right_stick_x) > 0.1) {
                isMoving = false;
                if (driveSubsystem != null) {
                    driveSubsystem.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
                }
            } else if (!isMoving && driveSubsystem != null) {
                driveSubsystem.stop();
            }

            // --- VISUALIZATION ---
            drawField();
            updateTelemetry();

            sleep(50);
        }

        if (driveSubsystem != null) driveSubsystem.stop();
        navigator.closeVision();
    }

    private void updateTargetTile() {
        if (targetColumn < 'A' || targetColumn > 'F') targetColumn = 'C';
        if (targetRow < 1 || targetRow > 6) targetRow = 3;
        targetOffsetX = Math.max(0, Math.min(24, targetOffsetX));
        targetOffsetY = Math.max(0, Math.min(24, targetOffsetY));
        movePower = Math.max(0, Math.min(1, movePower));
        
        currentTarget = new TileCoordinate(targetColumn, targetRow, targetOffsetX, targetOffsetY);
    }

    private void startMovingToTarget() {
        if (driveSubsystem == null || currentTarget == null) return;
        isMoving = true;
    }

    private void updateMovement() {
        if (driveSubsystem == null || currentTarget == null) {
            isMoving = false;
            return;
        }

        TileCoordinate currentPos = driveSubsystem.getCurrentPosition();
        if (currentPos == null) return;

        double distance = currentPos.distanceTo(currentTarget);
        if (distance <= positionTolerance) {
            isMoving = false;
            driveSubsystem.stop();
            return;
        }

        double angle = currentPos.angleTo(currentTarget);
        double currentHeading = driveSubsystem.getCurrentHeading();
        double turnAngle = angle - currentHeading;
        while (turnAngle > Math.PI) turnAngle -= 2 * Math.PI;
        while (turnAngle < -Math.PI) turnAngle += 2 * Math.PI;
        
        double forward = Math.cos(angle) * movePower;
        double strafe = Math.sin(angle) * movePower;
        double turn = Math.signum(turnAngle) * Math.min(Math.abs(turnAngle) / Math.PI, 1.0) * movePower * 0.5;
        
        driveSubsystem.drive(forward, strafe, turn);
    }

    private void drawField() {
        if (driveSubsystem == null) return;

        TileCoordinate pos = driveSubsystem.getCurrentPosition();
        double heading = driveSubsystem.getCurrentHeading();
        
        if (pos != null) {
            // Conversion: Tile (0 to 144) -> Panels Center (-72 to 72)
            double fx = pos.getX() - 72.0;
            double fy = pos.getY() - 72.0;

            // 1. Draw Robot
            panelsField.moveCursor(fx, fy);
            panelsField.setStyle("none", "#00AA00", 2.0); 
            panelsField.circle(9.0); // 18 inch robot

            // 2. Draw Heading
            double arrowLen = 12.0;
            double headX = fx + Math.cos(heading) * arrowLen;
            double headY = fy + Math.sin(heading) * arrowLen;
            panelsField.moveCursor(fx, fy);
            panelsField.line(headX, headY);

            // 3. Draw Target
            if (currentTarget != null) {
                double tx = currentTarget.getX() - 72.0;
                double ty = currentTarget.getY() - 72.0;

                // Path Line
                panelsField.moveCursor(fx, fy);
                panelsField.setStyle("none", "#FFFF00", 1.0);
                panelsField.line(tx, ty);

                // Target Box
                panelsField.moveCursor(tx, ty);
                panelsField.setStyle("#33FF0000", "#FF0000", 2.0); 
                panelsField.rect(24.0, 24.0);
            }

            panelsField.update();
        }
    }

    private void updateTelemetry() {
        telemetryM.addLine("=== Move To Tile Test (Panels) ===");
        
        if (driveSubsystem != null) {
            TileCoordinate pos = driveSubsystem.getCurrentPosition();
            if (pos != null) {
                telemetryM.addData("Robot", "%s (X:%.1f, Y:%.1f)", 
                    pos.getTilePosition(), pos.getX(), pos.getY());
            }
        }

        if (currentTarget != null) {
            telemetryM.addData("Target", "%s (X:%.1f, Y:%.1f)", 
                currentTarget.getTilePosition(), currentTarget.getX(), currentTarget.getY());
            
            if (driveSubsystem != null && driveSubsystem.getCurrentPosition() != null) {
                double dist = driveSubsystem.getCurrentPosition().distanceTo(currentTarget);
                telemetryM.addData("Distance", "%.1f inches", dist);
            }
        }
        
        telemetryM.addData("State", isMoving ? "MOVING" : "IDLE");
        telemetryM.update();
    }
}