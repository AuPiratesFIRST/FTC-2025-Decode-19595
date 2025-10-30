package org.firstinspires.ftc.teamcode.SubSystems.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.teamcode.SubSystems.Drive.TileCoordinate;
import org.firstinspires.ftc.teamcode.SubSystems.Drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Vision.AprilTagNavigator;
import com.acmerobotics.dashboard.canvas.*;

@TeleOp(name = "Vision Tile Dashboard Test", group = "Test")
public class VisionTileDashboardTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DriveSubsystem driveSubsystem = null;
        try {
            driveSubsystem = new DriveSubsystem(hardwareMap, telemetry);
        } catch (Throwable t) {
            telemetry.addData("DriveSubsystem", "Not available: %s", t.getMessage());
        }

        AprilTagNavigator navigator = new AprilTagNavigator(driveSubsystem, hardwareMap, telemetry);

        // Initialize drawing canvas (FullPanels)
        FtcDashboard dashboard = FtcDashboard.getInstance();

        if (driveSubsystem != null) {
            // Optional: set a reasonable starting pose (center of A1, facing +X)
            driveSubsystem.setPosition(new TileCoordinate('A', 1, 12, 12));
            driveSubsystem.setHeading(0.0);
        }

        telemetry.addLine("Vision ready. Streaming to FTC Dashboard...");
        telemetry.addLine("Press START to begin localization.");
        telemetry.addLine("Dashboard overlay drawing enabled");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            boolean updated = false;

            // Prefer triangulation from alliance goal tags, then fall back
            if (!updated) updated = navigator.updateRobotPositionFromTriangulation();
            if (!updated) updated = navigator.updateRobotPositionFromAllianceGoals();
            if (!updated) updated = navigator.updateRobotPosition();

            navigator.updateDECODELocalizationTelemetry();

            if (!updated) {
                telemetry.addLine("Localization not updated this cycle.");
            }

            // Draw robot current pose onto FTC Dashboard overlay
            if (driveSubsystem != null) {
                TileCoordinate pos = driveSubsystem.getCurrentPosition();
                double heading = driveSubsystem.getCurrentHeading();
                if (pos != null) {
                    // Convert TileCoordinate (0..144, bottom-left origin) -> Field View frame (origin at center)
                    double x = pos.getX();
                    double y = pos.getY();
                    double fx = x - 72.0;
                    double fy = y - 72.0;
                    TelemetryPacket packet = new TelemetryPacket();
                    var overlay = packet.fieldOverlay();
                    // Optional: custom grid example (page frame)
                    overlay.drawGrid(0, 0, 144, 144, 7, 7);
                    // Robot footprint
                    overlay.setStroke("#00AA00");
                    overlay.strokeCircle(fx, fy, 2.0);
                    // Heading arrow
                    double arrowLen = 8.0;
                    double x2 = fx + Math.cos(heading) * arrowLen;
                    double y2 = fy + Math.sin(heading) * arrowLen;
                    overlay.strokeLine(fx, fy, x2, y2);
                    // Sample rectangle near robot
                    overlay.setStroke("#AA0000");
                    overlay.strokeRect(fx + 6.0, fy + 3.0, 6.0, 3.0);
                    // Sample polyline path
                    overlay.setStroke("#0066CC");
                    double step = 6.0;
                    overlay.strokeLine(fx, fy, fx + step, fy);
                    overlay.strokeLine(fx + step, fy, fx + 2*step, fy + step);
                    overlay.strokeLine(fx + 2*step, fy + step, fx + 3*step, fy + step);
                    // Text label at robot position
                    overlay.setFill("#FFFFFF");
                    overlay.fillText(pos.getTilePosition(), fx + 2.5, fy + 2.5, "10px Arial", 0.0, false);
                    // Example image (page frame coordinates): top-left at (24,24), 48" square
                    overlay.drawImage("/dash/ftc.jpg", 24, 24, 48, 48);
                    dashboard.sendTelemetryPacket(packet);
                }
            }

            telemetry.update();
            sleep(50);
        }

        navigator.closeVision();
    }
}


