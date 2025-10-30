## AprilTag Localization Test

This module provides AprilTag-based localization integrated with the tile coordinate system. A ready-to-run test OpMode streams detections to FTC Dashboard and reports the robot's estimated tile and field position.

### Test OpMode

- Name: `Vision Tile Dashboard Test`
- Path: `org.firstinspires.ftc.teamcode.SubSystems.test.VisionTileDashboardTest`
- Type: TeleOp

### What it does

- Initializes the FTC SDK AprilTag processor (36h11) and starts a live stream to FTC Dashboard.
- Attempts triangulation using the alliance goal tags (IDs 20 and 24); falls back to single-tag localization.
- Updates telemetry with:
  - Best detection info and confidence
  - Robot tile (A–F, 1–6) and tab (V–Z, 1–5)
  - Field X/Y (inches) and heading (degrees)

### How to run

1. Build and deploy to the RC.
2. From the Driver Station, select `Vision Tile Dashboard Test` under the Test group and press INIT.
3. Open FTC Dashboard and connect to the RC (`http://<rc-ip>:8080/dash`).
4. Press START on the Driver Station. The camera stream and localization telemetry will appear on the Dashboard.

### Notes

- The stream is started automatically by `AprilTagNavigator`.
- If `DriveSubsystem` is available, the robot pose is pushed into it so tile outputs update accordingly.
- Triangulation uses a simple average of poses from tags 20 and 24 when both are visible.
