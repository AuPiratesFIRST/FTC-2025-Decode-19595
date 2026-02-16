package org.firstinspires.ftc.teamcode.SubSystems.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.teamcode.SubSystems.Drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Vision.AprilTagNavigator;

@TeleOp(name = "Tag Chaser: With Deadbands", group = "Vision")
public class TagChaserOp extends LinearOpMode {

    private DriveSubsystem drive;
    private AprilTagNavigator navigator;
    private ElapsedTime runtime = new ElapsedTime();

    // --- GAINS ---
    private final double Kp = 0.079;
    private final double StrafeKp = 0.075;
    private final double StrafeKd = 0.030;
    private final double TurnKp = 0.020;

    // --- DEADBANDS (Adjust these if the robot jitters) ---
    private final double STRAFE_DEADBAND = 0.75;  // Inches: Ignore lateral error less than 0.75"
    private final double FORWARD_DEADBAND = 1.0;  // Inches: Ignore distance error less than 1"
    private final double TURN_DEADBAND = 1.5;     // Degrees: Ignore rotation error less than 1.5 deg

    private final double DESIRED_DISTANCE = 36.0;
    private final double SPEED_LIMIT = 1.0;

    // Logic State for D-Term
    private double lastErrorX = 0;
    private double lastTime = 0;

    @Override
    public void runOpMode() {
        drive = new DriveSubsystem(hardwareMap, telemetry);
        navigator = new AprilTagNavigator(drive, hardwareMap, telemetry);

        while (!isStopRequested() && !opModeIsActive()) {
            navigator.initializeCameraControls();
            telemetry.addData("Status", "Ready");
            telemetry.update();
        }

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            AprilTagDetection tag = navigator.getBestDetection();

            if (tag != null && tag.ftcPose != null) {
                double currentTime = runtime.seconds();
                double deltaTime = currentTime - lastTime;

                // --- 1. FORWARD/BACKWARD (With Deadband) ---
                double errory = (tag.ftcPose.y - DESIRED_DISTANCE);
                double driveForward = 0;
                if (Math.abs(errory) > FORWARD_DEADBAND) {
                    driveForward = errory * Kp;
                }

                // --- 2. STRAFE (PD Control with Deadband) ---
                double errorx = tag.ftcPose.x;
                double driveStrafe = 0;

                if (Math.abs(errorx) > STRAFE_DEADBAND) {
                    double pTermX = errorx * StrafeKp;
                    double dTermX = 0;
                    if (deltaTime > 0) {
                        dTermX = ((errorx - lastErrorX) / deltaTime) * StrafeKd;
                    }
                    driveStrafe = pTermX + dTermX;
                }

                // --- 3. ROTATION (With Deadband) ---
                double errorRot = tag.ftcPose.bearing;
                double driveTurn = 0;
                if (Math.abs(errorRot) > TURN_DEADBAND) {
                    driveTurn = errorRot * TurnKp;
                }

                // --- 4. APPLY ---
                drive.drive(
                        Range.clip(driveForward, -SPEED_LIMIT, SPEED_LIMIT),
                        Range.clip(driveStrafe, -SPEED_LIMIT, SPEED_LIMIT),
                        Range.clip(driveTurn, -SPEED_LIMIT, SPEED_LIMIT)
                );

                // --- 5. UPDATE HISTORY ---
                lastErrorX = errorx;
                lastTime = currentTime;

                telemetry.addData("Dist Error", "%.1f", errory);
                telemetry.addData("Strafe Error", "%.1f", errorx);
                telemetry.addData("Turn Error", "%.1f", errorRot);
                telemetry.update();

            } else {
                drive.stop();
                lastErrorX = 0;
                lastTime = runtime.seconds();
                telemetry.addData("Status", "No Tag Seen");
                telemetry.update();
            }
        }
    }
}