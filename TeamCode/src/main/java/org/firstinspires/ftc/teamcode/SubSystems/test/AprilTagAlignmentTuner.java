package org.firstinspires.ftc.teamcode.SubSystems.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

// Panels Configuration Imports
import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.Sorter;

// Panels Telemetry
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.teamcode.SubSystems.Drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Vision.AprilTagNavigator;

@Configurable
@TeleOp(name = "AprilTag Tuner (Graph + Panels)", group = "Test")
public class AprilTagAlignmentTuner extends LinearOpMode {

    // === TUNING VARIABLES (Editable Live in Panels) ===
    
    // PID Gains
    @Sorter(sort = 0) public static double kP_Strafe = 0.03;
    @Sorter(sort = 1) public static double kP_Forward = 0.03;
    @Sorter(sort = 2) public static double kP_Rotation = 0.015;
    @Sorter(sort = 3) public static double MAX_POWER = 0.40;

    // Targets
    @Sorter(sort = 10) public static double Target_Distance = 134.0; // inches
    @Sorter(sort = 11) public static double Target_Angle = 21.0;     // degrees

    // Deadbands (Tolerance)
    @Sorter(sort = 20) public static double Deadband_Position = 0.75; // inches
    @Sorter(sort = 21) public static double Deadband_Angle = 1.5;     // degrees

    // === SUBSYSTEMS ===
    private DriveSubsystem drive;
    private AprilTagNavigator aprilTag;
    
    // Using JoinedTelemetry to send data to both Driver Station and Panels Graph
    private JoinedTelemetry telemetryM; 

    // === STATE ===
    private boolean isAligned = false;

    @Override
    public void runOpMode() {
        // 1. Initialize Joined Telemetry
        telemetryM = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);

        // 2. Initialize Hardware
        drive = new DriveSubsystem(hardwareMap, telemetry);
        aprilTag = new AprilTagNavigator(drive, hardwareMap, telemetry);

        // 3. Register this class with Panels
        PanelsConfigurables.INSTANCE.refreshClass(this);

        telemetryM.addLine("AprilTag Tuner Ready");
        telemetryM.addLine("1. Add 'Graph' widget in Panels");
        telemetryM.addLine("2. Select 'Error_Dist' or 'Error_Ang'");
        telemetryM.addLine("3. Hold Y to align");
        telemetryM.addLine("");
        telemetryM.addLine("⚠️ Start with low MAX_POWER!");
        telemetryM.addLine("Note: No IMU fallback in this tuner");
        telemetryM.update();

        waitForStart();

        // Init camera settings
        aprilTag.initializeCameraControls();

        while (opModeIsActive()) {
            // A. Update Vision
            aprilTag.updateRobotPositionFromAllianceGoals();
            AprilTagDetection targetTag = aprilTag.getBestAllianceGoalDetection();

            // B. Controls & Logic
            if (gamepad1.y) {
                if (targetTag != null) {
                    runAlignment(targetTag);
                } else {
                    drive.stop();
                }
            } else {
                drive.stop();
                isAligned = false;
            }

            // C. Telemetry (Graphing happens here)
            updateTelemetry(targetTag);
            
            // Small delay to prevent network flooding (Graph works best around 20-50ms updates)
            sleep(20);
        }

        drive.stop();
        aprilTag.closeVision();
    }

    // Store last corrections for telemetry
    private double lastCorrectionForward = 0;
    private double lastCorrectionStrafe = 0;
    private double lastCorrectionTurn = 0;

    private void runAlignment(AprilTagDetection tag) {
        double[] corrections = aprilTag.calculateAlignmentCorrections(
                tag,
                Target_Distance,
                Target_Angle,
                Deadband_Position,
                Deadband_Position,
                Deadband_Angle,
                kP_Strafe,
                kP_Forward,
                kP_Rotation,
                MAX_POWER
        );

        if (corrections != null) {
            lastCorrectionForward = corrections[0];
            lastCorrectionStrafe = corrections[1];
            lastCorrectionTurn = corrections[2];
            
            drive.drive(lastCorrectionStrafe, lastCorrectionForward, lastCorrectionTurn);
            isAligned = (corrections[3] == 1.0);
        } else {
            drive.stop();
            lastCorrectionForward = 0;
            lastCorrectionStrafe = 0;
            lastCorrectionTurn = 0;
        }
    }

    private void updateTelemetry(AprilTagDetection tag) {
        // Status for Text Telemetry
        telemetryM.addData("Status", gamepad1.y ? "ALIGNING" : "STANDBY");
        telemetryM.addData("Aligned", isAligned);

        if (tag != null) {
            // Calculate Errors (positive = need to move in positive direction)
            double errDist = tag.ftcPose.y - Target_Distance;
            double errAng = Target_Angle - tag.ftcPose.yaw;  // Fixed: Target - Current for correct sign
            double errStrafe = tag.ftcPose.x;  // Strafe error from center 

            // === DATA FOR GRAPHING ===
            // Panels Graph looks for <Name>:<Value>
            // JoinedTelemetry handles formatting automatically for numbers
            
            // Errors (input to PID)
            telemetryM.addData("Error_Dist", errDist);   // Graph this to tune kP_Forward
            telemetryM.addData("Error_Ang", errAng);     // Graph this to tune kP_Rotation
            telemetryM.addData("Error_Strafe", errStrafe); // Graph this to tune kP_Strafe
            
            // Also useful to graph the raw position vs target
            telemetryM.addData("Raw_Dist", tag.ftcPose.y);
            telemetryM.addData("Target_Dist", Target_Distance);
            
            // Correction Outputs (PID output - actual motor commands)
            // Graph these alongside errors to see response time & overshoot
            telemetryM.addData("Correction_Forward", lastCorrectionForward);
            telemetryM.addData("Correction_Strafe", lastCorrectionStrafe);
            telemetryM.addData("Correction_Turn", lastCorrectionTurn);
        } else {
            // Send 0 or previous value when tag lost to keep graph scrolling
            telemetryM.addData("Error_Dist", 0);
            telemetryM.addData("Error_Ang", 0);
        }

        telemetryM.update();
    }
}