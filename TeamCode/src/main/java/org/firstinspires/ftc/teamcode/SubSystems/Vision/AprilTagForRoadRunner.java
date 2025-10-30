package org.firstinspires.ftc.teamcode.SubSystems.Vision;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;


public class AprilTagForRoadRunner {
    private final HardwareMap hardwareMap;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    public AprilTagForRoadRunner(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public void init() {
        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Create the vision portal by using a builder.
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
    }

    public int detectPosition() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                // Assuming tag IDs 1, 2, and 3 correspond to positions 1, 2, and 3
                if (detection.id == 1) {
                    return 1;
                } else if (detection.id == 2) {
                    return 2;
                } else if (detection.id == 3) {
                    return 3;
                }
            }
        }
        return 0; // No tag detected
    }
}