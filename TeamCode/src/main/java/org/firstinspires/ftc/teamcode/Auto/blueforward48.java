package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.SubSystems.Drive.DriveSubsystem;

@Autonomous(name = "blueDrive Forward 48 Inches", group = "Testing")
public class blueforward48 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        DriveSubsystem drive = new DriveSubsystem(hardwareMap, telemetry);

        telemetry.addLine("Initialized. Ready to run.");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        telemetry.addLine("Driving forward 78 inches...");
        telemetry.update();

        // *** Power usually around 0.5–0.7 for encoder accuracy ***
        drive.moveInches(44, 0.6);


        telemetry.addLine("Done.");
        telemetry.update();

        sleep(500); // brief pause
    }
}
