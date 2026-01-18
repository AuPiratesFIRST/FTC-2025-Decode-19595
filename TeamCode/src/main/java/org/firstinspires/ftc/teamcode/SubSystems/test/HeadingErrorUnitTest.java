package org.firstinspires.ftc.teamcode.SubSystems.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.SubSystems.Drive.DriveSubsystem;

@TeleOp(name = "Heading Error Unit Test", group = "Test")
public class HeadingErrorUnitTest extends LinearOpMode {

    private DriveSubsystem drive;

    private final double[] targets = new double[] { 0.0, 90.0, 180.0, -180.0, 270.0, -90.0 };
    private int targetIndex = 0;

    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;
    private boolean lastB = false;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new DriveSubsystem(hardwareMap, telemetry);

        telemetry.addLine("Heading Error Unit Test");
        telemetry.addLine("Use dpad left/right to change target angle.");
        telemetry.addLine("Press B to reset IMU heading to 0°.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            handleInputs();

            double currentDeg = drive.getHeadingDegrees();
            double targetDeg = targets[targetIndex];

            double methodError = drive.getHeadingError(targetDeg);
            double directError = AngleUnit.normalizeDegrees(targetDeg - currentDeg);

            telemetry.addLine("=== HEADING ===");
            telemetry.addData("Current Heading (deg)", String.format("%.2f", currentDeg));
            telemetry.addData("Target Heading (deg)", String.format("%.2f", targetDeg));

            telemetry.addLine("=== ERROR COMPARISON ===");
            telemetry.addData("DriveSubsystem.getHeadingError()", String.format("%.2f", methodError));
            telemetry.addData("AngleUnit.normalizeDegrees(target - current)", String.format("%.2f", directError));

            telemetry.addLine("=== NOTES ===");
            telemetry.addLine("Errors should match and remain within [-180, 180].");
            telemetry.addLine("Try positions near ±180° to verify wrap behavior.");
            telemetry.update();

            sleep(50);
        }
    }

    private void handleInputs() {
        // Target selection with simple debounce
        boolean dpadLeft = gamepad1.dpad_left;
        boolean dpadRight = gamepad1.dpad_right;
        boolean b = gamepad1.b;

        if (dpadRight && !lastDpadRight) {
            targetIndex = (targetIndex + 1) % targets.length;
        }
        if (dpadLeft && !lastDpadLeft) {
            targetIndex = (targetIndex - 1 + targets.length) % targets.length;
        }
        if (b && !lastB) {
            drive.resetHeading();
        }

        lastDpadLeft = dpadLeft;
        lastDpadRight = dpadRight;
        lastB = b;
    }
}
