package org.firstinspires.ftc.teamcode.SubSystems.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.SubSystems.Spindexer.SpindexerSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Shooter.ShooterSubsystem;

@TeleOp(name = "Spindexer Test", group = "Test")
public class SpindexerTest extends LinearOpMode {

    private SpindexerSubsystem spindexer;
    private ShooterSubsystem shooter;

    private boolean lastA = false;
    private boolean lastB = false;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addLine("Initializing...");
        telemetry.update();

        // Initialize subsystems
        shooter = new ShooterSubsystem(hardwareMap, telemetry);
        spindexer = new SpindexerSubsystem(hardwareMap, telemetry, shooter);

        telemetry.addLine("READY");
        telemetry.addLine("Controls:");
        telemetry.addLine("  A = Toggle Intake / Outtake Mode");
        telemetry.addLine("  B = Advance to Next Pocket");
        telemetry.addLine("  Start = Exit Test");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            //-------------------------------
            // MODE TOGGLE (A Button)
            //-------------------------------
            if (gamepad1.a && !lastA) {
                spindexer.setIntakeMode(!spindexer.isIntakeMode());  // A toggles mode (Intake / Outtake)
                telemetry.addData("Mode", spindexer.isIntakeMode() ? "Intake" : "Outtake");
            }
            lastA = gamepad1.a;

            //-------------------------------
            // ADVANCE POSITION (B Button)
            //-------------------------------
            if (gamepad1.b && !lastB) {
                spindexer.advance();  // Advance to next pocket
                telemetry.addLine("Advancing Position...");
            }
            lastB = gamepad1.b;

            //-------------------------------
            // UPDATE SPINDEXER LOOP
            //-------------------------------
            spindexer.update();

            //-------------------------------
            // TELEMETRY
            //-------------------------------
            telemetry.addLine("===== SPINDEXER =====");
            telemetry.addData("Mode", spindexer.isIntakeMode() ? "Intake" : "Outtake");
            telemetry.addData("Current Position", spindexer.getMotorPosition());
            telemetry.addData("Target Position", spindexer.getTargetPosition());
            telemetry.addData("Moving", spindexer.isMoving());
            telemetry.addData("Settling", spindexer.isSettling());
            telemetry.addData("Shooter Pending", spindexer.isShooterPending());
            telemetry.addData("Index", spindexer.getIndex());

            telemetry.update();

            // End test if Start button is pressed
            if (gamepad1.start) {
                break;
            }

            sleep(20);  // Small delay to avoid hogging CPU
        }

        telemetry.addLine("Test Ended.");
        telemetry.update();
    }
}
