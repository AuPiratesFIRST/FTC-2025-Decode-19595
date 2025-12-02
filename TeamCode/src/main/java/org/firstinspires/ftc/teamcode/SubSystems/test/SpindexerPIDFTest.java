package org.firstinspires.ftc.teamcode.SubSystems.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SubSystems.Spindexer.SpindexerSubsystemPIDF;
import org.firstinspires.ftc.teamcode.SubSystems.Shooter.ShooterSubsystem;

@TeleOp(name = "Spindexer PIDF Test", group = "Test")
public class SpindexerPIDFTest extends LinearOpMode {

    private SpindexerSubsystemPIDF spindexer;
    private ShooterSubsystem shooter;

    private boolean lastA = false;
    private boolean lastB = false;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addLine("Initializing...");
        telemetry.update();

        shooter = new ShooterSubsystem(hardwareMap, telemetry);
        spindexer = new SpindexerSubsystemPIDF(hardwareMap, telemetry, shooter);

        telemetry.addLine("READY");
        telemetry.addLine("Controls:");
        telemetry.addLine("  A = Toggle Intake / Outtake Mode");
        telemetry.addLine("  B = Advance to Next Pocket");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            //-------------------------------
            // MODE TOGGLE
            //-------------------------------
            if (gamepad1.a && !lastA) {
                spindexer.setIntakeMode(!gamepad1.left_bumper);   // A toggles mode
            }
            lastA = gamepad1.a;

            //-------------------------------
            // ADVANCE POSITION
            //-------------------------------
            if (gamepad1.b && !lastB) {
                spindexer.advance();
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
            telemetry.addData("Mode", "(A toggles)  " + (gamepad1.left_bumper ? "OUTTAKE" : "INTAKE"));
            telemetry.addData("OpMode Input", "B = advance");

            telemetry.update();

            sleep(20);
        }
    }
}
