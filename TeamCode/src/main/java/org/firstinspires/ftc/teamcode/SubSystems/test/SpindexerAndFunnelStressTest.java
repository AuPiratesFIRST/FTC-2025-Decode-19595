package org.firstinspires.ftc.teamcode.SubSystems.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.SubSystems.Spindexer.OldSpindexerSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Funnel.FunnelSubsystem;

@TeleOp(name = "STRESS TEST: Spindexer vs Cams", group = "Test")
public class SpindexerAndFunnelStressTest extends LinearOpMode {

    private OldSpindexerSubsystem spindexer;
    private FunnelSubsystem funnel;

    private boolean activeHoldEnabled = false;
    private boolean lastInputA = false;

    @Override
    public void runOpMode() {
        spindexer = new OldSpindexerSubsystem(hardwareMap, telemetry);
        funnel = new FunnelSubsystem(hardwareMap, telemetry);

        telemetry.addLine("=== COMPETITION STRESS TEST ===");
        telemetry.addLine("A: Toggle ACTIVE LOCKDOWN (On/Off)");
        telemetry.addLine("RT: Hold to FIRE CAMS (Funnel)");
        telemetry.addLine("\nGOAL: Fire cams while Active is ON.");
        telemetry.addLine("Spindexer error should be < 5 ticks.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // 1. Toggle Lockdown Logic
            if (gamepad1.a && !lastInputA) {
                activeHoldEnabled = !activeHoldEnabled;
                if (activeHoldEnabled) {
                    // Lock exactly where it is now
                    spindexer.lockCurrentPosition();
                }
            }
            lastInputA = gamepad1.a;

            // 2. Funnel Control (Hold Right Trigger to push)
            if (gamepad1.right_trigger > 0.5) {
                funnel.extend();
            } else {
                funnel.retract();
            }

            // 3. Spindexer Execution
            if (activeHoldEnabled) {
                spindexer.setPIDEnabled(true);
                spindexer.update();
            } else {
                // PASSIVE: No power, only mechanical brake
                spindexer.setPIDEnabled(false);
                spindexer.setManualPower(0);
            }

            // 4. Telemetry & Stability Analysis
            double error = spindexer.shortestError(spindexer.getTargetPosition(), spindexer.getCurrentPosition());

            telemetry.addData("HOLD MODE", activeHoldEnabled ? "--- ACTIVE LOCKDOWN ---" : "PASSIVE (BRAKE ONLY)");
            telemetry.addData("Funnel State", funnel.isExtended() ? "PUSHING BALL" : "IDLE");
            telemetry.addLine("---------------------------------");
            telemetry.addData("Target Position", spindexer.getTargetPosition());
            telemetry.addData("Current Position", spindexer.getCurrentPosition());
            telemetry.addData("WIGGLE ERROR (Ticks)", "%.1f", error);

            // Visual Warning for Competition
            if (Math.abs(error) > 10) {
                telemetry.addLine(" WARNING: SPINDEXER MOVED TOO MUCH!");
            } else if (funnel.isExtended()) {
                telemetry.addLine(" STABLE UNDER LOAD");
            }

            telemetry.update();
        }
    }
}