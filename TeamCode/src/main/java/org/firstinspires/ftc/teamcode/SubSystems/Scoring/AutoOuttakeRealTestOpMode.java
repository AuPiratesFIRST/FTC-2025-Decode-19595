//package org.firstinspires.ftc.teamcode.SubSystems.Scoring;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.teamcode.SubSystems.Sensors.ColorSensorSubsystem;
//import org.firstinspires.ftc.teamcode.SubSystems.Spindexer.OldSpindexerSubsystem;
//import org.firstinspires.ftc.teamcode.SubSystems.Shooter.ShooterSubsystem;
//import org.firstinspires.ftc.teamcode.SubSystems.Funnel.FunnelSubsystem;
//
//@TeleOp(name = "REAL TEST - Auto Outtake", group = "TEST")
//public class AutoOuttakeRealTestOpMode extends LinearOpMode {
//
//    private ColorSensorSubsystem colorSensor;
//    private OldSpindexerSubsystem spindexer;
//    private ShooterSubsystem shooter;
//    private FunnelSubsystem funnel;
//    private AutoOuttakeController autoOuttake;
//
//    // ⚠️ Use your REAL motif here
//    private static final ArtifactColor[] TARGET_MOTIF = {
//            ArtifactColor.GREEN,
//            ArtifactColor.PURPLE,
//            ArtifactColor.GREEN
//    };
//
//    @Override
//    public void runOpMode() {
//
//        // --- Initialize Hardware ---
//        colorSensor = new ColorSensorSubsystem(hardwareMap, telemetry);
//        spindexer = new OldSpindexerSubsystem(hardwareMap, telemetry);
//        shooter = new ShooterSubsystem(hardwareMap, telemetry);
//        funnel = new FunnelSubsystem(hardwareMap, telemetry);
//
//        autoOuttake = new AutoOuttakeController(
//                colorSensor,
//                TARGET_MOTIF,
//                spindexer,
//                shooter,
//                funnel,
//                telemetry
//        );
//
//        // Tune here if needed
//        autoOuttake.setScoreThreshold(6);      // REAL detection threshold
//        autoOuttake.setTargetShooterRPM(5220); // Match competition value
//
//        telemetry.addLine("REAL Auto Outtake Test Ready");
//        telemetry.addLine("Load rings → wait for auto fire");
//        telemetry.addLine("B = EMERGENCY STOP");
//        telemetry.update();
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//
//            // --- Emergency Stop ---
//            if (gamepad1.b) {
//                shooter.stop();
//                funnel.retract();
//                spindexer.setIntakeMode(true);
//                telemetry.addLine("!!! EMERGENCY STOP !!!");
//                telemetry.update();
//                continue;
//            }
//
//            // --- Run Auto Outtake ---
//            autoOuttake.update();
//
//            // --- LIVE DEBUG TELEMETRY ---
//            telemetry.addData("Auto State", autoOuttake.getState());
//            telemetry.addData("Pattern Score", colorSensor.getPatternScore());
//            telemetry.addData("Detected Count", colorSensor.getDetectedRamp().length);
//
//            telemetry.addData("Shooter RPM",
//                    "%.0f / %.0f",
//                    shooter.getCurrentRPM(),
//                    shooter.getTargetRPM());
//
//            telemetry.addData("Spindexer Index", spindexer.getIndex());
//            telemetry.update();
//        }
//    }
//}
