package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import org.firstinspires.ftc.teamcode.SubSystems.Drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Shooter.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Spindexer.OldSpindexerSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Vision.AprilTagNavigator;
import org.firstinspires.ftc.teamcode.SubSystems.Control.AimController;

@Autonomous(name = "Blue Auto – Simple 3 Shot (TeleOp Matched)", group = "Autonomous")
public class SimpleBlueAuto extends OpMode {

    private DriveSubsystem drive;
    private ShooterSubsystem shooter;
    private OldSpindexerSubsystem spindexer;
    private AprilTagNavigator aprilTag;
    private AimController aimController;

    private ElapsedTime stateTimer = new ElapsedTime();

    // === AUTO CONSTANTS ===
    private static final double SHOOTER_RPM = 5225;

    private static final int[] POSITIONS = {0, 50, 100};
    private int shotIndex = 0;

    private enum State {
        ALIGN,
        SPIN_UP,
        FIRE,
        DONE
    }

    private State state = State.ALIGN;

    @Override
    public void init() {
        drive = new DriveSubsystem(hardwareMap, telemetry);
        shooter = new ShooterSubsystem(hardwareMap, telemetry);
        spindexer = new OldSpindexerSubsystem(hardwareMap, telemetry);
        aprilTag = new AprilTagNavigator(drive, hardwareMap, telemetry);

        aprilTag.initializeCameraControls();
        drive.resetHeading();

        // Initialize AimController with TeleOp-matched settings
        aimController = new AimController(aprilTag, drive, telemetry);
        aimController.setDesiredDistance(134);
        aimController.setDesiredAngle(21);
        aimController.setGains(0.03, 0.03, 0.015);
        aimController.setMaxPower(0.40);
        aimController.setDeadbands(0.75, 1.5, 2.0);
        aimController.setTargetTagId(24);
        aimController.setVisionLossTimeout(500);
        aimController.setVisionSmoothing(0.3);

        telemetry.addLine("Auto Initialized (TeleOp Matched)");
    }

    @Override
    public void start() {
        stateTimer.reset();
    }

    @Override
    public void loop() {

        spindexer.update();
        shooter.updateVoltageCompensation();

        switch (state) {

            case ALIGN:
                AimController.AlignmentResult result = aimController.update();
                drive.drive(result.strafe, result.forward, result.turn);
                
                if (result.aligned) {
                    drive.stop();
                    state = State.SPIN_UP;
                    stateTimer.reset();
                }
                break;

            case SPIN_UP:
                shooter.setTargetRPM(SHOOTER_RPM);
                if (shooter.isAtTargetRPM() || stateTimer.seconds() > 2.0) {
                    state = State.FIRE;
                    stateTimer.reset();
                }
                break;

            case FIRE:
                spindexer.goToPosition(POSITIONS[shotIndex]);
                if (spindexer.isAtPosition() || stateTimer.seconds() > 0.6) {
                    shotIndex++;
                    stateTimer.reset();

                    if (shotIndex >= POSITIONS.length) {
                        state = State.DONE;
                    }
                }
                break;

            case DONE:
                drive.stop();
                shooter.stop();
                break;
        }

        if (state == State.ALIGN) {
            AimController.AlignmentResult result = aimController.update();
            telemetry.addData("ALIGN MODE", result.mode);
        }
        
        telemetry.addData("STATE", state);
        telemetry.addData("SHOT", shotIndex + 1);
        telemetry.update();
    }
}
