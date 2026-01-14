package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.SubSystems.Drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Shooter.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Spindexer.OldSpindexerSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Funnel.FunnelSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Vision.AprilTagNavigator;
import org.firstinspires.ftc.teamcode.SubSystems.Control.AimController;

@Autonomous(name = "Blue Auto – 3 Shot Preload (Funnel)", group = "Autonomous")
public class SimpleBlueAuto extends OpMode {

    // --- Subsystems ---
    private DriveSubsystem drive;
    private ShooterSubsystem shooter;
    private OldSpindexerSubsystem spindexer;
    private FunnelSubsystem funnel;
    private AprilTagNavigator aprilTag;
    private AimController aimController;

    private ElapsedTime stateTimer = new ElapsedTime();

    private static final double SHOOTER_RPM = 5225;
    private int shotIndex = 0;

    // --- Funnel state machine ---
    private enum FunnelState { RETRACTED, EXTENDING, EXTENDED, RETRACTING }
    private FunnelState funnelState = FunnelState.RETRACTED;
    private static final long FUNNEL_EXTEND_TIME_MS = 400;
    private static final long FUNNEL_HOLD_TIME_MS = 300;

    // --- Auto states ---
    private enum State { ALIGN, SPIN_UP, FIRE, DONE }
    private State state = State.ALIGN;

    @Override
    public void init() {
        // --- Initialize subsystems ---
        drive = new DriveSubsystem(hardwareMap, telemetry);
        shooter = new ShooterSubsystem(hardwareMap, telemetry);
        spindexer = new OldSpindexerSubsystem(hardwareMap, telemetry);
        funnel = new FunnelSubsystem(hardwareMap, telemetry);
        aprilTag = new AprilTagNavigator(drive, hardwareMap, telemetry);

        aprilTag.initializeCameraControls();
        drive.resetHeading();

        // --- AimController setup ---
        aimController = new AimController(aprilTag, drive, telemetry);
        aimController.setDesiredDistance(134);
        aimController.setDesiredAngle(21);
        aimController.setGains(0.03, 0.03, 0.015);
        aimController.setMaxPower(0.40);
        aimController.setDeadbands(0.75, 1.5, 2.0);
        aimController.setTargetTagId(24);
        aimController.setVisionLossTimeout(500);
        aimController.setVisionSmoothing(0.3);

        telemetry.addLine("Blue Auto – 3 Shot Preload Initialized");
    }

    @Override
    public void start() {
        stateTimer.reset();
        spindexer.setIntakeMode(false);
        funnel.retract();
        funnelState = FunnelState.RETRACTED;
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

                // Wait until shooter is at or near target RPM
                if (shooter.isAtTargetRPM() || Math.abs(shooter.getCurrentRPM() - SHOOTER_RPM) < 100) {
                    state = State.FIRE;
                    stateTimer.reset();
                    shotIndex = 0;
                }
                break;

            case FIRE:
                // --- Handle one shot at a time ---
                if (shotIndex < 3) {

                    // Step 1: Move spindexer to current shot
                    spindexer.goToPositionForCurrentMode(shotIndex);

                    // Step 2: Wait until spindexer is at position
                    if (spindexer.isAtPosition()) {

                        // Step 3: Wait until shooter is ready
                        if (shooter.isAtTargetRPM() || Math.abs(shooter.getCurrentRPM() - SHOOTER_RPM) < 100) {

                            // Step 4: Funnel state machine
                            switch (funnelState) {

                                case RETRACTED:
                                    funnel.extend();
                                    funnelState = FunnelState.EXTENDING;
                                    stateTimer.reset();
                                    break;

                                case EXTENDING:
                                    if (stateTimer.milliseconds() >= FUNNEL_EXTEND_TIME_MS) {
                                        funnelState = FunnelState.EXTENDED;
                                        stateTimer.reset();
                                    }
                                    break;

                                case EXTENDED:
                                    // Hold briefly while firing
                                    if (stateTimer.milliseconds() >= FUNNEL_HOLD_TIME_MS) {
                                        funnel.retract();
                                        funnelState = FunnelState.RETRACTING;
                                        stateTimer.reset();
                                    }
                                    break;

                                case RETRACTING:
                                    if (stateTimer.milliseconds() >= FUNNEL_EXTEND_TIME_MS) {
                                        // Shot complete → next shot
                                        shotIndex++;
                                        funnelState = FunnelState.RETRACTED;
                                        stateTimer.reset();
                                    }
                                    break;
                            }
                        }
                    }

                } else {
                    state = State.DONE; // All shots complete
                }
                break;

            case DONE:
                drive.stop();
                shooter.stop();
                break;
        }

        // --- Telemetry ---
        AimController.AlignmentResult result = aimController.update();
        telemetry.addData("STATE", state);
        telemetry.addData("SHOT", shotIndex);
        telemetry.addData("Spindexer Target", spindexer.getTargetPosition());
        telemetry.addData("Spindexer Current", spindexer.getCurrentPosition());
        telemetry.addData("Aligned?", result.aligned);
        telemetry.addData("Shooter RPM", "%.0f / %.0f", shooter.getCurrentRPM(), shooter.getTargetRPM());
        telemetry.addData("Funnel State", funnelState);
        telemetry.update();
    }
}
