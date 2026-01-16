package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.SubSystems.Drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Shooter.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Spindexer.OldSpindexerSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Funnel.FunnelSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Vision.AprilTagNavigator;
import org.firstinspires.ftc.teamcode.SubSystems.Control.AimController;

@Autonomous(name = "Blue Auto – 3 Shot Preload (Funnel + Intake Hold)", group = "Autonomous")
public class SimpleBlueAuto extends OpMode {

    // --- Subsystems ---
    private DriveSubsystem drive;
    private ShooterSubsystem shooter;
    private IntakeSubsystem intake;
    private OldSpindexerSubsystem spindexer;
    private FunnelSubsystem funnel;
    private AprilTagNavigator aprilTag;
    private AimController aimController;

    private ElapsedTime stateTimer = new ElapsedTime();

    // Uses centralized value from ShooterSubsystem.TARGET_RPM
    private int shotIndex = 0;

    // === INTAKE CONFIG (AUTO HOLD) ===
    private static final double INTAKE_HOLD_POWER = 0.57;
    
    // === Return to Zero Config ===
    private double initialHeading = 0.0; // Store original heading at start
    private boolean returnToZeroEnabled = true; // Toggle for Blue/Red
    private double returnToZeroTime = 5.0; // Last N seconds to return to zero
    private static final double AUTONOMOUS_DURATION = 30.0; // FTC autonomous period

    // --- Funnel state machine ---
    private enum FunnelState { RETRACTED, EXTENDING, EXTENDED, RETRACTING, WAITING_FOR_SPINDEXER }
    private FunnelState funnelState = FunnelState.RETRACTED;
    private static final long FUNNEL_EXTEND_TIME_MS = 300; // Funnel extends quickly (~0.3s)
    private static final long FUNNEL_RETRACT_TIME_MS = 600; // Longer retract to fully clear
    private static final long FUNNEL_HOLD_TIME_MS = 300;
    private static final long FUNNEL_CLEAR_BUFFER_MS = 150; // Extra buffer before spindexer moves

    // --- Auto states ---
    private enum State { ALIGN, SPIN_UP, FIRE, RETURN_TO_ZERO, DONE }
    private State state = State.ALIGN;

    @Override
    public void init() {
        drive = new DriveSubsystem(hardwareMap, telemetry);
        shooter = new ShooterSubsystem(hardwareMap, telemetry);
        intake = new IntakeSubsystem(hardwareMap, telemetry);
        spindexer = new OldSpindexerSubsystem(hardwareMap, telemetry);
        funnel = new FunnelSubsystem(hardwareMap, telemetry);
        aprilTag = new AprilTagNavigator(drive, hardwareMap, telemetry);

        aprilTag.initializeCameraControls();
        drive.resetHeading();

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
        initialHeading = Math.toDegrees(drive.getHeading()); // Store original heading in DEGREES for return-to-zero

        // ✅ Start intake hold immediately
        intake.setPower(INTAKE_HOLD_POWER);
    }

    @Override
    public void loop() {

        // ✅ ALWAYS hold balls in spindexer
        intake.setPower(INTAKE_HOLD_POWER);

        spindexer.update();
        shooter.updateVoltageCompensation();

        // === Auto-trigger return-to-zero in last N seconds ===
        double elapsed = stateTimer.seconds();
        double remaining = AUTONOMOUS_DURATION - elapsed;
        
        if (returnToZeroEnabled && remaining <= returnToZeroTime 
                && state != State.DONE && state != State.RETURN_TO_ZERO) {
            state = State.RETURN_TO_ZERO;
        }

        switch (state) {

            case ALIGN:
                AimController.AlignmentResult align = aimController.update();
                drive.drive(align.strafe, align.forward, align.turn);

                if (align.aligned) {
                    drive.stop();
                    state = State.SPIN_UP;
                    stateTimer.reset();
                }
                break;

            case SPIN_UP:
                shooter.setTargetRPM(ShooterSubsystem.TARGET_RPM);

                if (shooter.isAtTargetRPM()
                        || Math.abs(shooter.getCurrentRPM() - ShooterSubsystem.TARGET_RPM) < 100) {
                    state = State.FIRE;
                    shotIndex = 0;
                    stateTimer.reset();
                }
                break;

            case FIRE:
                if (shotIndex < 3) {

                    // Step 1: Move spindexer
                    spindexer.goToPositionForCurrentMode(shotIndex);

                    if (spindexer.isAtPosition()
                            && (shooter.isAtTargetRPM()
                            || Math.abs(shooter.getCurrentRPM() - ShooterSubsystem.TARGET_RPM) < 100)) {

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
                                if (stateTimer.milliseconds() >= FUNNEL_HOLD_TIME_MS) {
                                    funnel.retract();
                                    funnelState = FunnelState.RETRACTING;
                                    stateTimer.reset();
                                }
                                break;

                            case RETRACTING:
                                if (stateTimer.milliseconds() >= FUNNEL_RETRACT_TIME_MS) {
                                    funnelState = FunnelState.WAITING_FOR_SPINDEXER;
                                    stateTimer.reset();
                                }
                                break;

                            case WAITING_FOR_SPINDEXER:
                                if (stateTimer.milliseconds() >= FUNNEL_CLEAR_BUFFER_MS) {
                                    shotIndex++;
                                    funnelState = FunnelState.RETRACTED;
                                    stateTimer.reset();
                                }
                                break;
                        }
                    }

                } else {
                    if (returnToZeroEnabled && remaining > returnToZeroTime) {
                        state = State.RETURN_TO_ZERO;
                    } else {
                        state = State.DONE;
                    }
                }
                break;

            case RETURN_TO_ZERO:
                double headingError = drive.getHeadingError(initialHeading); // initialHeading is already in degrees
                
                // Simple proportional turn to original heading
                if (Math.abs(headingError) > 2.0) { // 2° tolerance
                    double turnPower = Math.min(Math.abs(headingError) * 0.008, 0.3); // Smoother P factor
                    drive.drive(0, 0, headingError > 0 ? turnPower : -turnPower);
                } else {
                    drive.stop();
                    state = State.DONE;
                }
                break;

            case DONE:
                drive.stop();
                shooter.stop();
                intake.setPower(INTAKE_HOLD_POWER); // still hold balls
                break;
        }

        telemetry.addData("STATE", state);
        telemetry.addData("SHOT", shotIndex);
        telemetry.addData("INTAKE", "HOLDING");
        telemetry.addData("SHOOTER RPM", "%.0f / %.0f",
                shooter.getCurrentRPM(), shooter.getTargetRPM());
        telemetry.addData("FUNNEL", funnelState);
        telemetry.update();
    }
}
