package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.SubSystems.Control.AimController;
import org.firstinspires.ftc.teamcode.SubSystems.Drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Drive.TileCoordinate;
import org.firstinspires.ftc.teamcode.SubSystems.Drive.TileNavigator;
import org.firstinspires.ftc.teamcode.SubSystems.Funnel.FunnelSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Shooter.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Spindexer.OldSpindexerSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Vision.AprilTagNavigator;

@Autonomous(name = "Red Multi-Cycle Auto (D1)", group = "Autonomous")
public class RedMultiCycleAuto extends OpMode {

    /* ================= SUBSYSTEMS ================= */
    DriveSubsystem drive;
    ShooterSubsystem shooter;
    IntakeSubsystem intake;
    OldSpindexerSubsystem spindexer;
    FunnelSubsystem funnel;
    AprilTagNavigator aprilTag;
    AimController aim;
    TileNavigator tileNavigator;

    ElapsedTime stateTimer = new ElapsedTime();

    /* ================= CONSTANTS ================= */
    // Uses centralized value from ShooterSubsystem.TARGET_RPM
    static final double INTAKE_HOLD_POWER = 0.57;
    static final double CREEP_POWER = 0.25;
    static final double CREEP_DISTANCE_PER_BALL = 6.0;

    static final long FUNNEL_EXTEND_TIME_MS = 400;
    static final long FUNNEL_HOLD_TIME_MS = 300;

    /* ================= RETURN TO ZERO CONFIG ================= */
    private double initialHeading = 0.0; // Store original heading at start
    private boolean returnToZeroEnabled = true; // Toggle for Blue/Red
    private double returnToZeroTime = 5.0; // Last N seconds to return to zero
    private static final double AUTONOMOUS_DURATION = 30.0; // FTC autonomous period

    /* ================= RED D1 START ================= */
    // Adjust ONLY if your coordinate system differs
    static final double START_X = 72.0;   // Tile D
    static final double START_Y = 24.0;   // Tile 1

    /* ================= STATES ================= */
    private enum State {
        ALIGN_PRELOAD,
        SPINUP_PRELOAD,
        SHOOT_PRELOAD,

        MOVE_TO_INTAKE,
        INTAKE_CREEP,

        RETURN_TO_SHOOT,
        ALIGN_FINAL,
        SPINUP_FINAL,
        SHOOT_FINAL,
        RETURN_TO_ZERO,

        DONE
    }

    private enum FunnelState { RETRACTED, EXTENDING, EXTENDED, RETRACTING }

    State state = State.ALIGN_PRELOAD;
    FunnelState funnelState = FunnelState.RETRACTED;

    int shotIndex = 0;
    int intakeIndex = 0;

    TileCoordinate intakeTarget;

    /* ================= INIT ================= */
    @Override
    public void init() {
        drive = new DriveSubsystem(hardwareMap, telemetry);
        shooter = new ShooterSubsystem(hardwareMap, telemetry);
        intake = new IntakeSubsystem(hardwareMap, telemetry);
        spindexer = new OldSpindexerSubsystem(hardwareMap, telemetry);
        funnel = new FunnelSubsystem(hardwareMap, telemetry);
        aprilTag = new AprilTagNavigator(drive, hardwareMap, telemetry);

        aim = new AimController(aprilTag, drive, telemetry);
        aim.setAlliance(AimController.AllianceColor.RED);
        aim.setTargetTagId(24);

        tileNavigator = new TileNavigator(drive, telemetry);
        tileNavigator.setPosition(new TileCoordinate(START_X, START_Y));

        drive.resetHeading();
        spindexer.setIntakeMode(false);
        funnel.retract();
    }

    @Override
    public void start() {
        intake.setPower(INTAKE_HOLD_POWER);
        stateTimer.reset();
        initialHeading = drive.getHeadingDegrees(); // Store original heading in DEGREES for return-to-zero
    }

    /* ================= LOOP ================= */
    @Override
    public void loop() {

        spindexer.update();
        shooter.updateVoltageCompensation();
        intake.setPower(INTAKE_HOLD_POWER);

        // === Auto-trigger return-to-zero in last N seconds ===
        double elapsed = stateTimer.seconds();
        double remaining = AUTONOMOUS_DURATION - elapsed;
        
        if (returnToZeroEnabled && remaining <= returnToZeroTime 
                && state != State.DONE && state != State.RETURN_TO_ZERO) {
            state = State.RETURN_TO_ZERO;
        }

        switch (state) {

            /* -------- PRELOAD -------- */
            case ALIGN_PRELOAD:
                AimController.AlignmentResult a = aim.update();
                drive.drive(a.strafe, a.forward, a.turn);
                if (a.aligned) {
                    drive.stop();
                    state = State.SPINUP_PRELOAD;
                }
                break;

            case SPINUP_PRELOAD:
                shooter.setTargetRPM(ShooterSubsystem.TARGET_RPM);
                if (shooter.isAtTargetRPM()) {
                    shotIndex = 0;
                    state = State.SHOOT_PRELOAD;
                }
                break;

            case SHOOT_PRELOAD:
                if (runShooterCycle()) {
                    spindexer.setIntakeMode(true);
                    spindexer.lockCurrentPosition();
                    intakeIndex = 0;

                    // RED: creep UPFIELD (+Y)
                    intakeTarget = new TileCoordinate(
                            tileNavigator.getCurrentPosition().getX(),
                            tileNavigator.getCurrentPosition().getY() + CREEP_DISTANCE_PER_BALL
                    );

                    state = State.MOVE_TO_INTAKE;
                }
                break;

            /* -------- INTAKE -------- */
            case MOVE_TO_INTAKE:
                tileNavigator.moveToPosition(intakeTarget, CREEP_POWER);
                state = State.INTAKE_CREEP;
                break;

            case INTAKE_CREEP:
                boolean loaded = Math.abs(
                        spindexer.shortestError(
                                spindexer.getTargetPosition(),
                                spindexer.getCurrentPosition()
                        )) > 40;

                boolean reached = tileNavigator.isAtTile(intakeTarget, 0.5);

                if (loaded || reached) {
                    drive.stop();
                    spindexer.goToPositionForCurrentMode(intakeIndex++);

                    if (intakeIndex >= 3) {
                        spindexer.setIntakeMode(false);
                        state = State.RETURN_TO_SHOOT;
                    } else {
                        intakeTarget = new TileCoordinate(
                                intakeTarget.getX(),
                                intakeTarget.getY() + CREEP_DISTANCE_PER_BALL
                        );
                        tileNavigator.moveToPosition(intakeTarget, CREEP_POWER);
                    }
                }
                break;

            /* -------- FINAL -------- */
            case RETURN_TO_SHOOT:
                state = State.ALIGN_FINAL;
                break;

            case ALIGN_FINAL:
                AimController.AlignmentResult f = aim.update();
                drive.drive(f.strafe, f.forward, f.turn);
                if (f.aligned) {
                    drive.stop();
                    state = State.SPINUP_FINAL;
                }
                break;

            case SPINUP_FINAL:
                shooter.setTargetRPM(ShooterSubsystem.TARGET_RPM);
                if (shooter.isAtTargetRPM()) {
                    shotIndex = 0;
                    state = State.SHOOT_FINAL;
                }
                break;

            case SHOOT_FINAL:
                if (runShooterCycle()) {
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
                break;
        }

        telemetry.addData("STATE", state);
        telemetry.addData("SHOT", shotIndex);
        telemetry.addData("RPM", shooter.getCurrentRPM());
        telemetry.addData("FUNNEL", funnelState);
        telemetry.addData("Spindexer Safe", funnelState == FunnelState.RETRACTED ? "YES" : "NO");
        telemetry.addData("Start Tile", "D1");
        telemetry.update();
    }

    /* ================= SHOOTER ================= */
    private boolean runShooterCycle() {

        if (shotIndex >= 3) return true;

        // Only move spindexer when funnel is fully retracted
        if (funnelState == FunnelState.RETRACTED) {
            spindexer.goToPositionForCurrentMode(shotIndex);
        }

        if (!spindexer.isAtPosition() || !shooter.isAtTargetRPM())
            return false;

        switch (funnelState) {

            case RETRACTED:
                funnel.extend();
                funnelState = FunnelState.EXTENDING;
                stateTimer.reset();
                break;

            case EXTENDING:
                if (stateTimer.milliseconds() > FUNNEL_EXTEND_TIME_MS) {
                    funnelState = FunnelState.EXTENDED;
                    stateTimer.reset();
                }
                break;

            case EXTENDED:
                if (stateTimer.milliseconds() > FUNNEL_HOLD_TIME_MS) {
                    funnel.retract();
                    funnelState = FunnelState.RETRACTING;
                    stateTimer.reset();
                }
                break;

            case RETRACTING:
                if (stateTimer.milliseconds() > FUNNEL_EXTEND_TIME_MS) {
                    funnelState = FunnelState.RETRACTED;
                    shotIndex++;
                }
                break;
        }
        return false;
    }
}
