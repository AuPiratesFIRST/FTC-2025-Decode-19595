package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.SubSystems.Drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Drive.TileCoordinate;
import org.firstinspires.ftc.teamcode.SubSystems.Drive.TileNavigator;
import org.firstinspires.ftc.teamcode.SubSystems.Intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Spindexer.OldSpindexerSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Shooter.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Funnel.FunnelSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Vision.AprilTagNavigator;
import org.firstinspires.ftc.teamcode.SubSystems.Control.AimController;

@Autonomous(name = "Blue Multi-Cycle Auto (Final)", group = "Autonomous")
public class BlueMultiCycleAutoFinal extends OpMode {

    // === Subsystems ===
    DriveSubsystem drive;
    ShooterSubsystem shooter;
    IntakeSubsystem intake;
    OldSpindexerSubsystem spindexer;
    FunnelSubsystem funnel;
    AprilTagNavigator aprilTag;
    AimController aim;
    TileNavigator tileNavigator;

    ElapsedTime stateTimer = new ElapsedTime();

    // === Constants ===
    static final double INTAKE_HOLD_POWER = 0.57;
    static final double CREEP_POWER = 0.25;
    static final double CREEP_DISTANCE_PER_BALL = 6.0;
    static final long FUNNEL_EXTEND_TIME_MS = 400;
    static final long FUNNEL_HOLD_TIME_MS = 300;
    static final double WALL_BUMP_INCHES = 3.0;
    
    // === Return to Zero Config ===
    private double initialHeading = 0.0; // Store original heading at start
    private boolean returnToZeroEnabled = true; // Toggle for Blue/Red
    private double returnToZeroTime = 5.0; // Last N seconds to return to zero
    private static final double AUTONOMOUS_DURATION = 30.0; // FTC autonomous period

    // === Auto State ===
    private enum State {
        WALL_CLEARANCE,
        ALIGN_PRELOAD,
        SPINUP_PRELOAD,
        SHOOT_PRELOAD,
        ROTATE_BACK_ZERO,
        DRIVE_TO_C2,
        ROTATE_TO_INTAKE,
        INTAKE_CREEP,
        RETURN_PATH,
        FINAL_ALIGN,
        SPINUP_FINAL,
        FINAL_SHOOT,
        RETURN_TO_ZERO,
        DONE
    }

    private enum FunnelState { RETRACTED, EXTENDING, EXTENDED, RETRACTING }

    State state = State.WALL_CLEARANCE;
    FunnelState funnelState = FunnelState.RETRACTED;

    int shotIndex = 0;
    int intakeIndex = 0;
    TileCoordinate intakeTarget;

    @Override
    public void init() {
        drive = new DriveSubsystem(hardwareMap, telemetry);
        shooter = new ShooterSubsystem(hardwareMap, telemetry);
        intake = new IntakeSubsystem(hardwareMap, telemetry);
        spindexer = new OldSpindexerSubsystem(hardwareMap, telemetry);
        funnel = new FunnelSubsystem(hardwareMap, telemetry);
        aprilTag = new AprilTagNavigator(drive, hardwareMap, telemetry);

        aim = new AimController(aprilTag, drive, telemetry);
        // Added my Mr Cameron
        aim.setAlliance(AimController.AllianceColor.BLUE);
        aim.setTargetTagId(24);

        tileNavigator = new TileNavigator(drive, telemetry);
        tileNavigator.setPosition(new TileCoordinate(0, drive.getY()));

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

            /* ===== Phase 0 — Wall Clearance ===== */
            case WALL_CLEARANCE:
                if (drive.moveInches(WALL_BUMP_INCHES, 0.5)) {
                    stateTimer.reset();
                    state = State.ALIGN_PRELOAD;
                }
                break;

            /* ===== Phase 1 — Align Preloads ===== */
            case ALIGN_PRELOAD:
                AimController.AlignmentResult a = aim.update();
                drive.drive(a.forward, a.strafe, a.turn);
                if (a.aligned) {
                    drive.stop();
                    state = State.SPINUP_PRELOAD;
                }
                break;

            /* ===== Spinup Preload ===== */
            case SPINUP_PRELOAD:
                shooter.setTargetRPM(ShooterSubsystem.TARGET_RPM);
                if (shooter.isAtTargetRPM()) {
                    shotIndex = 0;
                    stateTimer.reset();
                    state = State.SHOOT_PRELOAD;
                }
                break;

            /* ===== Phase 2 — Shoot Preloads ===== */
            case SHOOT_PRELOAD:
                if (runShooterCycle()) {
                    state = State.ROTATE_BACK_ZERO;
                }
                break;

            /* ===== Rotate Back to Zero ===== */
            case ROTATE_BACK_ZERO:
                double error0 = drive.getHeadingError(0);
                if (Math.abs(error0) > 1.0) {
                    drive.drive(0, 0, error0 > 0 ? 0.3 : -0.3);
                } else {
                    drive.stop();
                    state = State.DRIVE_TO_C2;
                }
                break;

            /* ===== Drive to C2 ===== */
            case DRIVE_TO_C2:
                if (drive.moveInches(24, 0.5)) {
                    state = State.ROTATE_TO_INTAKE;
                }
                break;

            /* ===== Rotate to Intake ===== */
            case ROTATE_TO_INTAKE:
                double error90 = drive.getHeadingError(-90);
                if (Math.abs(error90) > 1.0) {
                    drive.drive(0, 0, error90 > 0 ? 0.3 : -0.3);
                } else {
                    drive.stop();
                    intakeIndex = 0;
                    spindexer.setIntakeMode(true);
                    intakeTarget = new TileCoordinate(
                            tileNavigator.getCurrentPosition().getX(),
                            tileNavigator.getCurrentPosition().getY() - CREEP_DISTANCE_PER_BALL
                    );
                    state = State.INTAKE_CREEP;
                }
                break;

            /* ===== Intake Creep ===== */
            case INTAKE_CREEP:
                tileNavigator.moveToPosition(intakeTarget, CREEP_POWER);

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
                        state = State.RETURN_PATH;
                    } else {
                        intakeTarget = new TileCoordinate(
                                intakeTarget.getX(),
                                intakeTarget.getY() - CREEP_DISTANCE_PER_BALL
                        );
                    }
                }
                break;

            /* ===== Return Path ===== */
            case RETURN_PATH:
                TileCoordinate origin = new TileCoordinate(0, drive.getY());
                if (!tileNavigator.isAtTile(origin, 0.5)) {
                    tileNavigator.moveToPosition(origin, 0.5);
                } else {
                    state = State.FINAL_ALIGN;
                }
                break;

            /* ===== Final Align ===== */
            case FINAL_ALIGN:
                AimController.AlignmentResult f = aim.update();
                drive.drive(f.forward, f.strafe, f.turn);
                if (f.aligned) {
                    drive.stop();
                    state = State.SPINUP_FINAL;
                }
                break;

            /* ===== Spinup Final ===== */
            case SPINUP_FINAL:
                shooter.setTargetRPM(ShooterSubsystem.TARGET_RPM);
                if (shooter.isAtTargetRPM()) {
                    shotIndex = 0;
                    stateTimer.reset();
                    state = State.FINAL_SHOOT;
                }
                break;

            /* ===== Final Shoot ===== */
            case FINAL_SHOOT:
                if (runShooterCycle()) {
                    if (returnToZeroEnabled && remaining > returnToZeroTime) {
                        state = State.RETURN_TO_ZERO;
                    } else {
                        state = State.DONE;
                    }
                }
                break;

            /* ===== Return to Zero Heading ===== */
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

            /* ===== Done ===== */
            case DONE:
                drive.stop();
                shooter.stop();
                intake.setPower(0);
                break;
        }

        telemetry.addData("STATE", state);
        telemetry.addData("SHOT", shotIndex);
        telemetry.addData("FUNNEL", funnelState);
        telemetry.addData("Spindexer Safe", funnelState == FunnelState.RETRACTED ? "YES" : "NO");
        telemetry.addData("RPM", shooter.getCurrentRPM());
        telemetry.update();
    }

    /* ================= SHOOTER HELPER ================= */
    private boolean runShooterCycle() {

        if (shotIndex >= 3) return true;

        // Only move spindexer when funnel is fully retracted
        if (funnelState == FunnelState.RETRACTED) {
            spindexer.goToPositionForCurrentMode(shotIndex);
        }

        if (!spindexer.isAtPosition() || !shooter.isAtTargetRPM()) return false;

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