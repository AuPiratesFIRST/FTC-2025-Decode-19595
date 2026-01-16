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

@Autonomous(name = "Blue Multi-Cycle Auto (Integrated)", group = "Autonomous")
public class BlueMultiCycleAuto extends OpMode {

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
    static final double SHOOTER_RPM = 5225;
    static final double INTAKE_HOLD_POWER = 0.57;
    static final double CREEP_POWER = 0.25;
    static final double CREEP_DISTANCE_PER_BALL = 6.0; // inches

    // === Funnel timing ===
    static final long FUNNEL_EXTEND_TIME_MS = 400;
    static final long FUNNEL_HOLD_TIME_MS = 300;

    // === Auto State ===
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

        DONE
    }

    private enum FunnelState { RETRACTED, EXTENDING, EXTENDED, RETRACTING }

    State state = State.ALIGN_PRELOAD;
    FunnelState funnelState = FunnelState.RETRACTED;

    int shotIndex = 0;
    int intakeIndex = 0;

    // Target position for TileNavigator during intake
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
        aim.setTargetTagId(24);

        tileNavigator = new TileNavigator(drive, telemetry);
        tileNavigator.setPosition(new TileCoordinate(0, drive.getPoseY())); // initial Y

        drive.resetHeading();
        spindexer.setIntakeMode(false);
        funnel.retract();
    }

    @Override
    public void start() {
        intake.setPower(INTAKE_HOLD_POWER);
        stateTimer.reset();
    }

    @Override
    public void loop() {

        // === Always-on updates ===
        spindexer.update();
        shooter.updateVoltageCompensation();
        intake.setPower(INTAKE_HOLD_POWER);

        switch (state) {

            /* ================= PRELOAD ================= */

            case ALIGN_PRELOAD:
                AimController.AlignmentResult a = aim.update();
                drive.drive(a.strafe, a.forward, a.turn);
                if (a.aligned) {
                    drive.stop();
                    state = State.SPINUP_PRELOAD;
                }
                break;

            case SPINUP_PRELOAD:
                shooter.setTargetRPM(SHOOTER_RPM);
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
                    // Set intake target for TileNavigator
                    intakeTarget = new TileCoordinate(
                            tileNavigator.getCurrentPosition().getX(),
                            tileNavigator.getCurrentPosition().getY() - CREEP_DISTANCE_PER_BALL
                    );
                    state = State.MOVE_TO_INTAKE;
                }
                break;

            /* ================= INTAKE ================= */

            case MOVE_TO_INTAKE:
                // Move towards the next intake position
                tileNavigator.moveToPosition(intakeTarget, CREEP_POWER);
                stateTimer.reset();
                state = State.INTAKE_CREEP;
                break;

            case INTAKE_CREEP:
                // Check if we reached intake target or spindexer loaded
                boolean loaded = Math.abs(
                        spindexer.shortestError(
                                spindexer.getTargetPosition(),
                                spindexer.getCurrentPosition()
                        )) > 40;

                boolean reached = tileNavigator.isAtTile(intakeTarget, 0.5); // 0.5 inch tolerance

                if (loaded || reached) {
                    drive.stop();
                    spindexer.goToPositionForCurrentMode(intakeIndex++);

                    if (intakeIndex >= 3) {
                        spindexer.setIntakeMode(false);
                        state = State.RETURN_TO_SHOOT;
                    } else {
                        // Move next 6" for next ball
                        intakeTarget = new TileCoordinate(
                                intakeTarget.getX(),
                                intakeTarget.getY() - CREEP_DISTANCE_PER_BALL
                        );
                        tileNavigator.moveToPosition(intakeTarget, CREEP_POWER);
                    }
                }
                break;

            /* ================= FINAL SHOTS ================= */

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
                shooter.setTargetRPM(SHOOTER_RPM);
                if (shooter.isAtTargetRPM()) {
                    shotIndex = 0;
                    state = State.SHOOT_FINAL;
                }
                break;

            case SHOOT_FINAL:
                if (runShooterCycle()) {
                    state = State.DONE;
                }
                break;

            case DONE:
                drive.stop();
                shooter.stop();
                break;
        }

        // === Telemetry ===
        telemetry.addData("STATE", state);
        telemetry.addData("SHOT", shotIndex);
        telemetry.addData("FUNNEL", funnelState);
        telemetry.addData("RPM", shooter.getCurrentRPM());
        telemetry.addData("Intake Target", intakeTarget != null ? intakeTarget.toString() : "N/A");
        telemetry.update();
    }

    /* ================= SHOOTER HELPER ================= */

    private boolean runShooterCycle() {

        if (shotIndex >= 3) return true;

        spindexer.goToPositionForCurrentMode(shotIndex);

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
