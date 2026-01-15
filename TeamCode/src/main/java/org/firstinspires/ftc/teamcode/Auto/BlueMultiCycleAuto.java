package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.SubSystems.Drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Spindexer.OldSpindexerSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Vision.AprilTagNavigator;
import org.firstinspires.ftc.teamcode.SubSystems.Control.AimController;

@Autonomous(name = "Blue Multi-Cycle Auto (Integrated)", group = "Autonomous")
public class BlueMultiCycleAuto extends OpMode {

    // === Subsystems ===
    DriveSubsystem drive;
    IntakeSubsystem intake;
    OldSpindexerSubsystem spindexer;
    AprilTagNavigator aprilTag;
    AimController aim;

    ElapsedTime stateTimer = new ElapsedTime();

    // === Constants ===
    static final double HEADING_GOAL = Math.toRadians(70);
    static final double HEADING_INTAKE = Math.toRadians(180);

    static final double INTAKE_HOLD_POWER = 0.57;

    static final double CREEP_POWER = 0.25;
    static final double CREEP_DISTANCE_PER_BALL = 6.0; // inches

    // === Auto State ===
    private enum State {
        ALIGN_PRELOAD,
        SHOOT_PRELOAD,
        MOVE_TO_INTAKE,
        INTAKE_CREEP,
        RETURN_TO_SHOOT,
        ALIGN_FINAL,
        SHOOT_FINAL,
        DONE
    }

    State state = State.ALIGN_PRELOAD;

    int intakeIndex = 0;
    double creepStartY;
    double targetCreepY;

    @Override
    public void init() {
        drive = new DriveSubsystem(hardwareMap, telemetry);
        intake = new IntakeSubsystem(hardwareMap, telemetry);
        spindexer = new OldSpindexerSubsystem(hardwareMap, telemetry);
        aprilTag = new AprilTagNavigator(drive, hardwareMap, telemetry);

        aim = new AimController(aprilTag, drive, telemetry);
        aim.setTargetTagId(24);

        drive.resetHeading();
        spindexer.setIntakeMode(false); // preload = outtake geometry
    }

    @Override
    public void start() {
        stateTimer.reset();
        intake.setPower(INTAKE_HOLD_POWER);
    }

    @Override
    public void loop() {

        // === Always-on updates ===
        spindexer.update();
        intake.setPower(INTAKE_HOLD_POWER);

        switch (state) {

            /* =============================
               PRELOAD ALIGN & SHOOT
               ============================= */

            case ALIGN_PRELOAD:
                AimController.AlignmentResult a = aim.update();
                drive.drive(a.strafe, a.forward, a.turn);

                if (a.aligned) {
                    drive.stop();
                    state = State.SHOOT_PRELOAD;
                    stateTimer.reset();
                }
                break;

            case SHOOT_PRELOAD:
                // Assume your existing shooter logic runs here
                if (stateTimer.milliseconds() > 2000) {
                    spindexer.setIntakeMode(true);   // switch to intake
                    spindexer.lockCurrentPosition();
                    intakeIndex = 0;
                    state = State.MOVE_TO_INTAKE;
                    stateTimer.reset();
                }
                break;

            /* =============================
               MOVE TO INTAKE ZONE
               ============================= */

            case MOVE_TO_INTAKE:
                // Stop drive & record starting Y for odometry-backed creep
                drive.stop();
                creepStartY = drive.getPoseY(); // field-relative inches
                targetCreepY = creepStartY - CREEP_DISTANCE_PER_BALL; // negative = forward
                state = State.INTAKE_CREEP;
                stateTimer.reset();
                break;

            /* =============================
               INTAKE WITH HEADING HOLD
               ============================= */

            case INTAKE_CREEP:

                // --- Heading hold ---
                double headingError = HEADING_INTAKE - drive.getHeading();
                while (headingError > Math.PI) headingError -= 2 * Math.PI;
                while (headingError < -Math.PI) headingError += 2 * Math.PI;
                double turn = Range.clip(headingError * 0.8, -0.2, 0.2);

                // --- Drive forward (negative Y) while holding heading ---
                drive.drive(0, -CREEP_POWER, turn);

                // --- Check odometry termination ---
                double currentY = drive.getPoseY();
                boolean reachedDistance = Math.abs(currentY - creepStartY) >= CREEP_DISTANCE_PER_BALL;

                // --- OR spindexer loaded (ball detected) ---
                boolean spindexerLoaded = Math.abs(
                        spindexer.shortestError(
                                spindexer.getTargetPosition(),
                                spindexer.getCurrentPosition()
                        )) > 40;

                if (reachedDistance || spindexerLoaded) {
                    drive.stop(); // stop drift

                    spindexer.goToPositionForCurrentMode(intakeIndex);
                    intakeIndex++;

                    if (intakeIndex < 3) {
                        // Prepare next creep
                        creepStartY = drive.getPoseY();
                        targetCreepY = creepStartY - CREEP_DISTANCE_PER_BALL;
                        stateTimer.reset();
                    } else {
                        // All balls collected → return to shoot
                        spindexer.setIntakeMode(false);
                        state = State.RETURN_TO_SHOOT;
                        stateTimer.reset();
                    }
                }
                break;

            /* =============================
               RETURN & FINAL SHOT
               ============================= */

            case RETURN_TO_SHOOT:
                drive.stop();
                state = State.ALIGN_FINAL;
                break;

            case ALIGN_FINAL:
                AimController.AlignmentResult f = aim.update();
                drive.drive(f.strafe, f.forward, f.turn);

                if (f.aligned) {
                    drive.stop();
                    state = State.SHOOT_FINAL;
                    stateTimer.reset();
                }
                break;

            case SHOOT_FINAL:
                if (stateTimer.milliseconds() > 2000) {
                    state = State.DONE;
                }
                break;

            case DONE:
                drive.stop();
                intake.setPower(INTAKE_HOLD_POWER);
                break;
        }

        telemetry.addData("STATE", state);
        telemetry.addData("INTAKE INDEX", intakeIndex);
        telemetry.addData("POSE Y", drive.getPoseY());
        telemetry.update();
    }
}
