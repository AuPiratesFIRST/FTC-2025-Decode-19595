package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import org.firstinspires.ftc.teamcode.SubSystems.Drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Shooter.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Spindexer.OldSpindexerSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Vision.AprilTagNavigator;

@Autonomous(name = "Blue Auto – Simple 3 Shot", group = "Autonomous")
public class SimpleBlueAuto extends OpMode {

    private DriveSubsystem drive;
    private ShooterSubsystem shooter;
    private OldSpindexerSubsystem spindexer;
    private AprilTagNavigator aprilTag;
    private ElapsedTime timer;

    // Shooter preset
    private static final double SHOOTER_POWER = 0.87;

    private static final int[] POSITIONS = {0, 50, 100}; //soulde be encoder ticks for 3 shots
    private int currentShotIndex = 0;

    private enum State {
        ALIGN_TO_TAG,
        SPIN_UP,
        FIRE,
        NEXT_SHOT,
        DRIVE_FORWARD,
        DONE
    }

    private State state = State.ALIGN_TO_TAG;

    @Override
    public void init() {
        drive = new DriveSubsystem(hardwareMap, telemetry);
        shooter = new ShooterSubsystem(hardwareMap, telemetry);
        spindexer = new OldSpindexerSubsystem(hardwareMap, telemetry);

        /**
         * YOUR TeleOp uses this constructor:
         *    new AprilTagNavigator(drive, hardwareMap, telemetry)
         * so Auto MUST use the same one.
         */
        aprilTag = new AprilTagNavigator(drive, hardwareMap, telemetry);

        timer = new ElapsedTime();
        telemetry.addLine("Initialized Simple Auto");
    }

    @Override
    public void start() {
        timer.reset();
        telemetry.addLine("Auto Started");
    }

    @Override
    public void loop() {
        spindexer.update();

        switch (state) {

            case ALIGN_TO_TAG:
                if (alignToAprilTag()) {
                    state = State.SPIN_UP;
                    timer.reset();
                }
                break;

            case SPIN_UP:
                shooter.setPower(SHOOTER_POWER);
                if (shooter.isAtTargetRPM() || timer.seconds() > 2.0) {
                    state = State.FIRE;
                    timer.reset();
                }
                break;

            case FIRE:
                spindexer.goToPosition(POSITIONS[currentShotIndex]);

                if (spindexer.isAtPosition() || timer.seconds() > 0.6) {
                    timer.reset();
                    state = State.NEXT_SHOT;
                }
                break;

            case NEXT_SHOT:
                if (currentShotIndex < 2) {
                    currentShotIndex++;
                    state = State.FIRE;
                    timer.reset();
                } else {
                    state = State.DRIVE_FORWARD;
                }
                break;

            case DRIVE_FORWARD:
                drive.moveInches(300, 0.8);
                if (drive.isAtTarget()) {
                    state = State.DONE;
                }
                break;

            case DONE:
                shooter.stop();
                drive.stop();
                break;
        }

        updateTelemetry();
    }

    // ------------------------------------------------------------
    // FIXED ALIGNMENT — now matches your TeleOp logic exactly
    // ------------------------------------------------------------
    private boolean alignToAprilTag() {

        aprilTag.updateRobotPositionFromAllianceGoals();
        AprilTagDetection blueGoal = aprilTag.getBestAllianceGoalDetection();

        if (blueGoal == null) {
            telemetry.addData("ALIGN", "No Tag Seen");
            drive.stop();
            return false;
        }

        double xOffset = blueGoal.ftcPose.x;   // Left/Right tag offset

        // === Proportional turn alignment (same as TeleOp) ===
        double kP = 0.02;              // Tune between 0.015–0.025
        double turnPower = xOffset * kP;

        if (Math.abs(xOffset) < 0.5) turnPower = 0; // Deadband

        // Clamp turn power
        turnPower = Math.max(-0.4, Math.min(0.4, turnPower));

        // Alignment tolerance
        if (Math.abs(xOffset) < 2.0) {
            drive.stop();
            telemetry.addData("ALIGN", "Aligned! xOffset=%.2f", xOffset);
            return true;
        }

        // Apply turning correction ONLY (no translation)
        drive.drive(0, 0, -turnPower);

        telemetry.addData("ALIGN", "xOffset=%.2f  turn=%.2f", xOffset, turnPower);
        return false;
    }

    private void updateTelemetry() {
        telemetry.addLine("=== SIMPLE AUTO ===");
        telemetry.addData("State", state);
        telemetry.addData("Shot Index", currentShotIndex);
        telemetry.addData("Shooter RPM", shooter.getCurrentRPM());
        telemetry.update();
    }
}
