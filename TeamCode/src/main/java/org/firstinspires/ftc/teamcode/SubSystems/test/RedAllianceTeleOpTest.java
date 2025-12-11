package org.firstinspires.ftc.teamcode.SubSystems.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.teamcode.SubSystems.Sensors.ColorSensorSubsystem;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import org.firstinspires.ftc.teamcode.SubSystems.Drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Shooter.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Spindexer.OldSpindexerSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Vision.AprilTagNavigator;
import org.firstinspires.ftc.teamcode.SubSystems.Vision.ObeliskMotifDetector;
import org.firstinspires.ftc.teamcode.SubSystems.Scoring.AutoOuttakeController;
import org.firstinspires.ftc.teamcode.SubSystems.Scoring.ArtifactColor;

/**
 * RED ALLIANCE TEST VERSION
 */
@TeleOp(name = "Red Alliance TeleOp TEST", group = "TeleOp")
public class RedAllianceTeleOpTest extends LinearOpMode {

    // === AUTO-ALIGNMENT CONSTANTS ADDED HERE ===
    private static final double DESIRED_SHOOTING_DISTANCE = 134;  // inches from tag angle
    private static final double DESIRED_SHOOTING_Angle = 21;
    private static final double KP_STRAFE = 0.03;                  // x alignment
    private static final double KP_FORWARD = 0.03;                 // z alignment
    private static final double KP_ROT = 0.015;                    // yaw correction

    private static final double MAX_AUTO_POWER = 0.40;             // slow alignment mode
    private static final double POSITION_DEADBAND = 0.75;          // inch accuracy
    private static final double ANGLE_DEADBAND_DEG = 1.5;          // degree accuracy


    private DriveSubsystem drive;
    private IntakeSubsystem intake;
    private ShooterSubsystem shooter;
    private OldSpindexerSubsystem spindexer;
    private AprilTagNavigator aprilTag;

    private ColorSensorSubsystem colorSensorSubsystem;

    // New: vision -> obelisk motif + auto outtake controller
    private ObeliskMotifDetector obeliskDetector;
    private NormalizedColorSensor colorSensor;
    private AutoOuttakeController autoOuttake;

    private int spindexerPositionIndex = 0;
    private boolean intakeMode = true;
    private boolean spindexerIsMoving = false;
    private boolean shooterNeedsToSpinUp = true;
    private boolean ballSettling = false;
    private boolean manualControlMode = false;

    private boolean shooterManuallyControlled = false;

    private boolean spindexerPressLast = false;
    private boolean dpadDownLast = false;
    private boolean rbPressedLast = false;
    private boolean yPressedLast = false;
    private boolean xPressedLast = false;

    private double driveSpeed = 1.0;
    private boolean aPressedLast = false;
    private boolean bPressedLast = false;

    private static final double TICKS_PER_REVOLUTION = 2150.8;
    private static final int POS_TICK_120 = (int) (TICKS_PER_REVOLUTION / 3.0);
    private static final int POS_TICK_240 = (int) (TICKS_PER_REVOLUTION * 2.0 / 3.0);

    private static final int[] INTAKE_POSITIONS = { 0, POS_TICK_120, POS_TICK_240 };

    private static int normalizePosition(int rawPosition) {
        int normalized = rawPosition % (int) TICKS_PER_REVOLUTION;
        if (normalized < 0) normalized += (int) TICKS_PER_REVOLUTION;
        return normalized;
    }

    private static final int[] OUTTAKE_POSITIONS = {
            normalizePosition(-90),
            normalizePosition(-265),
            normalizePosition(-434)
    };

    private static final int BALL_SETTLING_TICKS = 34;
   
    private static final double SHOOTER_TARGET_RPM = 5220.0;


    private static final boolean IS_BLUE_ALLIANCE = false;
    private static final int TARGET_TAG_ID = 24;

    @Override
    public void runOpMode() {
        // initialize subsystems
        drive = new DriveSubsystem(hardwareMap, telemetry);
        intake = new IntakeSubsystem(hardwareMap, telemetry);
        shooter = new ShooterSubsystem(hardwareMap, telemetry);
        spindexer = new OldSpindexerSubsystem(hardwareMap, telemetry);
        aprilTag = new AprilTagNavigator(drive, hardwareMap, telemetry);
        NormalizedColorSensor normColor =
                hardwareMap.get(NormalizedColorSensor.class, "sensor_color");

        ArtifactColor[] motif = new ArtifactColor[]{
                ArtifactColor.PURPLE, ArtifactColor.GREEN, ArtifactColor.PURPLE
        };
        colorSensorSubsystem = new ColorSensorSubsystem(normColor, null, telemetry, motif);


        // instantiate obelisk detector (uses aprilTag navigator)
        obeliskDetector = new ObeliskMotifDetector(aprilTag, telemetry);

        // color sensor (used by auto outtake)
        try {
            colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
        } catch (Exception e) {
            colorSensor = null;
            telemetry.addData("ColorSensor", "sensor_color not found");
        }

        telemetry.addData("Status", "Initialized - RED ALLIANCE TEST");
        telemetry.update();

        // --- INIT window: try to read the OBELISK motif before match start ---
        telemetry.addData("Obelisk", "Scanning during init...");
        telemetry.update();
        long initStart = System.currentTimeMillis();
        // give up to 3 seconds in init to read obelisk; short loop to remain responsive
        while (!isStarted() && !isStopRequested() && System.currentTimeMillis() - initStart < 3000) {
            // ask aprilTag subsystem to update its detections if it has such a method (optional)
            // e.g. aprilTag.getAprilTagProcessor().processOnceIfNeeded(); // optional - only if your processor requires it
            obeliskDetector.update();
            telemetry.update();
            sleep(50);
        }

        waitForStart();

        // After start: lock motif (use detected motif or fallback)
        motif = obeliskDetector.getMotif();
        if (motif == null) {
            // fallback: either choose a default motif or let driver decide (default here is GPG/GPG-like)
            motif = new ArtifactColor[]{ArtifactColor.GREEN, ArtifactColor.PURPLE, ArtifactColor.GREEN};
            telemetry.addData("Obelisk", "No motif read - using fallback %s", "GPG");
        } else {
            telemetry.addData("Obelisk", "Using motif: " + motif[0].getCharacter() + motif[1].getCharacter() + motif[2].getCharacter());
        }
        telemetry.update();

        // instantiate auto-outtake controller if we have a color sensor
        if (colorSensor != null) {
            autoOuttake = new AutoOuttakeController(colorSensorSubsystem, motif, spindexer, shooter, telemetry);// tune defaults if needed:
            autoOuttake.setScoreThreshold(6);
            autoOuttake.setTargetShooterRPM(SHOOTER_TARGET_RPM);
        } else {
            autoOuttake = null;
        }

        while (opModeIsActive()) {
            // update components regularly
            handleDriving();
            handleIntake();
            handleShooter();
            shooter.updateVoltageCompensation();

            // update AprilTag/obelisk if you want live re-scan (optional)
            obeliskDetector.update();

            // run auto outtake state machine (reads color sensor + sequences)
            if (autoOuttake != null) autoOuttake.update();

            // normal spindexer handling and update
            handleSpindexer();

            // alignment assistance using goal tags
            handleAprilTagAlignment();

            updateTelemetry();
        }

        drive.stop();
        intake.stop();
        shooter.stop();
        aprilTag.closeVision();
    }

   

    private void handleDriving() {
        float forward = gamepad1.left_stick_y;
        float strafe = -gamepad1.left_stick_x;
        float turn = -gamepad1.right_stick_x;

        double denominator = JavaUtil.maxOfList(JavaUtil.createListWith(0.5, 0.5 + Math.abs(turn)));

        boolean a = gamepad1.a;
        boolean b = gamepad1.b;
        if (a && !aPressedLast) driveSpeed = 1.0;
        if (b && !bPressedLast) driveSpeed = 0.5;
        aPressedLast = a;
        bPressedLast = b;

        if (!gamepad2.y) {
            drive.drive((forward / denominator) * driveSpeed,
                    (strafe / denominator) * driveSpeed,
                    (turn / denominator) * driveSpeed);
        } else {
            drive.drive((forward / denominator) * driveSpeed,
                    (strafe / denominator) * driveSpeed,
                    0);
        }
    }

    private void handleIntake() {
        if (gamepad2.left_trigger > 0.1) intake.setPower(1.0);
        else if (gamepad2.left_bumper) intake.setPower(-1.0);
        else intake.setPower(0);
    }

   private void handleShooter() {
    boolean rb = gamepad2.right_bumper;
    if (rb && !rbPressedLast) {
        shooterManuallyControlled = true;
        shooterNeedsToSpinUp = false;

        if (shooter.getTargetRPM() > 0) {
            shooter.stop();
            shooterManuallyControlled = false;
        } else {
            shooter.setTargetRPM(SHOOTER_TARGET_RPM);
        }
    }
    rbPressedLast = rb;
}

    private void handleSpindexer() {
        boolean x = gamepad2.x;
        if (x && !xPressedLast) {
            manualControlMode = !manualControlMode;
            if (manualControlMode) {
                spindexerIsMoving = false;
                ballSettling = false;
            }
        }
        xPressedLast = x;

        boolean dpadDown = gamepad2.dpad_down;
        if (dpadDown && !dpadDownLast && !manualControlMode) {
            intakeMode = !intakeMode;
            spindexerPositionIndex = 0;
        }
        dpadDownLast = dpadDown;

        if (manualControlMode) {
            handleManualSpindexer();
            return;
        }

        handleAutomatedSpindexer();
    }

    private void handleManualSpindexer() {
        boolean dpadLeft = gamepad2.dpad_left;
        boolean dpadRight = gamepad2.dpad_right;

        float manualPower = -gamepad2.right_stick_y;
        if (Math.abs(manualPower) > 0.1) {
            spindexer.setManualPower(manualPower * 0.5);
            spindexerIsMoving = false;
            return;
        } else {
            spindexer.setManualPower(0);
        }

        if (dpadLeft && !spindexerIsMoving) {
            spindexerPositionIndex = (spindexerPositionIndex - 1 + 3) % 3;
            spindexer.goToPosition(spindexerPositionIndex);
            spindexerIsMoving = true;
        }
        if (dpadRight && !spindexerIsMoving) {
            spindexerPositionIndex = (spindexerPositionIndex + 1) % 3;
            spindexer.goToPosition(spindexerPositionIndex);
            spindexerIsMoving = true;
        }

        if (spindexerIsMoving) {
            spindexer.update();
            if (spindexer.isAtPosition()) spindexerIsMoving = false;
        }
    }
    
    private int getCurrentModeTargetTicks(int index) {
        if (intakeMode) {
        return INTAKE_POSITIONS[index];
    } else {
        return OUTTAKE_POSITIONS[index];
    }
}

    private void handleAutomatedSpindexer() {
        spindexer.update();

        if (spindexerIsMoving) {
            if (spindexer.isAtPosition()) {
                spindexerIsMoving = false;

                if (intakeMode) {
                    ballSettling = true;
                    ballSettling = false;
                } else {
                    if (!shooterManuallyControlled) {
            shooter.setTargetRPM(SHOOTER_TARGET_RPM);
                        shooterNeedsToSpinUp = true;
                    }
                }
            }
        }

        if (ballSettling) ballSettling = false;

        if (shooterNeedsToSpinUp && !shooterManuallyControlled) {
            if (shooter.isAtTargetRPM()) shooterNeedsToSpinUp = false;
        }

        boolean spPress = gamepad2.a;
        boolean canMove =
                !spindexerIsMoving &&
                        !ballSettling &&
                        (!shooterNeedsToSpinUp || shooter.isAtTargetRPM() || shooterManuallyControlled);

        if (spPress && !spindexerPressLast && canMove) {
            spindexerPositionIndex = (spindexerPositionIndex + 1) % 3;
           spindexer.goToPosition(getCurrentModeTargetTicks(spindexerPositionIndex));
            spindexerIsMoving = true;
        }
        spindexerPressLast = spPress;
    }


   private void handleAprilTagAlignment() {
    if (gamepad2.y) {
        aprilTag.updateRobotPositionFromAllianceGoals();
        AprilTagDetection tag = aprilTag.getBestAllianceGoalDetection();

        if (tag == null || tag.id != TARGET_TAG_ID) {
            telemetry.addData("ALIGN", "Tag NOT found");
            drive.drive(0, 0, 0); // Stop movement if tag is lost
            return;
        }

        // Force shooter to spin while aligning
        if (!shooterManuallyControlled) {
            shooter.setTargetRPM(SHOOTER_TARGET_RPM);
        }

        double xOffset = tag.ftcPose.x;
        double forwardError = tag.ftcPose.y - DESIRED_SHOOTING_DISTANCE;
        double angleError = tag.ftcPose.yaw + DESIRED_SHOOTING_Angle;
        
        // --- STAGE 1: ANGLE CORRECTION ---
        if (Math.abs(angleError) > ANGLE_DEADBAND_DEG) {
            
            // Proportional (P) control for turning
            double turnPower = KP_ROT * angleError;
            
            // Clamp turn power
            turnPower = Range.clip(turnPower, -MAX_AUTO_POWER, MAX_AUTO_POWER);

            // Drive ONLY the turn power
            drive.drive(0, 0, turnPower);
            
            telemetry.addData("ALIGN", "STAGE 1: CORRECTING ANGLE...");
            telemetry.addData("  Yaw Error", "%.2f°", angleError);

        } 
        // --- STAGE 2: POSITION CORRECTION ---
        else {
            
            double strafePower  = KP_STRAFE  * xOffset;
            double forwardPower = KP_FORWARD * forwardError;
            double turnPower    = 0; // Angle is locked, so turnPower is ZERO

            // Deadbands for position
            if (Math.abs(xOffset) < POSITION_DEADBAND) strafePower = 0;
            if (Math.abs(forwardError) < POSITION_DEADBAND) forwardPower = 0;
            
            // Clamp position powers
            strafePower  = Range.clip(strafePower,  -MAX_AUTO_POWER, MAX_AUTO_POWER);
            forwardPower = Range.clip(forwardPower, -MAX_AUTO_POWER, MAX_AUTO_POWER);

            drive.drive(
                    forwardPower,
                    strafePower,
                    turnPower // This is 0
            );
            
            boolean positionAligned = (strafePower == 0 && forwardPower == 0);
            
            telemetry.addData("ALIGN", positionAligned ? "LOCKED ON" : "STAGE 2: CORRECTING POSITION...");
            telemetry.addData("  X Offset", "%.2f in", xOffset);
            telemetry.addData("  Y Dist", "%.2f in (Target %.1f)", tag.ftcPose.y, DESIRED_SHOOTING_DISTANCE);
            telemetry.addData("  Yaw (Locked)", "%.2f°", angleError); // show locked error
        }
    }
}

    private void updateTelemetry() {
        telemetry.addData("=== RED ALLIANCE TELEOP TEST ===", "");
        telemetry.addData("Drive Speed", driveSpeed);
        telemetry.addData("Spindexer Control", manualControlMode ? "MANUAL" : "AUTOMATED");
        telemetry.addData("Spindexer Mode", intakeMode ? "INTAKE" : "OUTTAKE");
        telemetry.addData("Spindexer Position", spindexerPositionIndex);
        telemetry.addData("Shooter RPM", "%.0f / %.0f",
                shooter.getCurrentRPM(), shooter.getTargetRPM());
        aprilTag.updateDECODELocalizationTelemetry();
        telemetry.update();
    }
}