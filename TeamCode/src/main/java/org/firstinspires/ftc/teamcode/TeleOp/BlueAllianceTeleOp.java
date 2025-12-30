package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import org.firstinspires.ftc.teamcode.SubSystems.Drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Shooter.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Spindexer.OldSpindexerSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Vision.AprilTagNavigator;
//import org.firstinspires.ftc.teamcode.SubSystems.Funnel.FunnelSubsystem;

/**
 * Blue Alliance TeleOp with AprilTag alignment and automated scoring sequence.
 * 
 * Gamepad 1 (Driver):
 * - Left Stick Y: Forward/backward
 * - Left Stick X: Strafe left/right
 * - Right Stick X: Turn
 * - Right Stick Y: Diagonal forward/backward (adds to forward movement)
 * - A/B: Drive speed toggle (1.0 / 0.5)
 * - D-Pad Up: Toggle inverse direction (for when controller is at back of robot)
 * 
 * Gamepad 2 (Operator):
 * - Left Trigger: Intake forward
 * - Left Bumper: Intake reverse
 * - Right Bumper: Toggle shooter on/off
 * - A Button: Advance spindexer position (automated sequence)
 * - X Button: Toggle manual/automated spindexer control
 * - D-Pad Down: Toggle intake/outtake mode
 * - D-Pad Left/Right: Manual spindexer control (when in manual mode)
 * - Right Stick Y: Manual spindexer power control (when in manual mode)
 * - Y Button: Align to Blue Alliance goal (Tag 20)
 * - B Button: Toggle funnel servos (extend/retract)
 * 
 * Automated Sequence:
 * - INTAKE: Move spindexer → Wait for position → Ball settling → Ready
 * - OUTTAKE: Move spindexer → Wait for position → Spin up shooter → Wait for RPM → Ready
 */
@TeleOp(name = "Blue Alliance TeleOp", group = "TeleOp")
public class BlueAllianceTeleOp extends LinearOpMode {

    // Subsystems
    private DriveSubsystem drive;
    private IntakeSubsystem intake;
    private ShooterSubsystem shooter;
    private OldSpindexerSubsystem spindexer;
    private AprilTagNavigator aprilTag;
//    private FunnelSubsystem funnel;

    // Spindexer control variables
    private int spindexerPositionIndex = 0;
    private boolean intakeMode = true; // true = Intake, false = Outtake
    private boolean spindexerIsMoving = false;
    private boolean shooterNeedsToSpinUp = true;
    private boolean ballSettling = false;
    private boolean manualControlMode = false; // true = Manual, false = Automated
    
    // Shooter control flags - separate from automated sequence
    private boolean shooterManuallyControlled = false; // Track if shooter is manually controlled
    private int shotNumber = 0;
    
    // Timers
    private ElapsedTime actionTimer; // Used for shooter stability
    private ElapsedTime shotTimer;   // Used for shot-to-shot delays
    
    // Button state tracking
    private boolean spindexerPressLast = false;
    private boolean dpadDownLast = false;
    private boolean rbPressedLast = false;
    private boolean yPressedLast = false;
    private boolean xPressedLast = false; // Manual control toggle
    private boolean dpadLeftLast = false;
    private boolean dpadRightLast = false;
    private boolean aPressedLast = false;
    private boolean bPressedLast = false;
    
    // Drive speed toggle
    private double driveSpeed = 1.0;

    // Spindexer positions (inspired by Decode20252026 but using subsystem)
    private static final double TICKS_PER_REVOLUTION = 2150.8;
    private static final int POS_TICK_120 = (int) (TICKS_PER_REVOLUTION / 3.0); // 717
    private static final int POS_TICK_240 = (int) (TICKS_PER_REVOLUTION * 2.0 / 3.0); // 1434
    
    // Intake positions (evenly spaced)
    private static final int[] INTAKE_POSITIONS = { 0, POS_TICK_120, POS_TICK_240 };
    
    // Outtake positions (measured values normalized)
    private static int normalizePosition(int rawPosition) {
        int normalized = rawPosition % (int) TICKS_PER_REVOLUTION;
        if (normalized < 0) normalized += (int) TICKS_PER_REVOLUTION;
        return normalized;
    }
    
    private static final int[] OUTTAKE_POSITIONS = {
        normalizePosition(-90),   // 2060
        normalizePosition(-265),  // 1885
        normalizePosition(-434)   // 1716
    };
    
    // === AUTO-ALIGNMENT CONSTANTS ===
    private static final double DESIRED_SHOOTING_DISTANCE = 134;
    private static final double DESIRED_SHOOTING_Angle = 21;
    private static final double KP_STRAFE = 0.03;
    private static final double KP_FORWARD = 0.03;
    private static final double KP_ROT = 0.015;

    private static final double MAX_AUTO_POWER = 0.40;
    private static final double POSITION_DEADBAND = 0.75;
    private static final double ANGLE_DEADBAND_DEG = 1.5;

    // Shooter RPM target
    private static final double SHOOTER_TARGET_RPM = 5220.0;

    // Alliance configuration
    private static final boolean IS_BLUE_ALLIANCE = true;
    private static final int TARGET_TAG_ID = 20; // Blue alliance goal tag

    @Override
    public void runOpMode() {
        // Initialize subsystems
        drive = new DriveSubsystem(hardwareMap, telemetry);
        intake = new IntakeSubsystem(hardwareMap, telemetry);
        shooter = new ShooterSubsystem(hardwareMap, telemetry);
        spindexer = new OldSpindexerSubsystem(hardwareMap, telemetry);
        aprilTag = new AprilTagNavigator(drive, hardwareMap, telemetry);
//        funnel = new FunnelSubsystem(hardwareMap, telemetry);
        
        // Initialize Timers
        shotTimer = new ElapsedTime();
        actionTimer = new ElapsedTime();

        telemetry.addData("Status", "Initialized - Blue Alliance");
        telemetry.addData("Target Goal", "Blue Alliance (Tag 20)");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // ==================== GAMEPAD 1: DRIVING ====================
            handleDriving();

            // ==================== GAMEPAD 2: OPERATOR CONTROLS ====================
            handleIntake();
            handleShooter();
            shooter.updateVoltageCompensation();
            handleSpindexer();
            enforceShooterSpindexerSafety();
//            handleFunnel();
            handleAprilTagAlignment();

            // ==================== TELEMETRY ====================
            updateTelemetry();
        }

        // Cleanup
        drive.stop();
        intake.stop();
        shooter.stop();
//        funnel.retract(); // Retract funnels on stop
        aprilTag.closeVision();
    }

    private void handleDriving() {
        // Inverse direction toggle (D-Pad Up on Gamepad 1)
        boolean dpadUp = gamepad1.dpad_up;
        if (dpadUp && !dpadUpLast) {
            inverseDirection = !inverseDirection;
        }
        dpadUpLast = dpadUp;
        
        // Get standard controls from left stick
        float forward = gamepad1.left_stick_y;
        float strafe = -gamepad1.left_stick_x;
        float turn = -gamepad1.right_stick_x;
        
        // Add diagonal forward/backward movement from right stick Y axis
        // Right stick Y = diagonal forward/backward (adds to base forward movement)
        float diagonalForward = gamepad1.right_stick_y;
        
        // Combine normal movement with diagonal forward/backward
        forward += diagonalForward;
        
        // Apply inverse direction if enabled
        if (inverseDirection) {
            forward = -forward;
            strafe = -strafe;
            turn = -turn;
        }
        
        double denominator = JavaUtil.maxOfList(JavaUtil.createListWith(0.5, 0.5 + Math.abs(turn)));

        if (gamepad1.a && !aPressedLast) driveSpeed = 1.0;
        if (gamepad1.b && !bPressedLast) driveSpeed = 0.5;
        aPressedLast = gamepad1.a;
        bPressedLast = gamepad1.b;

        // Store drive values for use in alignment (if not aligning)
        // Alignment will override turn when Y button is held
        if (!gamepad2.y) {
            // Normal driving - apply drive with speed multiplier
            // Diagonal movement is automatically handled by mecanum kinematics
            drive.drive((forward / denominator) * driveSpeed, 
                       (strafe / denominator) * driveSpeed, 
                       (turn / denominator) * driveSpeed);
        } else {
            // When aligning, only allow forward/strafe, turn is controlled by alignment
            drive.drive((forward / denominator) * driveSpeed, 
                       (strafe / denominator) * driveSpeed, 
                       0); // Turn will be set by alignment
        }
    }

    private void handleIntake() {
        if (gamepad2.left_trigger > 0.1) {
            intake.setPower(1.0); // Forward intake
        } else if (gamepad2.left_bumper) {
            intake.setPower(-1.0); // Reverse/outtake
        } else {
            intake.setPower(0);
        }
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
        // Manual/Automated control toggle (X Button)
        boolean x = gamepad2.x;
        if (x && !xPressedLast) {
            manualControlMode = !manualControlMode;
            if (manualControlMode) {
                // Switch to manual mode - stop automated movement and disable PID
                spindexerIsMoving = false;
                ballSettling = false;
                spindexer.setPIDEnabled(false);
                spindexer.setManualPower(0); // Stop any current movement
            } else {
                // Switch back to automated mode - re-enable PID
                spindexer.setPIDEnabled(true);
            }
        }
        xPressedLast = x;

        // Mode toggle (D-Pad Down) - only in automated mode
        boolean dpadDown = gamepad2.dpad_down;
        if (dpadDown && !dpadDownLast && !manualControlMode) {
            intakeMode = !intakeMode;
            spindexerPositionIndex = 0; // Reset position when mode changes
            spindexer.setIntakeMode(intakeMode); // Update subsystem mode for position selection
        }
        dpadDownLast = dpadDown;

        // Manual control mode
        if (manualControlMode) {
            handleManualSpindexer();
            return; // Skip automated logic
        }

        // Automated control mode
        handleAutomatedSpindexer();
    }

    private void handleManualSpindexer() {
        float manualPower = -gamepad2.right_stick_y;
        if (Math.abs(manualPower) > 0.1) {
            spindexer.setManualPower(manualPower * OldSpindexerSubsystem.getRecommendedManualPowerMultiplier());
            spindexerIsMoving = false;
        } else {
            spindexer.setManualPower(0);
        }

        if (gamepad2.dpad_left && !dpadLeftLast && !spindexerIsMoving) {
            spindexerPositionIndex = (spindexerPositionIndex - 1 + 3) % 3;
            spindexer.setPIDEnabled(true); // Re-enable PID for position control
            spindexer.goToPosition(spindexerPositionIndex);
            spindexerIsMoving = true;
        }
        if (gamepad2.dpad_right && !dpadRightLast && !spindexerIsMoving) {
            spindexerPositionIndex = (spindexerPositionIndex + 1) % 3;
            spindexer.setPIDEnabled(true); // Re-enable PID for position control
            spindexer.goToPosition(spindexerPositionIndex);
            spindexerIsMoving = true;
        }
        dpadLeftLast = gamepad2.dpad_left;
        dpadRightLast = gamepad2.dpad_right;

        if (spindexerIsMoving) {
            spindexer.update();
            if (spindexer.isAtPosition()) spindexerIsMoving = false;
        }
    }

    private void handleAutomatedSpindexer() {
        spindexer.update();

        if (spindexerIsMoving && spindexer.isAtPosition()) {
            spindexerIsMoving = false;
            if (!intakeMode && !shooterManuallyControlled) {
                shooter.setTargetRPM(SHOOTER_TARGET_RPM);
                shooterNeedsToSpinUp = true;
                actionTimer.reset();
                shotTimer.reset();
            }
        }

        if (shooterNeedsToSpinUp && !shooterManuallyControlled) {
            if (shooter.isAtTargetRPM()) {
                long stabilityDelay = (shotNumber == 0) ? 300 : 200;
                if (actionTimer.milliseconds() > stabilityDelay) {
                    shooterNeedsToSpinUp = false;
                }
            } else {
                actionTimer.reset();
            }
        }

        boolean spPress = gamepad2.a;
        long minDelayBetweenShots = (!intakeMode && spindexerPositionIndex == 0) ? 400 : 250;
        
        boolean canMove = !spindexerIsMoving &&
                (!shooterNeedsToSpinUp || shooter.isAtTargetRPM() || shooterManuallyControlled) &&
                (shotTimer.milliseconds() > minDelayBetweenShots || intakeMode);

        if (spPress && !spindexerPressLast && canMove) {
            if (!intakeMode) shotNumber = spindexerPositionIndex;
            spindexerPositionIndex = (spindexerPositionIndex + 1) % 3;
            spindexer.goToPositionForCurrentMode(spindexerPositionIndex);
            spindexerIsMoving = true;
            actionTimer.reset();
            shotTimer.reset();
        }
        spindexerPressLast = spPress;
    }
//
//    private void handleFunnel() {
//        // Funnel toggle (B Button on Gamepad 2)
//        boolean b = gamepad2.b;
//        if (b && !b2PressedLast) {
//            funnel.toggle();
//        }
//        bPressedLast = b;
//    }

    private void enforceShooterSpindexerSafety() {
        if (spindexer.isMoving() || !spindexer.isAtPosition()) {
            if (shooter.getTargetRPM() > 0 && !shooterManuallyControlled) {
                shooter.stop();
                shooterNeedsToSpinUp = true;
            }
        }
    }

    private void handleAprilTagAlignment() {
        if (gamepad2.y) {
            aprilTag.updateRobotPositionFromAllianceGoals();
            AprilTagDetection tag = aprilTag.getBestAllianceGoalDetection();

            if (tag == null || tag.id != TARGET_TAG_ID) {
                drive.drive(0, 0, 0);
                return;
            }

            if (!shooterManuallyControlled) shooter.setTargetRPM(SHOOTER_TARGET_RPM);

            double[] corrections = aprilTag.calculateAlignmentCorrections(
                    tag, DESIRED_SHOOTING_DISTANCE, DESIRED_SHOOTING_Angle,
                    POSITION_DEADBAND, POSITION_DEADBAND, ANGLE_DEADBAND_DEG,
                    KP_STRAFE, KP_FORWARD, KP_ROT, MAX_AUTO_POWER);

            if (corrections != null) {
                drive.drive(corrections[1], corrections[0], corrections[2]);
            }
        }
    }

    private void updateTelemetry() {
        telemetry.addData("Spindexer", manualControlMode ? "MANUAL" : "AUTO");
        telemetry.addData("Mode", intakeMode ? "INTAKE" : "OUTTAKE");
        telemetry.addData("RPM", "%.0f / %.0f", shooter.getCurrentRPM(), shooter.getTargetRPM());
        telemetry.update();
    }

}

