package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import org.firstinspires.ftc.teamcode.SubSystems.Drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Shooter.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Spindexer.OldSpindexerSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Vision.AprilTagNavigator;

/**
 * Blue Alliance TeleOp with AprilTag alignment and automated scoring sequence.
 * 
 * Gamepad 1 (Driver):
 * - Left Stick: Drive forward/backward and strafe
 * - Right Stick X: Turn
 * - A/B: Drive speed toggle (1.0 / 0.5)
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

    // Spindexer control variables
    private int spindexerPositionIndex = 0;
    private boolean intakeMode = true; // true = Intake, false = Outtake
    private boolean spindexerIsMoving = false;
    private boolean shooterNeedsToSpinUp = false;
    private boolean ballSettling = false;
    private boolean manualControlMode = false; // true = Manual, false = Automated
    
    // Button state tracking
    private boolean spindexerPressLast = false;
    private boolean dpadDownLast = false;
    private boolean rbPressedLast = false;
    private boolean yPressedLast = false;
    private boolean xPressedLast = false; // Manual control toggle
    
    // Drive speed toggle
    private double driveSpeed = 1.0;
    private boolean aPressedLast = false;
    private boolean bPressedLast = false;

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
    
    // Ball settling: 0.25" counterclockwise
    private static final int BALL_SETTLING_TICKS = 34;
    
    // Shooter power
    private static final double SHOOTER_POWER = 0.87; // 70% = ~4200 RPM

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
            handleSpindexer();
            handleAprilTagAlignment();

            // ==================== TELEMETRY ====================
            updateTelemetry();
        }

        // Cleanup
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

        // Drive speed toggle
        boolean a = gamepad1.a;
        boolean b = gamepad1.b;
        if (a && !aPressedLast) driveSpeed = 1.0;
        if (b && !bPressedLast) driveSpeed = 0.5;
        aPressedLast = a;
        bPressedLast = b;

        // Store drive values for use in alignment (if not aligning)
        // Alignment will override turn when Y button is held
        if (!gamepad2.y) {
            // Normal driving - apply drive with speed multiplier
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
            if (shooter.getTargetRPM() > 0) {
                shooter.stop();
            } else {
                shooter.setPower(SHOOTER_POWER);
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
                // Switch to manual mode - stop automated movement
                spindexerIsMoving = false;
                ballSettling = false;
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
        // Manual position selection (D-Pad Left/Right)
        boolean dpadLeft = gamepad2.dpad_left;
        boolean dpadRight = gamepad2.dpad_right;
        
        if (dpadLeft && !spindexerIsMoving) {
            spindexerPositionIndex = (spindexerPositionIndex - 1 + 3) % 3; // Move backward
            spindexer.goToPositionForCurrentMode(spindexerPositionIndex);
            spindexerIsMoving = true;
        }
        if (dpadRight && !spindexerIsMoving) {
            spindexerPositionIndex = (spindexerPositionIndex + 1) % 3; // Move forward
            spindexer.goToPositionForCurrentMode(spindexerPositionIndex);
            spindexerIsMoving = true;
        }

        // Manual power control (Right Stick Y) - overrides position control
        float manualPower = -gamepad2.right_stick_y;
        if (Math.abs(manualPower) > 0.1) {
            spindexer.setManualPower(manualPower * 0.5); // Scale down for safety
            spindexerIsMoving = false; // Disable position tracking when manually controlling
        } else if (!spindexerIsMoving) {
            // If not manually controlling and not moving to position, stop motor
            spindexer.setManualPower(0);
        }

        // Update spindexer PID control (for position mode)
        if (spindexerIsMoving) {
            spindexer.update();
            if (spindexer.isAtPosition()) {
                spindexerIsMoving = false;
            }
        }
    }

    private void handleAutomatedSpindexer() {
        // Update spindexer PID control (must be called every loop)
        spindexer.update();

        // Check if spindexer reached target position
        if (spindexerIsMoving) {
            if (spindexer.isAtPosition()) {
                spindexerIsMoving = false;

                if (intakeMode) {
                    // INTAKE MODE: Start ball settling movement (counterclockwise)
                    // Use manual power control for small settling movement
                    // Note: This is a simplified approach - you may need to adjust
                    ballSettling = true;
                    // For now, we'll skip settling and mark as ready
                    // In a full implementation, you'd use setManualPower() for settling
                    ballSettling = false; // Simplified - remove this line if implementing full settling
                } else {
                    // OUTTAKE MODE: Start shooter spin-up
                    shooter.setPower(SHOOTER_POWER);
                    shooterNeedsToSpinUp = true;
                }
            }
        }

        // Check if ball settling is complete (if implemented)
        if (ballSettling) {
            // Settling logic would go here
            // For now, simplified version
            ballSettling = false;
        }

        // Check if shooter reached target RPM
        if (shooterNeedsToSpinUp) {
            if (shooter.isAtTargetRPM()) {
                shooterNeedsToSpinUp = false;
            }
        }

        // Spindexer advance button (A) - only in automated mode
        boolean spPress = gamepad2.a;
        boolean canMove = !spindexerIsMoving && !ballSettling &&
                (!shooterNeedsToSpinUp || shooter.isAtTargetRPM());

        if (spPress && !spindexerPressLast && canMove) {
            // Advance to next position
            spindexerPositionIndex = (spindexerPositionIndex + 1) % 3;
            
            // Use subsystem method that automatically selects position based on current mode
            // This uses intake positions in intake mode, outtake positions in outtake mode
            spindexer.goToPositionForCurrentMode(spindexerPositionIndex);
            spindexerIsMoving = true;
        }
        spindexerPressLast = spPress;
    }

    private void handleAprilTagAlignment() {
        boolean y = gamepad2.y;

        // Continuous alignment when button is held
        if (y) {
            // Update robot position from AprilTags
            aprilTag.updateRobotPositionFromAllianceGoals();
            AprilTagDetection blueGoal = aprilTag.getBestAllianceGoalDetection();

            if (blueGoal != null && blueGoal.id == TARGET_TAG_ID) {
                // Calculate left/right offset to determine alignment
                double xOffset = blueGoal.ftcPose.x;  // left/right offset in inches

                // Get current forward/strafe from gamepad (INVERTED - controller at back)
                float forward = gamepad1.left_stick_y;  // INVERTED
                float strafe = -gamepad1.left_stick_x;  // INVERTED
                double denominator = JavaUtil.maxOfList(JavaUtil.createListWith(0.5, 0.5));

                // Proportional control on X offset (left/right)
                double kP = 0.02;  // tune 0.015–0.025
                double turnPower = xOffset * kP;

                // Deadband to prevent jitter
                if (Math.abs(xOffset) < 0.5) turnPower = 0;

                turnPower = Math.max(-0.4, Math.min(0.4, turnPower));

                // Apply alignment turn while preserving forward/strafe movement
                drive.drive(
                        (forward / denominator) * driveSpeed,
                        (strafe / denominator) * driveSpeed,
                        -turnPower
                );

                telemetry.addData("Alignment", "X offset: %.2f, turn: %.2f", xOffset, turnPower);
            } else {
                telemetry.addData("Alignment", "Red Goal (Tag 24) not detected");
            }
        }
        yPressedLast = y;
    }

    private void updateTelemetry() {
        telemetry.addData("=== BLUE ALLIANCE TELEOP ===", "");
        telemetry.addData("Drive Speed", driveSpeed);
        telemetry.addData("Spindexer Control", manualControlMode ? "MANUAL" : "AUTOMATED");
        telemetry.addData("Spindexer Mode", intakeMode ? "INTAKE" : "OUTTAKE");
        telemetry.addData("Spindexer Position", spindexerPositionIndex);
        telemetry.addData("Spindexer Status", spindexerIsMoving ? "MOVING" :
                (ballSettling ? "SETTLING" :
                (shooterNeedsToSpinUp ? "WAITING FOR RPM" : "READY")));
        telemetry.addData("Shooter RPM", "%.0f / %.0f", shooter.getCurrentRPM(), shooter.getTargetRPM());
        telemetry.addData("Shooter At RPM", shooter.isAtTargetRPM());
        
        // AprilTag info
        aprilTag.updateDECODELocalizationTelemetry();
        
        telemetry.update();
    }

}

