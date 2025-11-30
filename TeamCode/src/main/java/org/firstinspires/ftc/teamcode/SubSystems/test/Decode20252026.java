package org.firstinspires.ftc.teamcode.SubSystems.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.teamcode.SubSystems.Shooter.ShooterSubsystem;

@TeleOp(name = "Decode20252026 (SPINDEXER CUMULATIVE FIX)")
public class Decode20252026 extends LinearOpMode {

    private DcMotor rightFront, rightRear, leftFront, leftRear;
    private DcMotor intake;
    private DcMotor spindexer;

    // OLD SHOOTER CODE (commented out - replaced with ShooterSubsystem)
    // private DcMotor shooterL, shooterR;

    // Shooter subsystem for RPM control
    private ShooterSubsystem shooter;

    // --- SPINDEXER CONTROL VARIABLES ---
    private boolean spindexerPressLast = false;
    private boolean dpadDownLast = false;
    private int spindexerPosition = 0;

    // Spindexer power settings - different speeds for intake vs outtake
    private static final double INTAKE_SPINDEXER_POWER = 0.4; // Slower for ball entry
    private static final double OUTTAKE_SPINDEXER_POWER = 0.8; // Faster for shooting

    // Encoder definitions - Using GoBILDA RS-555 with 2150.8 ticks per revolution
    // If your motor has different TPR, adjust this value
    private static final double TICKS_PER_REVOLUTION = 2150.0;
    private static final int POS_TICK_120 = (int) (TICKS_PER_REVOLUTION / 3.0); // approx 716
    private static final int POS_TICK_240 = (int) (TICKS_PER_REVOLUTION * 2.0 / 3.0); // approx 1433

    // Intake positions (for intaking balls) - evenly spaced at 120° intervals
    private static final int[] INTAKE_POSITIONS = { 0, POS_TICK_120, POS_TICK_240 };

    // Outtake positions (for shooting balls) - measured encoder values, normalized
    // Measured values: {-90, -265, -434} normalized to 0-2150 range
    // Note: Position 3 (index 2) shoots first in counterclockwise sequence
    private static final int[] OUTTAKE_POSITIONS = {
            normalizePosition(-90), // Position 1 (index 0)
            normalizePosition(-265), // Position 2 (index 1)
            normalizePosition(-434) // Position 3 (index 2) - shoots first
    };

    // Helper method to normalize negative encoder values to 0-2150 range
    private static int normalizePosition(int rawPosition) {
        int normalized = rawPosition % (int) TICKS_PER_REVOLUTION;
        if (normalized < 0) {
            normalized += (int) TICKS_PER_REVOLUTION;
        }
        return normalized;
    }

    // Get the appropriate position array based on mode
    private int[] getPositionArray() {
        return intakeMode ? INTAKE_POSITIONS : OUTTAKE_POSITIONS;
    }

    // Position tolerance for "at target" checking (in encoder ticks)
    private static final int POSITION_TOLERANCE = 15; // Motor is considered "at position" within 15 ticks

    // Ball settling: 0.25 inches counterclockwise to help ball settle into
    // spindexer
    // Formula: (0.25 inches) / (2π × radius) × 2150 ticks
    // Adjust BALL_SETTLING_TICKS based on your spindexer radius:
    // - 2.5" radius ≈ 34 ticks
    // - 3.0" radius ≈ 28 ticks
    private static final int BALL_SETTLING_TICKS = 34; // Adjust based on your spindexer size

    // **Cumulative target variable**: This stores the absolute, running target.
    private int absoluteTarget = 0;

    // Flag to track if spindexer is currently moving to a target
    private boolean spindexerIsMoving = false;

    // Flag to track if ball settling movement is active (intake mode only)
    private boolean ballSettling = false;
    private int settlingTarget = 0;

    // Flag to track if shooter needs to spin up after spindexer reaches position
    private boolean shooterNeedsToSpinUp = false;

    // Shooter power setting (adjustable)
    private static final double SHOOTER_POWER = 0.7; // 70% power = ~4200 RPM (adjust as needed)

    private boolean intakeMode = true; // true = Intake, false = Outtake

    // Shooter manual toggle (for testing)
    private boolean shooterOn = false;
    private boolean rbPressedLast = false;

    // Drive speed toggle
    private double driveSpeed = 1.0;
    private boolean aPressedLast = false;
    private boolean bPressedLast = false;

    @Override
    public void runOpMode() {
        float forward, strafe, turn;
        double denominator;

        // --- HARDWARE MAPPING ---
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");

        intake = hardwareMap.get(DcMotor.class, "Intake");
        spindexer = hardwareMap.get(DcMotor.class, "Spindexer");

        // OLD SHOOTER CODE (commented out - replaced with ShooterSubsystem)
        // shooterL = hardwareMap.get(DcMotor.class, "shooterL");
        // shooterR = hardwareMap.get(DcMotor.class, "shooterR");

        // Initialize shooter subsystem
        shooter = new ShooterSubsystem(hardwareMap, telemetry);

        // --- SPINDEXER ENCODER SETUP ---
        spindexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // 1. Stop and Reset Encoder
        spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // 2. Initialize absolute target to 0 (CRITICAL: must set before switching
        // modes)
        absoluteTarget = 0;
        spindexer.setTargetPosition(absoluteTarget);

        // 3. Switch to RUN_TO_POSITION mode (must be after setTargetPosition)
        spindexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // --- MOTOR DIRECTIONS ---
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);

        intake.setDirection(DcMotor.Direction.FORWARD);

        // OLD SHOOTER CODE (commented out - replaced with ShooterSubsystem)
        // shooterL.setDirection(DcMotor.Direction.FORWARD);
        // shooterR.setDirection(DcMotor.Direction.REVERSE);

        // Set Spindexer direction - CRITICAL FIX:
        // If encoder counts NEGATIVE when target is POSITIVE, change this to REVERSE
        // If encoder counts POSITIVE when target is POSITIVE, keep as FORWARD
        // Based on your videos showing negative counts, try REVERSE first
        spindexer.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {

            // --- DRIVE (Unchanged) ---
            forward = -gamepad1.left_stick_y;
            strafe = gamepad1.left_stick_x;
            turn = gamepad1.right_stick_x;
            denominator = JavaUtil.maxOfList(JavaUtil.createListWith(0.5, 0.5 + Math.abs(turn)));

            boolean a = gamepad1.a;
            boolean b = gamepad1.b;
            if (a && !aPressedLast)
                driveSpeed = 1.0;
            if (b && !bPressedLast)
                driveSpeed = 0.5;
            aPressedLast = a;
            bPressedLast = b;

            leftFront.setPower(((forward + strafe + turn) / denominator) * driveSpeed);
            leftRear.setPower(((forward - (strafe - turn)) / denominator) * driveSpeed);
            rightFront.setPower(((forward - (strafe + turn)) / denominator) * driveSpeed);
            rightRear.setPower(((forward + (strafe - turn)) / denominator) * driveSpeed);

            // --- SHOOTER & INTAKE ---
            // Manual shooter toggle (for testing - can be disabled if using automatic
            // sequence)
            boolean rb = gamepad2.right_bumper;
            if (rb && !rbPressedLast) {
                shooterOn = !shooterOn;
                if (shooterOn) {
                    shooter.setPower(SHOOTER_POWER);
                } else {
                    shooter.stop();
                }
            }
            rbPressedLast = rb;

            // OLD SHOOTER CODE (commented out - replaced with ShooterSubsystem)
            // boolean rb = gamepad2.right_bumper;
            // if (rb && !rbPressedLast) shooterOn = !shooterOn;
            // rbPressedLast = rb;
            // shooterL.setPower(shooterOn ? 1.0 : 0);
            // shooterR.setPower(shooterOn ? 1.0 : 0);

            if (gamepad2.left_trigger > 0.1)
                intake.setPower(-1.0);
            else if (gamepad2.left_bumper)
                intake.setPower(1.0);
            else
                intake.setPower(0);

            // --- MODE TOGGLE (D-PAD DOWN) ---
            boolean dpadDown = gamepad2.dpad_down;
            if (dpadDown && !dpadDownLast) {
                intakeMode = !intakeMode;
                spindexerPosition = 0; // reset index when mode changes
            }
            dpadDownLast = dpadDown;

            // --- SPINDEXER CONTROL (A button) ---
            // INTAKE Sequence: Move spindexer → Wait for position → Ball settling (0.25"
            // counterclockwise)
            // OUTTAKE Sequence: Move spindexer → Wait for position → Spin up shooter → Wait
            // for RPM

            // Step 1: Check if spindexer has reached its target position
            if (spindexerIsMoving) {
                if (isSpindexerAtTarget()) {
                    // Spindexer has reached target - stop motor
                    spindexer.setPower(0);
                    spindexerIsMoving = false;

                    if (intakeMode) {
                        // INTAKE MODE: Start ball settling movement (counterclockwise)
                        settlingTarget = spindexer.getCurrentPosition() - BALL_SETTLING_TICKS;
                        spindexer.setTargetPosition(settlingTarget);
                        spindexer.setPower(INTAKE_SPINDEXER_POWER);
                        ballSettling = true;
                    } else {
                        // OUTTAKE MODE: Start shooter spin-up
                        shooter.setPower(SHOOTER_POWER);
                        shooterNeedsToSpinUp = true;
                    }
                }
            }

            // Step 2: Check if ball settling is complete (INTAKE mode only)
            if (ballSettling) {
                int currentPos = spindexer.getCurrentPosition();
                if (Math.abs(currentPos - settlingTarget) <= POSITION_TOLERANCE) {
                    spindexer.setPower(0);
                    ballSettling = false;
                    // Ball is settled, ready for next intake position
                }
            }

            // Step 3: Check if shooter has reached target RPM (OUTTAKE mode only)
            if (shooterNeedsToSpinUp) {
                if (shooter.isAtTargetRPM()) {
                    // Shooter is at target RPM - ready for next spindexer movement
                    shooterNeedsToSpinUp = false;
                }
            }

            boolean spPress = gamepad2.a;
            // Only process button press if:
            // - Spindexer is NOT currently moving
            // - Ball settling is NOT active (intake mode)
            // - Shooter is at target RPM (or doesn't need to spin up) (outtake mode)
            boolean canMoveSpindexer = !spindexerIsMoving && !ballSettling &&
                    (!shooterNeedsToSpinUp || shooter.isAtTargetRPM());

            if (spPress && !spindexerPressLast && canMoveSpindexer) {
                // Advance position index
                spindexerPosition = (spindexerPosition + 1) % 3;

                // 1. Get the target position from the appropriate array (intake or outtake)
                int[] currentPositions = getPositionArray();
                int nextNormalizedTarget = currentPositions[spindexerPosition];

                // 2. Get the motor's CURRENT RAW POSITION (not the previous target!)
                // This is the CRITICAL FIX - we must use actual encoder position, not
                // absoluteTarget
                int currentRawPosition = spindexer.getCurrentPosition();

                // 3. Normalize the current position to 0-TICKS_PER_REVOLUTION range
                int currentNormalizedPosition = currentRawPosition % (int) TICKS_PER_REVOLUTION;
                if (currentNormalizedPosition < 0) {
                    currentNormalizedPosition += (int) TICKS_PER_REVOLUTION;
                }

                // 4. Calculate the shortest distance forward to reach the next normalized
                // target
                int travelDifference = nextNormalizedTarget - currentNormalizedPosition;

                // If the travel difference is negative, wrap around (shortest path forward)
                // Example: from 1433 to 0, difference is -1433, so add 2150 to get 717
                if (travelDifference < 0) {
                    travelDifference += (int) TICKS_PER_REVOLUTION;
                }

                // 5. Calculate NEW absolute target based on CURRENT raw position + travel
                // difference
                // This ensures we always move forward from where we actually are, not where we
                // thought we were
                absoluteTarget = currentRawPosition + travelDifference;

                // Set the target position
                spindexer.setTargetPosition(absoluteTarget);

                // Apply appropriate power based on mode
                double spindexerPower = intakeMode ? INTAKE_SPINDEXER_POWER : OUTTAKE_SPINDEXER_POWER;
                spindexer.setPower(spindexerPower);

                // Mark spindexer as moving - prevents new button presses until target reached
                spindexerIsMoving = true;
            }
            spindexerPressLast = spPress;

            // --- TELEMETRY ---
            int[] currentPositions = getPositionArray();
            int normalizedTarget = currentPositions[spindexerPosition];
            int currentPos = spindexer.getCurrentPosition();
            int normalizedCurrentPos = currentPos % (int) TICKS_PER_REVOLUTION;
            if (normalizedCurrentPos < 0)
                normalizedCurrentPos += TICKS_PER_REVOLUTION;

            telemetry.addData("Drive Speed", driveSpeed);
            telemetry.addData("Spindexer Mode", intakeMode ? "INTAKE" : "OUTTAKE");
            telemetry.addData("Spindexer Pos Index", spindexerPosition);
            telemetry.addData("Position Set", intakeMode ? "INTAKE (0, 716, 1433)" : "OUTTAKE (2060, 1885, 1716)");
            telemetry.addData("Normalized Target (0-2150)", normalizedTarget);
            telemetry.addData("**Absolute Target (Cumulative)**", absoluteTarget);
            telemetry.addData("Spindexer Encoder (Raw)", currentPos);
            telemetry.addData("Spindexer Encoder (Norm)", normalizedCurrentPos);
            telemetry.addData("Distance to Target", absoluteTarget - currentPos);
            telemetry.addData("Motor Is Busy", spindexer.isBusy());
            telemetry.addData("Spindexer Status", spindexerIsMoving ? "MOVING - Wait for position..."
                    : (ballSettling ? "SETTLING - Ball entry..."
                            : (shooterNeedsToSpinUp ? "WAITING - Shooter spinning up..." : "READY")));
            telemetry.addData("At Target Position", isSpindexerAtTarget());
            telemetry.addData("Shooter RPM", "%.0f / %.0f", shooter.getCurrentRPM(), shooter.getTargetRPM());
            telemetry.addData("Shooter At RPM", shooter.isAtTargetRPM());
            telemetry.addData("Can Move Spindexer", canMoveSpindexer);
            telemetry.addData("Motor Direction", "REVERSE (change if encoder counts wrong)");
            telemetry.update();
        }
    }

    /**
     * Check if spindexer is at target position (within tolerance).
     * Similar to ShooterSubsystem.isAtTargetRPM() but for position control.
     * 
     * @return True if spindexer is within tolerance of target position
     */
    private boolean isSpindexerAtTarget() {
        int currentPos = spindexer.getCurrentPosition();
        int distanceToTarget = Math.abs(absoluteTarget - currentPos);
        return distanceToTarget <= POSITION_TOLERANCE;
    }
}
