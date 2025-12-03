package org.firstinspires.ftc.teamcode.SubSystems.Spindexer;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SubSystems.Shooter.ShooterSubsystem;

public class SpindexerSubsystemPIDF {

    private final DcMotorEx motor;
    private final Telemetry telemetry;
    private final ShooterSubsystem shooter;

    // --------------------------------------------------------------------------
    // ENCODER CONSTANTS
    // --------------------------------------------------------------------------
    private static final double TICKS_PER_REV = 2150.8;

    // 120° positions (INTAKE)
    private static final int POS_120 = (int)(TICKS_PER_REV / 3.0);
    private static final int POS_240 = (int)(TICKS_PER_REV * 2.0 / 3.0);

    private static final int[] INTAKE_POSITIONS = {
            (int)TICKS_PER_REV,
            POS_120,
            POS_240
    };

    // OUTTAKE measured positions
    private static final int[] OUTTAKE_POSITIONS = {
            90,
            265,
            434
    };

    // Ball settling offset (tune for your radius)
    private static final int BALL_SETTLE_TICKS = 34;

    private static final int POSITION_TOLERANCE = 15;

    // --------------------------------------------------------------------------
    // PIDF VALUES
    // --------------------------------------------------------------------------
    private static final PIDFCoefficients PIDF =
            new PIDFCoefficients(2.5, 0.1, 0.2, 0.5);

    // --------------------------------------------------------------------------
    // STATE
    // --------------------------------------------------------------------------
    private boolean intakeMode = true;
    private int index = 0;

    private int absoluteTarget = 0;
    private boolean moving = false;

    private boolean ballSettling = false;
    private int settlingTarget = 0;

    private boolean shooterSpinUpPending = false;
    private final double SHOOT_POWER = 0.7;

    // 1. Private field to store the initial position.
    private int initialPosition;

    public SpindexerSubsystemPIDF(HardwareMap hw, Telemetry telemetry, ShooterSubsystem shooter) {
        this.telemetry = telemetry;
        this.shooter = shooter;

        motor = hw.get(DcMotorEx.class, "Spindexer");

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set initial position and target before switching to RUN_TO_POSITION mode
        initialPosition = motor.getCurrentPosition();  // Capture the current position of the motor
        absoluteTarget = initialPosition;  // Set the initial target to current position

        // Set PID coefficients
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, PIDF);

        motor.setDirection(DcMotor.Direction.REVERSE);  // Ensure correct direction

        // Set the target position
        motor.setTargetPosition(0);

        // Now switch to RUN_TO_POSITION mode
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motor.setPower(0);  // Start motor with no power initially

        // Initialization is complete
        telemetry.addData("Spindexer Initialized", "Ready for operation.");
        telemetry.update();
    }

    // --------------------------------------------------------------------------
    // MODE CONTROL
    // --------------------------------------------------------------------------
    public void setIntakeMode(boolean mode) {
        if (intakeMode != mode) {
            intakeMode = mode;
            index = 0;
        }
    }

    public boolean isIntakeMode() {
        return intakeMode;
    }

    // --------------------------------------------------------------------------
    // GET CURRENT TARGET ARRAY
    // --------------------------------------------------------------------------
    private int[] getArray() {
        return intakeMode ? INTAKE_POSITIONS : OUTTAKE_POSITIONS;
    }

    // --------------------------------------------------------------------------
    // CHECK TARGET REACHED
    // --------------------------------------------------------------------------
    private boolean atTarget(int tar) {
        int currPos = motor.getCurrentPosition();
        int targetDistance = Math.abs(currPos - tar);
        boolean atTarget = targetDistance <= POSITION_TOLERANCE;  // Use a tolerance to account for slight errors

        telemetry.addData("AtTarget", atTarget);
        telemetry.addData("CurrentPos", currPos);
        telemetry.addData("TargetPos", tar);
        telemetry.addData("TargetDistance", targetDistance);
        telemetry.addData("PositionTolerance", POSITION_TOLERANCE);

        return atTarget;
    }

    // --------------------------------------------------------------------------
    // ADVANCE TO NEXT POSITION (CALLED FROM OP-MODE)
    // --------------------------------------------------------------------------
    public void advance() {
        if (moving || ballSettling || shooterSpinUpPending)
            return;

        // Advance index
        index = (index + 1) % 3;

        int[] arr = getArray();
        int delta = arr[index];  // Desired relative distance to move

        // Capture current position and calculate the absolute target
        initialPosition = motor.getCurrentPosition(); // Capture current position

        // Convert delta (relative) into an absolute encoder target
        absoluteTarget = initialPosition - delta; // or +delta if moving in the forward direction

        // Set the target position
        motor.setTargetPosition(absoluteTarget);

        // Ensure that the motor is in RUN_TO_POSITION mode and set power before moving
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set motor power for the move (0.4 for intake, 0.8 for outtake)
        motor.setPower(intakeMode ? 0.4 : 0.8);

        // Mark the movement as started
        moving = true;
    }

    // --------------------------------------------------------------------------
    // UPDATE LOOP (CALL EVERY CYCLE)
    // --------------------------------------------------------------------------
    public void update() {
        // -------------------------------
        // MOVEMENT COMPLETE (Target Reached)
        // -------------------------------
        if (moving && atTarget(absoluteTarget)) {
            moving = false;  // Stop the movement flag
            motor.setPower(0);  // Stop the motor once the target is reached

            if (intakeMode) {
                // Ball settling small correction after intake
                settlingTarget = motor.getCurrentPosition() - BALL_SETTLE_TICKS;  // Adjust the target slightly
                motor.setTargetPosition(settlingTarget);
                motor.setPower(0.4);  // Low power for settling
                ballSettling = true;  // Ball settling is active
            } else {
                // Start shooter spin-up if in outtake mode
                shooter.setPower(SHOOT_POWER);
                shooterSpinUpPending = true;
            }
        }

        // -------------------------------
        // BALL SETTLING DONE (After Intake)
        // -------------------------------
        if (ballSettling && atTarget(settlingTarget)) {
            ballSettling = false;  // Stop settling
            motor.setPower(0);  // Ensure motor is stopped
        }

        // -------------------------------
        // SHOOTER READY
        // -------------------------------
        if (shooterSpinUpPending && shooter.isAtTargetRPM()) {
            shooterSpinUpPending = false;  // Shooter is ready, stop pending flag
        }

        // -------------------------------
        // TELEMETRY
        // -------------------------------
        telemetry.addData("Mode", intakeMode ? "INTAKE" : "OUTTAKE");
        telemetry.addData("Index", index);
        telemetry.addData("RawPos", motor.getCurrentPosition());
        int[] arr = getArray();
        if (index < arr.length) {
            telemetry.addData("TargetPos", arr[index]);
        }
        telemetry.addData("AbsTarget", absoluteTarget);
        telemetry.addData("Busy", motor.isBusy());
        telemetry.addData("Moving", moving);
        telemetry.addData("Settling", ballSettling);
        telemetry.addData("ShooterPending", shooterSpinUpPending);
    }
}
