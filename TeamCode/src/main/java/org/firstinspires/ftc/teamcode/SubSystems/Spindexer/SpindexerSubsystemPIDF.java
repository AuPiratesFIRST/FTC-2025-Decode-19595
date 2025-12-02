package org.firstinspires.ftc.teamcode.SubSystems.Spindexer;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, PIDF);

        motor.setDirection(DcMotor.Direction.REVERSE);

        absoluteTarget = 0;
        motor.setTargetPosition(absoluteTarget);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Initialize the initial position to current motor position
        initialPosition = motor.getCurrentPosition();
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
        boolean atTarget = Math.abs(currPos - initialPosition) >= Math.abs(tar);
        telemetry.addData("AtTarget", atTarget);
        telemetry.addData("CurrentPos", currPos);
        telemetry.addData("InitPos", initialPosition);
        telemetry.addData("InitPosAbs", Math.abs(initialPosition));
        telemetry.addData("tar", tar);
        telemetry.addData("DistanceMoved", Math.abs(currPos - initialPosition));
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
        // MOVEMENT COMPLETE
        // -------------------------------
        if (moving && atTarget(absoluteTarget)) {
            moving = false;
            motor.setPower(0);

            if (intakeMode) {
                // BALL SETTLING
                settlingTarget = motor.getCurrentPosition() - BALL_SETTLE_TICKS;
                motor.setTargetPosition(settlingTarget);
                motor.setPower(0.4);
                ballSettling = true;
            } else {
                shooter.setPower(SHOOT_POWER);
                shooterSpinUpPending = true;
            }
        }

        // -------------------------------
        // BALL SETTLING DONE
        // -------------------------------
        if (ballSettling && atTarget(settlingTarget)) {
            ballSettling = false;
            motor.setPower(0);
        }

        // -------------------------------
        // SHOOTER READY
        // -------------------------------
        if (shooterSpinUpPending && shooter.isAtTargetRPM()) {
            shooterSpinUpPending = false;
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
