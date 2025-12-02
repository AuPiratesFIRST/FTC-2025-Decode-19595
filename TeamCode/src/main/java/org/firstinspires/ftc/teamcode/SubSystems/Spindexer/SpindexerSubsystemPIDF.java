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

    private static final int[] INTAKE_POSITIONS = { 0, POS_120, POS_240 };

    // OUTTAKE measured & normalized positions
    private static final int[] OUTTAKE_POSITIONS = {
            normalize(-90),
            normalize(-265),
            normalize(-434)
    };

    private static int normalize(int raw) {
        int v = raw % (int)TICKS_PER_REV;
        return (v < 0 ? v + (int)TICKS_PER_REV : v);
    }

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

    public SpindexerSubsystemPIDF(HardwareMap hw, Telemetry telemetry, ShooterSubsystem shooter) {
        this.telemetry = telemetry;
        this.shooter = shooter;

        motor = hw.get(DcMotorEx.class, "Spindexer");

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, PIDF);

        motor.setDirection(DcMotor.Direction.REVERSE);

        absoluteTarget = 0;
        motor.setTargetPosition(absoluteTarget);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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

    // --------------------------------------------------------------------------
    // GET CURRENT TARGET ARRAY
    // --------------------------------------------------------------------------
    private int[] getArray() {
        return intakeMode ? INTAKE_POSITIONS : OUTTAKE_POSITIONS;
    }

    // --------------------------------------------------------------------------
    // CHECK TARGET REACHED
    // --------------------------------------------------------------------------
    private boolean atTarget(int t) {
        return Math.abs(motor.getCurrentPosition() - t) <= POSITION_TOLERANCE;
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
        int nextNorm = arr[index];

        int raw = motor.getCurrentPosition();
        int norm = raw % (int)TICKS_PER_REV;
        if (norm < 0) norm += (int)TICKS_PER_REV;

        int diff = nextNorm - norm;
        if (diff < 0)
            diff += (int)TICKS_PER_REV;

        absoluteTarget = raw + diff;
        motor.setTargetPosition(absoluteTarget);

        motor.setPower(intakeMode ? 0.4 : 0.8);
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
        telemetry.addData("AbsTarget", absoluteTarget);
        telemetry.addData("Busy", motor.isBusy());
        telemetry.addData("Moving", moving);
        telemetry.addData("Settling", ballSettling);
        telemetry.addData("ShooterPending", shooterSpinUpPending);
    }
}
