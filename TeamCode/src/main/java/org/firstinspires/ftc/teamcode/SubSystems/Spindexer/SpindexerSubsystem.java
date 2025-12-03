package org.firstinspires.ftc.teamcode.SubSystems.Spindexer;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SubSystems.Shooter.ShooterSubsystem;

public class SpindexerSubsystem {

    private final DcMotorEx motor;
    private final ShooterSubsystem shooter;
    private final Telemetry telemetry;

    // ---------------- MOTOR CONSTANTS ----------------
    private static final double CPR = 537.7; // 19.2:1 Yellow Jacket
    private static final double TICKS_60  = CPR / 6.0;   // ≈ 89.61
    private static final double TICKS_120 = CPR / 3.0;   // ≈ 179.23


    private static final double INTAKE_POWER  = 0.45;
    private static final double OUTTAKE_POWER = 0.70;

    private static final int BALL_SETTLE_TICKS = 25;
    private static final int TARGET_TOLERANCE = 10;

    // ---------------- PIDF (RUN_TO_POSITION) ----------------
    private static final PIDFCoefficients PIDF =
            new PIDFCoefficients(8, 0, 0.6, 0);

    // ---------------- STATE ----------------
    private double accum = 0;      // cumulative target angle (ticks)
    private int zero = 0;          // reference position
    private int target = 0;

    private boolean intakeMode = true;
    private int index = 0;

    private boolean moving = false;
    private boolean settling = false;
    private int settlingTarget = 0;

    private boolean shooterPending = false;
    private final double SHOOT_POWER = 0.7;

    // ---------------- OUTTAKE POSITIONS (CUSTOMIZABLE) ----------------
    // Measure these with a REV Hub by rotating by hand
    private static final int[] OUTTAKE_POSITIONS = {
            30,   // Slot 0 eject position
            210,  // Slot 1 eject position
            390   // Slot 2 eject position
    };

    public SpindexerSubsystem(HardwareMap hw, Telemetry tel, ShooterSubsystem shooter) {
        this.telemetry = tel;
        this.shooter = shooter;

        motor = hw.get(DcMotorEx.class, "Spindexer");

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setTargetPositionTolerance(TARGET_TOLERANCE);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Apply correct PID
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, PIDF);

        zero = motor.getCurrentPosition();
        accum = 0;
        target = zero;
    }

    // -------------------------------------------------------
    // MODE CONTROL
    // -------------------------------------------------------

    public void setIntakeMode(boolean isIntake) {
        if (this.intakeMode != isIntake) {
            this.intakeMode = isIntake;
            this.index = 0;
        }
    }

    public boolean isIntakeMode() {
        return intakeMode;
    }

    // -------------------------------------------------------
    // ADVANCE (CALLED FROM DRIVER)
    // -------------------------------------------------------

    public void advance() {
        if (moving || settling || shooterPending) return;

        index = (index + 1) % 3;

        if (intakeMode) {
            accum += TICKS_120;  // 120° rotation
            target = (int)Math.rint(zero + accum);

            motor.setTargetPosition(target);
            motor.setPower(INTAKE_POWER);
        } else {
            target = zero + OUTTAKE_POSITIONS[index];
            motor.setTargetPosition(target);
            motor.setPower(OUTTAKE_POWER);
        }

        moving = true;
    }

    // -------------------------------------------------------
    // UPDATE LOOP
    // -------------------------------------------------------

    public void update() {

        // ---- FINISHED MOVING ----
        if (moving && !motor.isBusy()) {
            moving = false;
            motor.setPower(0);

            if (intakeMode) {
                // Ball settling small correction
                settlingTarget = motor.getCurrentPosition() - BALL_SETTLE_TICKS;
                motor.setTargetPosition(settlingTarget);
                motor.setPower(INTAKE_POWER);
                settling = true;

            } else {
                // Start shooter spin-up
                shooter.setPower(SHOOT_POWER);
                shooterPending = true;
            }
        }

        // ---- SETTLING COMPLETE ----
        if (settling && !motor.isBusy()) {
            settling = false;
            motor.setPower(0);
        }

        // ---- SHOOTER READY ----
        if (shooterPending && shooter.isAtTargetRPM()) {
            shooterPending = false;
        }

        // ---- TELEMETRY ----
        telemetry.addData("Mode", intakeMode ? "INTAKE" : "OUTTAKE");
        telemetry.addData("Index", index);
        telemetry.addData("Encoder", motor.getCurrentPosition());
        telemetry.addData("Target", target);
        telemetry.addData("Busy", motor.isBusy());
        telemetry.addData("Moving", moving);
        telemetry.addData("Settling", settling);
        telemetry.addData("ShooterPending", shooterPending);
    }
}
