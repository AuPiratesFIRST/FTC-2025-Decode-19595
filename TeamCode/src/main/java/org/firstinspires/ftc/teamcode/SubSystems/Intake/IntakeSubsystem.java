package org.firstinspires.ftc.teamcode.SubSystems.Intake;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Intake subsystem using GoBILDA 312 RPM motor.
 * Provides forward and reverse intake control.
 */
public class IntakeSubsystem {

    private final DcMotorEx intakeMotor;
    private final Telemetry telemetry;

    // Motor specifications
    private static final double MOTOR_MAX_RPM = 312.0; // GoBILDA 312 RPM motor

    // Default power settings
    private double intakePower = 0.8; // Forward intake power
    private double outtakePower = -0.8; // Reverse/outtake power (default)

    public enum IntakeDirection {
        FORWARD(1),
        REVERSE(-1),
        STOP(0);

        private final int direction;

        IntakeDirection(int direction) {
            this.direction = direction;
        }

        public int getDirection() {
            return direction;
        }
    }

    public IntakeSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        // Initialize motor
        intakeMotor = hardwareMap.get(DcMotorEx.class, "Intake");

        configureMotor();
    }

    private void configureMotor() {
        // Set motor mode
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setDirection(DcMotorEx.Direction.REVERSE);
        // Set zero power behavior
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    /**
     * Start intake in forward direction
     */
    public void start() {
        intakeMotor.setPower(intakePower);

        if (telemetry != null) {
            telemetry.addData("Intake", "Running Forward");
        }
    }

    /**
     * Start intake in reverse direction (outtake)
     */
    public void reverse() {
        intakeMotor.setPower(outtakePower);

        if (telemetry != null) {
            telemetry.addData("Intake", "Running Reverse");
        }
    }

    /**
     * Stop intake
     */
    public void stop() {
        intakeMotor.setPower(0);

        if (telemetry != null) {
            telemetry.addData("Intake", "Stopped");
        }
    }

    /**
     * Set intake direction
     * 
     * @param direction IntakeDirection enum
     */
    public void setDirection(IntakeDirection direction) {
        switch (direction) {
            case FORWARD:
                start();
                break;
            case REVERSE:
                reverse();
                break;
            case STOP:
                stop();
                break;
        }
    }

    /**
     * Set intake power directly
     * 
     * @param power Motor power (-1.0 to 1.0, positive = forward, negative =
     *              reverse)
     */
    public void setPower(double power) {
        power = Range.clip(power, -1.0, 1.0);
        intakeMotor.setPower(power);

        if (telemetry != null) {
            telemetry.addData("Intake Power", "%.2f", power);
        }
    }

    /**
     * Set forward intake power
     * 
     * @param power Power value (0.0 to 1.0)
     */
    public void setIntakePower(double power) {
        this.intakePower = Range.clip(power, 0.0, 1.0);
    }

    /**
     * Set reverse/outtake power
     * 
     * @param power Power value (-1.0 to 0.0)
     */
    public void setOuttakePower(double power) {
        this.outtakePower = Range.clip(power, -1.0, 0.0);
    }

    /**
     * Get current motor power
     * 
     * @return Current power setting
     */
    public double getCurrentPower() {
        return intakeMotor.getPower();
    }

    /**
     * Get current RPM
     * 
     * @return Current RPM
     */
    public double getCurrentRPM() {
        return intakeMotor.getVelocity();
    }

    /**
     * Check if intake is running
     * 
     * @return True if motor power is not zero
     */
    public boolean isRunning() {
        return Math.abs(intakeMotor.getPower()) > 0.01;
    }

    /**
     * Update telemetry with intake information
     */
    public void updateTelemetry() {
        if (telemetry != null) {
            telemetry.addData("Intake Status", isRunning() ? "Active" : "Stopped");
            telemetry.addData("Intake Power", "%.2f", getCurrentPower());
            telemetry.addData("Intake RPM", "%.0f", getCurrentRPM());
            telemetry.addData("Intake Direction",
                    getCurrentPower() > 0 ? "Forward" : getCurrentPower() < 0 ? "Reverse" : "Stopped");
        }
    }
}

