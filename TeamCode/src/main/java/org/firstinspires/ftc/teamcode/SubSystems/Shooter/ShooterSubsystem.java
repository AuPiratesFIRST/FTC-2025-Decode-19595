package org.firstinspires.ftc.teamcode.SubSystems.Shooter;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Shooter subsystem for GoBILDA 6000 RPM motors with 3-inch wheels.
 * Now includes **battery voltage compensation** to maintain stable RPM.
 */
public class ShooterSubsystem {

    private final DcMotorEx leftShooterMotor;
    private final DcMotorEx rightShooterMotor;
    private final Telemetry telemetry;

    // === NEW: Battery Voltage ===
    private final VoltageSensor batteryVoltageSensor;
    private static final double NOMINAL_VOLTAGE = 13.0;   // Fully charged 3-cell
    private double voltageCorrectionFactor = 1.0;
    // ============================

    // Motor specifications
    private static final double MOTOR_MAX_RPM = 6000.0;
    private static final double TICKS_PER_REVOLUTION = 28.0;
    private static final double WHEEL_DIAMETER = 3.0;
    private static final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;

    // Velocity control
    private double targetRPM = 0.0;
    private double rpmTolerance = 100.0;
    private boolean useVelocityControl = true;
    private boolean usePercentageTolerance = true;

    // Power limits
    private double minPower = 0.3;
    private double maxPower = 1.0;
    private double defaultPower = 0.7;

    public enum ShooterSpeed {
        LOW(0.5),
        MEDIUM(0.7),
        HIGH(0.9),
        MAX(1.0);

        private final double power;
        ShooterSpeed(double power) {
            this.power = power;
        }
        public double getPower() { return power; }
    }

    public ShooterSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        leftShooterMotor = hardwareMap.get(DcMotorEx.class, "shooterL");
        rightShooterMotor = hardwareMap.get(DcMotorEx.class, "shooterR");


        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();


        configureMotors();
    }

    private void configureMotors() {
        leftShooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightShooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        rightShooterMotor.setDirection(DcMotor.Direction.REVERSE);

        PIDFCoefficients velocityPIDF = new PIDFCoefficients(35.0, 0.15, 12.0, 15.0);
        leftShooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, velocityPIDF);
        rightShooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, velocityPIDF);
    }



    // VOLTAGE COMPENSATION 


    /** Update the voltage correction factor each loop */
    public void updateVoltageCompensation() {
        double voltage = batteryVoltageSensor.getVoltage();
        voltageCorrectionFactor = NOMINAL_VOLTAGE / voltage;

        if (telemetry != null) {
            telemetry.addData("Battery Voltage", "%.2f V", voltage);
            telemetry.addData("Voltage Compensation", "%.2f×", voltageCorrectionFactor);
        }
    }

    /** Get raw battery voltage */
    public double getCurrentVoltage() {
        return batteryVoltageSensor.getVoltage();
    }





    /** Set power → automatically converted to RPM when velocity control is on */
    public void setPower(double power) {
        power = Range.clip(power, minPower, maxPower);

        if (useVelocityControl) {
            setTargetRPM(MOTOR_MAX_RPM * power);
        } else {
            leftShooterMotor.setPower(power);
            rightShooterMotor.setPower(power);
            this.targetRPM = MOTOR_MAX_RPM * power;
        }
    }




    // setTargetRPM NOW USES VOLTAGE COMPENSATION 
    
    public void setTargetRPM(double rpm) {
        targetRPM = Range.clip(rpm, 0.0, MOTOR_MAX_RPM);


        double compensatedRPM = targetRPM * voltageCorrectionFactor;

        double ticksPerSecond = (compensatedRPM * TICKS_PER_REVOLUTION) / 60.0;

        leftShooterMotor.setVelocity(ticksPerSecond);
        rightShooterMotor.setVelocity(ticksPerSecond);

        if (telemetry != null) {
            telemetry.addData("Target RPM", "%.0f", targetRPM);
            telemetry.addData("Compensated RPM", "%.0f", compensatedRPM);
        }
    }
    // 



    public void setSpeed(ShooterSpeed speed) { setPower(speed.getPower()); }

    public void start() { setPower(defaultPower); }

    public boolean isAtTargetRPM() {
        double currentRPM = getCurrentRPM();
        double error = Math.abs(currentRPM - targetRPM);

        if (usePercentageTolerance) {
            return error <= targetRPM * 0.02;
        } else {
            return error <= rpmTolerance;
        }
    }

    public double getRPMError() { return Math.abs(getCurrentRPM() - targetRPM); }

    public double getRPMErrorPercentage() {
        return targetRPM == 0 ? 100.0 : (getRPMError() / targetRPM) * 100.0;
    }

    public double getCurrentRPM() {
        double leftRPM = leftShooterMotor.getVelocity() * 60.0 / TICKS_PER_REVOLUTION;
        double rightRPM = rightShooterMotor.getVelocity() * 60.0 / TICKS_PER_REVOLUTION;
        return (leftRPM + rightRPM) / 2.0;
    }

    public void stop() {
        leftShooterMotor.setVelocity(0);
        rightShooterMotor.setVelocity(0);
        targetRPM = 0;
    }

    public void updateTelemetry() {
        if (telemetry == null) return;

        telemetry.addData("Shooter Status", isAtTargetRPM() ? "READY ✓" : "SPINNING...");
        telemetry.addData("Current RPM", "%.0f", getCurrentRPM());
        telemetry.addData("Target RPM", "%.0f", targetRPM);
        telemetry.addData("RPM Error", "%.0f", getRPMError());
        telemetry.addData("Error %", "%.1f%%", getRPMErrorPercentage());
        telemetry.addData("Voltage", "%.2f V", getCurrentVoltage());

        telemetry.update();
    }
}