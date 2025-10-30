package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import java.util.Arrays;
import java.util.List;

@TeleOp(name = "Motor Directions", group = "Teleop Test")
public class MotorDirections extends OpMode {

    // Declare motor objects
    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;

    // List to hold all motors for easier management
    private List<DcMotorEx> motors;

    // Motor names from Driver Hub configuration
    private static final String LEFT_FRONT_NAME = "leftFront";
    private static final String LEFT_REAR_NAME = "leftRear";
    private static final String RIGHT_FRONT_NAME = "rightFront";
    private static final String RIGHT_REAR_NAME = "rightRear";

    // Motor directions configuration (adjust if motors spin the wrong way)
    private static final DcMotor.Direction LEFT_FRONT_DIR = DcMotor.Direction.REVERSE;
    private static final DcMotor.Direction LEFT_REAR_DIR = DcMotor.Direction.REVERSE;
    private static final DcMotor.Direction RIGHT_FRONT_DIR = DcMotor.Direction.FORWARD;
    private static final DcMotor.Direction RIGHT_REAR_DIR = DcMotor.Direction.FORWARD;

    @Override
    public void init() {
        // Initialize motors using hardware map
        leftFront = hardwareMap.get(DcMotorEx.class, LEFT_FRONT_NAME);
        leftRear = hardwareMap.get(DcMotorEx.class, LEFT_REAR_NAME);
        rightFront = hardwareMap.get(DcMotorEx.class, RIGHT_FRONT_NAME);
        rightRear = hardwareMap.get(DcMotorEx.class, RIGHT_REAR_NAME);

        // Set motor directions
        setMotorDirections();

        // Add motors to a list for easy iteration
        motors = Arrays.asList(leftFront, leftRear, rightFront, rightRear);

        // Configure motors with max RPM, float behavior, and other settings
        configureMotors();

        // Telemetry to guide the user
        telemetry.addLine("Press A/Y/B/X to spin individual motors. Adjust direction constants if needed.");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Set motor directions every loop to ensure any changes are applied
        setMotorDirections();

        // Control motor power based on gamepad buttons (A/Y/B/X) or triggers (left/right)
        controlMotorPower();

        // Update telemetry with motor status, directions, encoder values, and velocities
        updateTelemetry();
    }

    // Set motor directions as per configuration
    private void setMotorDirections() {
        leftFront.setDirection(LEFT_FRONT_DIR);
        leftRear.setDirection(LEFT_REAR_DIR);
        rightFront.setDirection(RIGHT_FRONT_DIR);
        rightRear.setDirection(RIGHT_REAR_DIR);
    }

    // Configure motor settings, including max RPM and zero power behavior
    private void configureMotors() {
        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0); // Full RPM potential
            motor.setMotorType(motorConfigurationType);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // Float when not powered
        }
    }

    // Control the motor power based on gamepad button presses or trigger presses
    private void controlMotorPower() {
        // Control motors individually with gamepad buttons
        leftFront.setPower(gamepad1.a ? 1 : 0);  // A button: left front motor
        leftRear.setPower(gamepad1.y ? 1 : 0);   // Y button: left rear motor
        rightFront.setPower(gamepad1.b ? 1 : 0); // B button: right front motor
        rightRear.setPower(gamepad1.x ? 1 : 0);  // X button: right rear motor

        // If the right trigger is pressed, run all motors at full power
        if (gamepad1.right_trigger > 0.1) {  // Adjust the threshold if needed
            setAllMotorsPower(1);  // Full power to all motors
        }
        // If the left trigger is pressed, run all motors at half power
        else if (gamepad1.left_trigger > 0.1) {  // Left trigger functionality
            setAllMotorsPower(0.5);  // Half power to all motors
        }
    }

    // Helper method to set power for all motors
    private void setAllMotorsPower(double power) {
        leftFront.setPower(power);
        leftRear.setPower(power);
        rightFront.setPower(power);
        rightRear.setPower(power);
    }

    // Update telemetry with motor info: direction, encoder positions, and velocities
    private void updateTelemetry() {
        telemetry.addLine("A → Left Front | Y → Left Rear | B → Right Front | X → Right Rear");

        // Display motor directions
        telemetry.addData("Left Front Direction", LEFT_FRONT_DIR);
        telemetry.addData("Left Rear Direction", LEFT_REAR_DIR);
        telemetry.addData("Right Front Direction", RIGHT_FRONT_DIR);
        telemetry.addData("Right Rear Direction", RIGHT_REAR_DIR);

        // Display encoder positions (counts)
        telemetry.addData("Left Front Encoder", getEncoderValue(leftFront, LEFT_FRONT_DIR));
        telemetry.addData("Left Rear Encoder", getEncoderValue(leftRear, LEFT_REAR_DIR));
        telemetry.addData("Right Front Encoder", getEncoderValue(rightFront, RIGHT_FRONT_DIR));
        telemetry.addData("Right Rear Encoder", getEncoderValue(rightRear, RIGHT_REAR_DIR));

        // Display motor velocities (in counts per second)
        telemetry.addData("Left Front Velocity", leftFront.getVelocity());
        telemetry.addData("Left Rear Velocity", leftRear.getVelocity());
        telemetry.addData("Right Front Velocity", rightFront.getVelocity());
        telemetry.addData("Right Rear Velocity", rightRear.getVelocity());

        telemetry.update();
    }

    // Helper method to account for encoder direction based on motor direction
    private int getEncoderValue(DcMotorEx motor, DcMotor.Direction motorDir) {
        int encoderValue = motor.getCurrentPosition();

        // Reverse encoder value if motor is reversed
        if (motorDir == DcMotor.Direction.REVERSE) {
            return -encoderValue;
        }

        return encoderValue;
    }
}
