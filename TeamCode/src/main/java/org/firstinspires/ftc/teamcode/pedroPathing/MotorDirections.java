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

    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;
    private List<DcMotorEx> motors;

    // 🛠️ Replace these with your actual configuration names from the Driver Hub
    private static final String LEFT_FRONT_NAME = "leftFront";
    private static final String LEFT_REAR_NAME = "leftRear";
    private static final String RIGHT_FRONT_NAME = "rightFront";
    private static final String RIGHT_REAR_NAME = "rightRear";

    // 🧭 Set motor directions here (change REVERSE/FORWARD if motors spin the wrong way)
    private static final DcMotor.Direction LEFT_FRONT_DIR = DcMotor.Direction.FORWARD;
    private static final DcMotor.Direction LEFT_REAR_DIR = DcMotor.Direction.FORWARD;
    private static final DcMotor.Direction RIGHT_FRONT_DIR = DcMotor.Direction.REVERSE;
    private static final DcMotor.Direction RIGHT_REAR_DIR = DcMotor.Direction.REVERSE;

    @Override
    public void init() {
        // Initialize motors
        leftFront = hardwareMap.get(DcMotorEx.class, LEFT_FRONT_NAME);
        leftRear = hardwareMap.get(DcMotorEx.class, LEFT_REAR_NAME);
        rightFront = hardwareMap.get(DcMotorEx.class, RIGHT_FRONT_NAME);
        rightRear = hardwareMap.get(DcMotorEx.class, RIGHT_REAR_NAME);

        setMotorDirections();

        motors = Arrays.asList(leftFront, leftRear, rightFront, rightRear);
        configureMotors();

        telemetry.addLine("Press A/Y/B/X to spin individual motors. Adjust direction constants if needed.");
        telemetry.update();
    }

    @Override
    public void loop() {
        setMotorDirections();
        controlMotorPower();
        updateTelemetry();
    }

    private void setMotorDirections() {
        leftFront.setDirection(LEFT_FRONT_DIR);
        leftRear.setDirection(LEFT_REAR_DIR);
        rightFront.setDirection(RIGHT_FRONT_DIR);
        rightRear.setDirection(RIGHT_REAR_DIR);
    }

    private void configureMotors() {
        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }

    private void controlMotorPower() {
        leftFront.setPower(gamepad1.a ? 1 : 0);
        leftRear.setPower(gamepad1.y ? 1 : 0);
        rightFront.setPower(gamepad1.b ? 1 : 0);
        rightRear.setPower(gamepad1.x ? 1 : 0);
    }

    private void updateTelemetry() {
        telemetry.addLine("A → Left Front | Y → Left Rear | B → Right Front | X → Right Rear");
        telemetry.addData("Left Front Direction", LEFT_FRONT_DIR);
        telemetry.addData("Left Rear Direction", LEFT_REAR_DIR);
        telemetry.addData("Right Front Direction", RIGHT_FRONT_DIR);
        telemetry.addData("Right Rear Direction", RIGHT_REAR_DIR);
        telemetry.update();
    }
}
