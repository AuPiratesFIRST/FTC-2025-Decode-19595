package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;  // Import Constants class


@TeleOp(name = "Motor Directions", group = "TeleOp")
public class MotorDirections extends OpMode {


    // Motor variables
    private DcMotor leftFrontMotor;
    private DcMotor leftRearMotor;
    private DcMotor rightFrontMotor;
    private DcMotor rightRearMotor;

    @Override
    public void init() {
        // Initialize motors using names from Constants
        leftFrontMotor = hardwareMap.get(DcMotor.class, Constants.driveConstants.leftFrontMotorName);
        leftRearMotor = hardwareMap.get(DcMotor.class, Constants.driveConstants.leftRearMotorName);
        rightFrontMotor = hardwareMap.get(DcMotor.class, Constants.driveConstants.rightFrontMotorName);
        rightRearMotor = hardwareMap.get(DcMotor.class, Constants.driveConstants.rightRearMotorName);

        // Set motor directions from Constants
        leftFrontMotor.setDirection(Constants.driveConstants.leftFrontMotorDirection);
        leftRearMotor.setDirection(Constants.driveConstants.leftRearMotorDirection);
        rightFrontMotor.setDirection(Constants.driveConstants.rightFrontMotorDirection);
        rightRearMotor.setDirection(Constants.driveConstants.rightRearMotorDirection);

        // Reset encoder counts before starting
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Use encoders to track motor movement (no power needed)
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        // Get the current encoder values for each motor
        int leftFrontEncoder = leftFrontMotor.getCurrentPosition();
        int leftRearEncoder = leftRearMotor.getCurrentPosition();
        int rightFrontEncoder = rightFrontMotor.getCurrentPosition();
        int rightRearEncoder = rightRearMotor.getCurrentPosition();

        // Display encoder values in telemetry
        telemetry.addData("Left Front Encoder", leftFrontEncoder);
        telemetry.addData("Left Rear Encoder", leftRearEncoder);
        telemetry.addData("Right Front Encoder", rightFrontEncoder);
        telemetry.addData("Right Rear Encoder", rightRearEncoder);

        // Update the telemetry on the driver station
        telemetry.update();
    }

    @Override
    public void stop() {
        // Stop all motors when the op mode is disabled or stopped
        leftFrontMotor.setPower(0);
        leftRearMotor.setPower(0);
        rightFrontMotor.setPower(0);
        rightRearMotor.setPower(0);
    }
}
