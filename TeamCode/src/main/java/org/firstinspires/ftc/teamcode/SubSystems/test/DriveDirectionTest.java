package org.firstinspires.ftc.teamcode.SubSystems.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Drive Direction and Encoder Test OpMode
 * 
 * This test helps verify:
 * 1. Motor directions are correct
 * 2. Encoder directions are correct
 * 3. Forward/strafe/turn movements work as expected
 * 
 * Controls:
 * - Individual Motor Test (Gamepad 1):
 *   - D-Pad Up = Left Front
 *   - D-Pad Right = Right Front
 *   - D-Pad Down = Left Rear
 *   - D-Pad Left = Right Rear
 *   - Hold button to run motor forward, release to stop
 * 
 * - Movement Test (Gamepad 1):
 *   - Left Stick Forward/Back = Forward/Backward movement
 *   - Left Stick Left/Right = Strafe Left/Right
 *   - Right Stick Left/Right = Turn Left/Right
 * 
 * - Reset Encoders (Gamepad 1):
 *   - Y = Reset all encoders to 0
 * 
 * - Mode Toggle (Gamepad 1):
 *   - X = Toggle between Individual Motor Test and Movement Test
 */
@TeleOp(name = "Drive Direction Test", group = "Test")
public class DriveDirectionTest extends LinearOpMode {

    private DcMotorEx leftFront, leftRear, rightFront, rightRear;
    
    private boolean individualMotorMode = true;
    private boolean lastX = false;
    private boolean lastY = false;
    
    // Encoder values at start (for relative tracking)
    private int lfStartPos, lrStartPos, rfStartPos, rrStartPos;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize motors
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");

        // Configure motors
        configureMotors();

        // Reset encoders
        resetEncoders();
        
        // Store starting positions
        lfStartPos = leftFront.getCurrentPosition();
        lrStartPos = leftRear.getCurrentPosition();
        rfStartPos = rightFront.getCurrentPosition();
        rrStartPos = rightRear.getCurrentPosition();

        telemetry.addLine("=== DRIVE DIRECTION TEST ===");
        telemetry.addLine("Mode: " + (individualMotorMode ? "INDIVIDUAL MOTOR" : "MOVEMENT"));
        telemetry.addLine("");
        telemetry.addLine("Individual Motor Test:");
        telemetry.addLine("  D-Pad Up = Left Front");
        telemetry.addLine("  D-Pad Right = Right Front");
        telemetry.addLine("  D-Pad Down = Left Rear");
        telemetry.addLine("  D-Pad Left = Right Rear");
        telemetry.addLine("");
        telemetry.addLine("Movement Test:");
        telemetry.addLine("  Left Stick = Forward/Strafe");
        telemetry.addLine("  Right Stick = Turn");
        telemetry.addLine("");
        telemetry.addLine("Controls:");
        telemetry.addLine("  X = Toggle Mode");
        telemetry.addLine("  Y = Reset Encoders");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Toggle mode
            boolean x = gamepad1.x;
            if (x && !lastX) {
                individualMotorMode = !individualMotorMode;
                stopAllMotors();
            }
            lastX = x;

            // Reset encoders
            boolean y = gamepad1.y;
            if (y && !lastY) {
                resetEncoders();
                lfStartPos = leftFront.getCurrentPosition();
                lrStartPos = leftRear.getCurrentPosition();
                rfStartPos = rightFront.getCurrentPosition();
                rrStartPos = rightRear.getCurrentPosition();
            }
            lastY = y;

            if (individualMotorMode) {
                handleIndividualMotorTest();
            } else {
                handleMovementTest();
            }

            updateTelemetry();
            sleep(20);
        }

        stopAllMotors();
    }

    private void configureMotors() {
        // Set to RUN_USING_ENCODER to read encoder values
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set zero power behavior
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set motor directions from Pedro Constants (for consistency)
        // This ensures the test uses the same directions as Pedro Pathing
        leftFront.setDirection(Constants.driveConstants.leftFrontMotorDirection);
        leftRear.setDirection(Constants.driveConstants.leftRearMotorDirection);
        rightFront.setDirection(Constants.driveConstants.rightFrontMotorDirection);
        rightRear.setDirection(Constants.driveConstants.rightRearMotorDirection);
    }

    private void resetEncoders() {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void handleIndividualMotorTest() {
        double power = 0.3; // Low power for testing

        // Test each motor individually
        if (gamepad1.dpad_up) {
            leftFront.setPower(power);
        } else {
            leftFront.setPower(0);
        }

        if (gamepad1.dpad_right) {
            rightFront.setPower(power);
        } else {
            rightFront.setPower(0);
        }

        if (gamepad1.dpad_down) {
            leftRear.setPower(power);
        } else {
            leftRear.setPower(0);
        }

        if (gamepad1.dpad_left) {
            rightRear.setPower(power);
        } else {
            rightRear.setPower(0);
        }
    }

    private void handleMovementTest() {
        // Get stick inputs (matching your teleop)
        float forward = -gamepad1.left_stick_y;
        float strafe = -gamepad1.left_stick_x;
        float turn = -gamepad1.right_stick_x;

        // Mecanum drive kinematics (matching your DriveSubsystem)
        double fl = forward + strafe + turn;
        double fr = forward - strafe - turn;
        double bl = forward - strafe + turn;
        double br = forward + strafe - turn;

        // Clip to [-1, 1]
        double max = Math.max(Math.max(Math.abs(fl), Math.abs(fr)), 
                             Math.max(Math.abs(bl), Math.abs(br)));
        if (max > 1.0) {
            fl /= max;
            fr /= max;
            bl /= max;
            br /= max;
        }

        leftFront.setPower(fl);
        rightFront.setPower(fr);
        leftRear.setPower(bl);
        rightRear.setPower(br);
    }

    private void stopAllMotors() {
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
    }

    private void updateTelemetry() {
        // Get current encoder positions
        int lfPos = leftFront.getCurrentPosition();
        int lrPos = leftRear.getCurrentPosition();
        int rfPos = rightFront.getCurrentPosition();
        int rrPos = rightRear.getCurrentPosition();

        // Calculate relative positions (from start)
        int lfRel = lfPos - lfStartPos;
        int lrRel = lrPos - lrStartPos;
        int rfRel = rfPos - rfStartPos;
        int rrRel = rrPos - rrStartPos;

        telemetry.addLine("=== DRIVE DIRECTION TEST ===");
        telemetry.addData("Mode", individualMotorMode ? "INDIVIDUAL MOTOR" : "MOVEMENT");
        telemetry.addLine("");
        
        telemetry.addLine("=== ENCODER VALUES ===");
        telemetry.addData("Left Front", "Absolute: %d | Relative: %d | Power: %.2f", 
                         lfPos, lfRel, leftFront.getPower());
        telemetry.addData("Right Front", "Absolute: %d | Relative: %d | Power: %.2f", 
                         rfPos, rfRel, rightFront.getPower());
        telemetry.addData("Left Rear", "Absolute: %d | Relative: %d | Power: %.2f", 
                         lrPos, lrRel, leftRear.getPower());
        telemetry.addData("Right Rear", "Absolute: %d | Relative: %d | Power: %.2f", 
                         rrPos, rrRel, rightRear.getPower());
        
        telemetry.addLine("");
        telemetry.addLine("=== DIRECTION VERIFICATION ===");
        telemetry.addLine("When moving FORWARD (intake forward):");
        telemetry.addLine("  All encoders should INCREASE (positive)");
        telemetry.addLine("");
        telemetry.addLine("When STRAFING RIGHT:");
        telemetry.addLine("  LF and RR should INCREASE");
        telemetry.addLine("  RF and LR should DECREASE");
        telemetry.addLine("");
        telemetry.addLine("When TURNING RIGHT (clockwise):");
        telemetry.addLine("  Left motors (LF, LR) should INCREASE");
        telemetry.addLine("  Right motors (RF, RR) should DECREASE");
        telemetry.addLine("");
        telemetry.addLine("=== ENCODER DIRECTION EXPLANATION ===");
        telemetry.addLine("Encoder direction tells Pedro Pathing:");
        telemetry.addLine("  - How encoder values change when robot moves");
        telemetry.addLine("  - FORWARD = encoder increases when moving forward");
        telemetry.addLine("  - REVERSE = encoder decreases when moving forward");
        telemetry.addLine("");
        telemetry.addLine("Current Pedro Encoder Directions:");
        // Encoder directions are stored as doubles (1.0 = FORWARD, -1.0 = REVERSE)
        double lfDir = Constants.localizerConstants.leftFrontEncoderDirection;
        double lrDir = Constants.localizerConstants.leftRearEncoderDirection;
        double rfDir = Constants.localizerConstants.rightFrontEncoderDirection;
        double rrDir = Constants.localizerConstants.rightRearEncoderDirection;
        telemetry.addData("  LF", lfDir > 0 ? "FORWARD" : "REVERSE");
        telemetry.addData("  LR", lrDir > 0 ? "FORWARD" : "REVERSE");
        telemetry.addData("  RF", rfDir > 0 ? "FORWARD" : "REVERSE");
        telemetry.addData("  RR", rrDir > 0 ? "FORWARD" : "REVERSE");
        telemetry.addLine("");
        telemetry.addLine("If encoders move OPPOSITE to expected:");
        telemetry.addLine("  Update encoder direction in Constants.java");
        telemetry.addLine("");
        telemetry.addLine("Controls:");
        telemetry.addLine("  X = Toggle Mode");
        telemetry.addLine("  Y = Reset Encoders");
        
        if (individualMotorMode) {
            telemetry.addLine("  D-Pad = Test Individual Motors");
        } else {
            telemetry.addLine("  Left Stick = Forward/Strafe");
            telemetry.addLine("  Right Stick = Turn");
        }
        
        telemetry.update();
    }
}

