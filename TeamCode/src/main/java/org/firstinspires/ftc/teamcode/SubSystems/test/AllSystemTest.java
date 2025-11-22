package org.firstinspires.ftc.teamcode.SubSystems.test;

import com.pedropathing.telemetry.SelectableOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


/**
 * Selectable OpMode for testing all subsystems.
 * Provides a menu system to select individual subsystem tests or test all
 * together.
 * 
 * @version 1.0
 */
@TeleOp(name = "All System Test", group = "Test")
public class AllSystemTest extends SelectableOpMode {

    public AllSystemTest() {
        super("Select a Subsystem Test", s -> {
            // Individual subsystem tests
            s.folder("Individual Tests", i -> {
                i.add("Shooter Test", ShooterTestOp::new);
                i.add("Spindexer Test", SpindexerTestOp::new);
                i.add("Intake Test", IntakeTestOp::new);
            });

            // Combined tests
            s.folder("Combined Tests", c -> {
                c.add("All Systems Together", AllSystemsTestOp::new);
                c.add("Shooter + Spindexer", ShooterSpindexerTestOp::new);
                c.add("Intake + Spindexer", IntakeSpindexerTestOp::new);
            });

            // Quick tests
            s.folder("Quick Tests", q -> {
                q.add("Shooter RPM Check", ShooterRPMCheckOp::new);
                q.add("Spindexer Position Check", SpindexerPositionCheckOp::new);
                q.add("Intake Direction Check", IntakeDirectionCheckOp::new);
            });
        });
    }
}

/**
 * Shooter test OpMode
 */
class ShooterTestOp extends OpMode {
    private org.firstinspires.ftc.teamcode.SubSystems.Shooter.ShooterSubsystem shooter;
    private boolean isRunning = false;
    private double currentPower = 0.0;

    @Override
    public void init() {
        shooter = new org.firstinspires.ftc.teamcode.SubSystems.Shooter.ShooterSubsystem(hardwareMap, telemetry);
        telemetry.addData("Status", "Ready");
        telemetry.addLine("Controls:");
        telemetry.addLine("Right Trigger: Increase power");
        telemetry.addLine("Left Trigger: Decrease power");
        telemetry.addLine("A: LOW, B: MEDIUM, X: HIGH, Y: MAX");
        telemetry.addLine("Right Bumper: Toggle on/off");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Power adjustment with triggers
        if (gamepad1.right_trigger > 0.1) {
            currentPower += 0.01;
            if (currentPower > 1.0)
                currentPower = 1.0;
        }
        if (gamepad1.left_trigger > 0.1) {
            currentPower -= 0.01;
            if (currentPower < 0.0)
                currentPower = 0.0;
        }

        // Speed presets
        if (gamepad1.a) {
            shooter.setSpeed(org.firstinspires.ftc.teamcode.SubSystems.Shooter.ShooterSubsystem.ShooterSpeed.LOW);
            currentPower = org.firstinspires.ftc.teamcode.SubSystems.Shooter.ShooterSubsystem.ShooterSpeed.LOW
                    .getPower();
            isRunning = true;
        }
        if (gamepad1.b) {
            shooter.setSpeed(org.firstinspires.ftc.teamcode.SubSystems.Shooter.ShooterSubsystem.ShooterSpeed.MEDIUM);
            currentPower = org.firstinspires.ftc.teamcode.SubSystems.Shooter.ShooterSubsystem.ShooterSpeed.MEDIUM
                    .getPower();
            isRunning = true;
        }
        if (gamepad1.x) {
            shooter.setSpeed(org.firstinspires.ftc.teamcode.SubSystems.Shooter.ShooterSubsystem.ShooterSpeed.HIGH);
            currentPower = org.firstinspires.ftc.teamcode.SubSystems.Shooter.ShooterSubsystem.ShooterSpeed.HIGH
                    .getPower();
            isRunning = true;
        }
        if (gamepad1.y) {
            shooter.setSpeed(org.firstinspires.ftc.teamcode.SubSystems.Shooter.ShooterSubsystem.ShooterSpeed.MAX);
            currentPower = org.firstinspires.ftc.teamcode.SubSystems.Shooter.ShooterSubsystem.ShooterSpeed.MAX
                    .getPower();
            isRunning = true;
        }

        // Toggle on/off
        if (gamepad1.right_bumper) {
            isRunning = !isRunning;
        }

        // Apply power
        if (isRunning) {
            shooter.setPower(currentPower);
        } else {
            shooter.stop();
        }

        telemetry.addData("Status", isRunning ? "RUNNING" : "STOPPED");
        telemetry.addData("Current Power", "%.2f", currentPower);
        shooter.updateTelemetry();
        telemetry.update();
    }
}

/**
 * Spindexer test OpMode
 */
class SpindexerTestOp extends OpMode {
    private org.firstinspires.ftc.teamcode.SubSystems.Spindexer.SpindexerSubsystem spindexer;
    private boolean manualMode = false;
    private double[] pidCoefficients = new double[3];

    @Override
    public void init() {
        spindexer = new org.firstinspires.ftc.teamcode.SubSystems.Spindexer.SpindexerSubsystem(hardwareMap, telemetry);
        pidCoefficients = spindexer.getPIDCoefficients();
        telemetry.addData("Status", "Ready");
        telemetry.addLine("Controls:");
        telemetry.addLine("A: Position 1, B: Position 2, X: Position 3");
        telemetry.addLine("D-Pad: Adjust PID (Up/Down: P, Left/Right: I)");
        telemetry.addLine("Bumpers: Adjust D gain");
        telemetry.addLine("Y: Reset encoder");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Position selection
        if (gamepad1.a) {
            spindexer.goToPosition(
                    org.firstinspires.ftc.teamcode.SubSystems.Spindexer.SpindexerSubsystem.SpindexerPosition.POSITION_1);
            manualMode = false;
        }
        if (gamepad1.b) {
            spindexer.goToPosition(
                    org.firstinspires.ftc.teamcode.SubSystems.Spindexer.SpindexerSubsystem.SpindexerPosition.POSITION_2);
            manualMode = false;
        }
        if (gamepad1.x) {
            spindexer.goToPosition(
                    org.firstinspires.ftc.teamcode.SubSystems.Spindexer.SpindexerSubsystem.SpindexerPosition.POSITION_3);
            manualMode = false;
        }

        // PID tuning
        if (gamepad1.dpad_up) {
            pidCoefficients[0] += 0.001;
            spindexer.setPIDCoefficients(pidCoefficients[0], pidCoefficients[1], pidCoefficients[2]);
        }
        if (gamepad1.dpad_down) {
            pidCoefficients[0] -= 0.001;
            if (pidCoefficients[0] < 0)
                pidCoefficients[0] = 0;
            spindexer.setPIDCoefficients(pidCoefficients[0], pidCoefficients[1], pidCoefficients[2]);
        }
        if (gamepad1.dpad_right) {
            pidCoefficients[1] += 0.0001;
            spindexer.setPIDCoefficients(pidCoefficients[0], pidCoefficients[1], pidCoefficients[2]);
        }
        if (gamepad1.dpad_left) {
            pidCoefficients[1] -= 0.0001;
            if (pidCoefficients[1] < 0)
                pidCoefficients[1] = 0;
            spindexer.setPIDCoefficients(pidCoefficients[0], pidCoefficients[1], pidCoefficients[2]);
        }
        if (gamepad1.right_bumper) {
            pidCoefficients[2] += 0.001;
            spindexer.setPIDCoefficients(pidCoefficients[0], pidCoefficients[1], pidCoefficients[2]);
        }
        if (gamepad1.left_bumper) {
            pidCoefficients[2] -= 0.001;
            if (pidCoefficients[2] < 0)
                pidCoefficients[2] = 0;
            spindexer.setPIDCoefficients(pidCoefficients[0], pidCoefficients[1], pidCoefficients[2]);
        }

        // Reset encoder
        if (gamepad1.y) {
            spindexer.reset();
        }

        // Manual control
        double manualPower = -gamepad1.left_stick_y;
        if (Math.abs(manualPower) > 0.1) {
            spindexer.setManualPower(manualPower * 0.5);
            manualMode = true;
        } else if (!manualMode) {
            spindexer.update();
        }

        telemetry.addData("Status", manualMode ? "MANUAL MODE" : "PID MODE");
        telemetry.addData("PID Coefficients", "P: %.3f, I: %.3f, D: %.3f",
                pidCoefficients[0], pidCoefficients[1], pidCoefficients[2]);
        spindexer.updateTelemetry();
        telemetry.update();
    }
}

/**
 * Intake test OpMode
 */
class IntakeTestOp extends OpMode {
    private org.firstinspires.ftc.teamcode.SubSystems.Intake.IntakeSubsystem intake;
    private boolean forwardToggle = false;
    private boolean reverseToggle = false;

    @Override
    public void init() {
        intake = new org.firstinspires.ftc.teamcode.SubSystems.Intake.IntakeSubsystem(hardwareMap, telemetry);
        telemetry.addData("Status", "Ready");
        telemetry.addLine("Controls:");
        telemetry.addLine("Right Trigger: Forward intake");
        telemetry.addLine("Left Trigger: Reverse/outtake");
        telemetry.addLine("A: Toggle forward, B: Toggle reverse, X: Stop");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Trigger controls
        if (gamepad1.right_trigger > 0.1) {
            intake.start();
            forwardToggle = false;
            reverseToggle = false;
        } else if (gamepad1.left_trigger > 0.1) {
            intake.reverse();
            forwardToggle = false;
            reverseToggle = false;
        } else {
            // Toggle controls
            if (gamepad1.a) {
                forwardToggle = !forwardToggle;
                reverseToggle = false;
                if (forwardToggle) {
                    intake.start();
                } else {
                    intake.stop();
                }
            }
            if (gamepad1.b) {
                reverseToggle = !reverseToggle;
                forwardToggle = false;
                if (reverseToggle) {
                    intake.reverse();
                } else {
                    intake.stop();
                }
            }
            if (gamepad1.x) {
                intake.stop();
                forwardToggle = false;
                reverseToggle = false;
            }

            // Manual power control
            double manualPower = -gamepad1.right_stick_y;
            if (Math.abs(manualPower) > 0.1) {
                intake.setPower(manualPower);
                forwardToggle = false;
                reverseToggle = false;
            } else if (!forwardToggle && !reverseToggle) {
                intake.stop();
            }
        }

        telemetry.addData("Status", intake.isRunning() ? "ACTIVE" : "STOPPED");
        intake.updateTelemetry();
        telemetry.update();
    }
}

/**
 * Test all subsystems together
 */
class AllSystemsTestOp extends OpMode {
    private org.firstinspires.ftc.teamcode.SubSystems.Shooter.ShooterSubsystem shooter;
    private org.firstinspires.ftc.teamcode.SubSystems.Spindexer.SpindexerSubsystem spindexer;
    private org.firstinspires.ftc.teamcode.SubSystems.Intake.IntakeSubsystem intake;

    @Override
    public void init() {
        telemetry.addData("Status", "Initializing All Systems...");
        telemetry.update();

        shooter = new org.firstinspires.ftc.teamcode.SubSystems.Shooter.ShooterSubsystem(hardwareMap, telemetry);
        spindexer = new org.firstinspires.ftc.teamcode.SubSystems.Spindexer.SpindexerSubsystem(hardwareMap, telemetry);
        intake = new org.firstinspires.ftc.teamcode.SubSystems.Intake.IntakeSubsystem(hardwareMap, telemetry);

        telemetry.addData("Status", "All Systems Ready");
        telemetry.addLine();
        telemetry.addLine("Gamepad 1 Controls:");
        telemetry.addLine("Shooter: Right Trigger (power), A/B/X/Y (presets)");
        telemetry.addLine("Spindexer: D-Pad (positions), Left Stick Y (manual)");
        telemetry.addLine();
        telemetry.addLine("Gamepad 2 Controls:");
        telemetry.addLine("Intake: Right Trigger (forward), Left Trigger (reverse)");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Shooter controls (Gamepad 1)
        if (gamepad1.right_trigger > 0.1) {
            shooter.setPower(gamepad1.right_trigger);
        } else if (gamepad1.a) {
            shooter.setSpeed(org.firstinspires.ftc.teamcode.SubSystems.Shooter.ShooterSubsystem.ShooterSpeed.LOW);
        } else if (gamepad1.b) {
            shooter.setSpeed(org.firstinspires.ftc.teamcode.SubSystems.Shooter.ShooterSubsystem.ShooterSpeed.MEDIUM);
        } else if (gamepad1.x) {
            shooter.setSpeed(org.firstinspires.ftc.teamcode.SubSystems.Shooter.ShooterSubsystem.ShooterSpeed.HIGH);
        } else if (gamepad1.y) {
            shooter.setSpeed(org.firstinspires.ftc.teamcode.SubSystems.Shooter.ShooterSubsystem.ShooterSpeed.MAX);
        } else if (gamepad1.right_bumper) {
            shooter.stop();
        }

        // Spindexer controls (Gamepad 1 D-Pad)
        if (gamepad1.dpad_up) {
            spindexer.goToPosition(
                    org.firstinspires.ftc.teamcode.SubSystems.Spindexer.SpindexerSubsystem.SpindexerPosition.POSITION_1);
        } else if (gamepad1.dpad_right) {
            spindexer.goToPosition(
                    org.firstinspires.ftc.teamcode.SubSystems.Spindexer.SpindexerSubsystem.SpindexerPosition.POSITION_2);
        } else if (gamepad1.dpad_down) {
            spindexer.goToPosition(
                    org.firstinspires.ftc.teamcode.SubSystems.Spindexer.SpindexerSubsystem.SpindexerPosition.POSITION_3);
        }
        spindexer.update();

        // Intake controls (Gamepad 2)
        if (gamepad2.right_trigger > 0.1) {
            intake.start();
        } else if (gamepad2.left_trigger > 0.1) {
            intake.reverse();
        } else {
            intake.stop();
        }

        // Update telemetry
        telemetry.addData("=== ALL SYSTEMS TEST ===", "");
        shooter.updateTelemetry();
        telemetry.addLine();
        spindexer.updateTelemetry();
        telemetry.addLine();
        intake.updateTelemetry();
        telemetry.update();
    }
}

/**
 * Test Shooter and Spindexer together
 */
class ShooterSpindexerTestOp extends OpMode {
    private org.firstinspires.ftc.teamcode.SubSystems.Shooter.ShooterSubsystem shooter;
    private org.firstinspires.ftc.teamcode.SubSystems.Spindexer.SpindexerSubsystem spindexer;

    @Override
    public void init() {
        telemetry.addData("Status", "Initializing Shooter + Spindexer...");
        telemetry.update();

        shooter = new org.firstinspires.ftc.teamcode.SubSystems.Shooter.ShooterSubsystem(hardwareMap, telemetry);
        spindexer = new org.firstinspires.ftc.teamcode.SubSystems.Spindexer.SpindexerSubsystem(hardwareMap, telemetry);

        telemetry.addData("Status", "Ready");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Shooter controls
        if (gamepad1.right_trigger > 0.1) {
            shooter.setPower(gamepad1.right_trigger);
        } else if (gamepad1.a) {
            shooter.setSpeed(org.firstinspires.ftc.teamcode.SubSystems.Shooter.ShooterSubsystem.ShooterSpeed.LOW);
        } else if (gamepad1.b) {
            shooter.setSpeed(org.firstinspires.ftc.teamcode.SubSystems.Shooter.ShooterSubsystem.ShooterSpeed.MEDIUM);
        } else if (gamepad1.x) {
            shooter.setSpeed(org.firstinspires.ftc.teamcode.SubSystems.Shooter.ShooterSubsystem.ShooterSpeed.HIGH);
        } else {
            shooter.stop();
        }

        // Spindexer controls
        if (gamepad1.dpad_up) {
            spindexer.goToPosition(
                    org.firstinspires.ftc.teamcode.SubSystems.Spindexer.SpindexerSubsystem.SpindexerPosition.POSITION_1);
        } else if (gamepad1.dpad_right) {
            spindexer.goToPosition(
                    org.firstinspires.ftc.teamcode.SubSystems.Spindexer.SpindexerSubsystem.SpindexerPosition.POSITION_2);
        } else if (gamepad1.dpad_down) {
            spindexer.goToPosition(
                    org.firstinspires.ftc.teamcode.SubSystems.Spindexer.SpindexerSubsystem.SpindexerPosition.POSITION_3);
        }
        spindexer.update();

        telemetry.addData("=== SHOOTER + SPINDEXER ===", "");
        shooter.updateTelemetry();
        telemetry.addLine();
        spindexer.updateTelemetry();
        telemetry.update();
    }
}

/**
 * Test Intake and Spindexer together
 */
class IntakeSpindexerTestOp extends OpMode {
    private org.firstinspires.ftc.teamcode.SubSystems.Intake.IntakeSubsystem intake;
    private org.firstinspires.ftc.teamcode.SubSystems.Spindexer.SpindexerSubsystem spindexer;

    @Override
    public void init() {
        telemetry.addData("Status", "Initializing Intake + Spindexer...");
        telemetry.update();

        intake = new org.firstinspires.ftc.teamcode.SubSystems.Intake.IntakeSubsystem(hardwareMap, telemetry);
        spindexer = new org.firstinspires.ftc.teamcode.SubSystems.Spindexer.SpindexerSubsystem(hardwareMap, telemetry);

        telemetry.addData("Status", "Ready");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Intake controls
        if (gamepad1.right_trigger > 0.1) {
            intake.start();
        } else if (gamepad1.left_trigger > 0.1) {
            intake.reverse();
        } else {
            intake.stop();
        }

        // Spindexer controls
        if (gamepad1.dpad_up) {
            spindexer.goToPosition(
                    org.firstinspires.ftc.teamcode.SubSystems.Spindexer.SpindexerSubsystem.SpindexerPosition.POSITION_1);
        } else if (gamepad1.dpad_right) {
            spindexer.goToPosition(
                    org.firstinspires.ftc.teamcode.SubSystems.Spindexer.SpindexerSubsystem.SpindexerPosition.POSITION_2);
        } else if (gamepad1.dpad_down) {
            spindexer.goToPosition(
                    org.firstinspires.ftc.teamcode.SubSystems.Spindexer.SpindexerSubsystem.SpindexerPosition.POSITION_3);
        }
        spindexer.update();

        telemetry.addData("=== INTAKE + SPINDEXER ===", "");
        intake.updateTelemetry();
        telemetry.addLine();
        spindexer.updateTelemetry();
        telemetry.update();
    }
}

/**
 * Quick test to check if shooter reaches target RPM
 */
class ShooterRPMCheckOp extends OpMode {
    private org.firstinspires.ftc.teamcode.SubSystems.Shooter.ShooterSubsystem shooter;
    private boolean testStarted = false;

    @Override
    public void init() {
        shooter = new org.firstinspires.ftc.teamcode.SubSystems.Shooter.ShooterSubsystem(hardwareMap, telemetry);
        telemetry.addData("Status", "Press A to start RPM check");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (gamepad1.a && !testStarted) {
            testStarted = true;
            shooter.setSpeed(org.firstinspires.ftc.teamcode.SubSystems.Shooter.ShooterSubsystem.ShooterSpeed.MEDIUM);
        }

        if (testStarted) {
            shooter.updateTelemetry();
            if (shooter.isAtTargetRPM()) {
                telemetry.addData("RPM CHECK", "PASSED - Shooter at target RPM!");
            } else {
                telemetry.addData("RPM CHECK", "WAITING - Spinning up...");
            }
        }

        telemetry.update();
    }
}

/**
 * Quick test to check spindexer positions
 */
class SpindexerPositionCheckOp extends OpMode {
    private org.firstinspires.ftc.teamcode.SubSystems.Spindexer.SpindexerSubsystem spindexer;
    private int currentTestPosition = 0;
    private boolean lastAPressed = false;

    @Override
    public void init() {
        spindexer = new org.firstinspires.ftc.teamcode.SubSystems.Spindexer.SpindexerSubsystem(hardwareMap, telemetry);
        telemetry.addData("Status", "Press A to cycle through positions");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Debounce button press to avoid rapid cycling
        if (gamepad1.a && !lastAPressed) {
            currentTestPosition = (currentTestPosition + 1) % 3;
            spindexer.goToPosition(currentTestPosition);
        }
        lastAPressed = gamepad1.a;

        spindexer.update();
        spindexer.updateTelemetry();
        telemetry.addData("Testing Position", currentTestPosition + 1);
        telemetry.update();
    }
}

/**
 * Quick test to check intake directions
 */
class IntakeDirectionCheckOp extends OpMode {
    private org.firstinspires.ftc.teamcode.SubSystems.Intake.IntakeSubsystem intake;

    @Override
    public void init() {
        intake = new org.firstinspires.ftc.teamcode.SubSystems.Intake.IntakeSubsystem(hardwareMap, telemetry);
        telemetry.addData("Status", "A: Forward, B: Reverse, X: Stop");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            intake.start();
        } else if (gamepad1.b) {
            intake.reverse();
        } else if (gamepad1.x) {
            intake.stop();
        }

        intake.updateTelemetry();
        telemetry.update();
    }
}
