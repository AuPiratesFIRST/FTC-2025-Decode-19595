package org.firstinspires.ftc.teamcode.TeleOp;

/**
 * Standalone Java test to verify Blue Alliance TeleOp logic without hardware.
 * Tests automated sequences, manual control, and AprilTag alignment logic.
 */
public class BlueAllianceTeleOpLogicTest {

    // Constants
    private static final double TICKS_PER_REVOLUTION = 2150.8;
    private static final int POS_TICK_120 = (int) (TICKS_PER_REVOLUTION / 3.0);
    private static final int POS_TICK_240 = (int) (TICKS_PER_REVOLUTION * 2.0 / 3.0);
    private static final double SHOOTER_POWER = 0.7;
    private static final int TARGET_TAG_ID = 20; // Blue alliance goal

    // Simulated subsystems
    private static class SimulatedSpindexer {
        int currentPosition = 0;
        int targetPosition = 0;
        double power = 0.0;
        boolean isBusy = false;
        boolean manualMode = false;

        void goToPosition(int index) {
            int[] positions = {0, POS_TICK_120, POS_TICK_240};
            this.targetPosition = positions[index];
            this.isBusy = true;
        }

        void setManualPower(double power) {
            this.power = power;
            this.manualMode = true;
            this.isBusy = false;
        }

        boolean isAtPosition() {
            return Math.abs(currentPosition - targetPosition) <= 15;
        }

        int getCurrentPosition() {
            return currentPosition;
        }

        void update() {
            if (!manualMode && isBusy && power > 0) {
                int distance = targetPosition - currentPosition;
                if (Math.abs(distance) <= 15) {
                    currentPosition = targetPosition;
                    isBusy = false;
                    power = 0.0;
                } else {
                    int moveAmount = (int) (50 * power);
                    if (distance > 0) {
                        currentPosition = Math.min(currentPosition + moveAmount, targetPosition);
                    } else {
                        currentPosition = Math.max(currentPosition - moveAmount, targetPosition);
                    }
                }
            }
        }
    }

    private static class SimulatedShooter {
        double currentRPM = 0.0;
        double targetRPM = 0.0;

        void setPower(double power) {
            this.targetRPM = 6000.0 * power;
        }

        void stop() {
            this.targetRPM = 0.0;
        }

        double getCurrentRPM() {
            return currentRPM;
        }

        double getTargetRPM() {
            return targetRPM;
        }

        boolean isAtTargetRPM() {
            if (targetRPM == 0) return false;
            double error = Math.abs(currentRPM - targetRPM);
            return error <= (targetRPM * 0.02);
        }

        void update() {
            if (targetRPM > 0) {
                double error = targetRPM - currentRPM;
                if (Math.abs(error) < 50) {
                    currentRPM = targetRPM;
                } else {
                    currentRPM += error * 0.1;
                }
            } else {
                currentRPM *= 0.9;
            }
        }
    }

    private static class SimulatedAprilTag {
        int tagId = 0;
        double yaw = 0.0;
        boolean detected = false;

        void simulateDetection(int tagId, double yaw) {
            this.tagId = tagId;
            this.yaw = yaw;
            this.detected = (tagId == TARGET_TAG_ID);
        }

        boolean isTargetDetected() {
            return detected && tagId == TARGET_TAG_ID;
        }
    }

    // Test state
    private int spindexerPositionIndex = 0;
    private boolean intakeMode = true;
    private boolean spindexerIsMoving = false;
    private boolean shooterNeedsToSpinUp = false;
    private boolean manualControlMode = false;

    private SimulatedSpindexer spindexer = new SimulatedSpindexer();
    private SimulatedShooter shooter = new SimulatedShooter();
    private SimulatedAprilTag aprilTag = new SimulatedAprilTag();

    public void testAutomatedSequence() {
        System.out.println("=== Testing Blue Alliance Automated Sequence ===\n");

        // Test INTAKE sequence
        System.out.println("--- INTAKE MODE TEST ---");
        intakeMode = true;
        spindexerPositionIndex = 0;

        for (int i = 0; i < 3; i++) {
            System.out.println("\n>>> Intake Ball " + (i + 1) + " <<<");
            simulateButtonPress(); // A button
            
            // Wait for movement
            int iterations = 0;
            while (spindexerIsMoving && iterations < 50) {
                spindexer.update();
                if (spindexer.isAtPosition()) {
                    spindexerIsMoving = false;
                    System.out.println("[Iteration " + iterations + "] Spindexer reached position");
                }
                iterations++;
            }
        }

        // Switch to OUTTAKE mode
        System.out.println("\n>>> SWITCHING TO OUTTAKE MODE <<<");
        intakeMode = false;
        spindexerPositionIndex = 0;

        // Test OUTTAKE sequence
        System.out.println("\n--- OUTTAKE MODE TEST ---");
        for (int i = 0; i < 3; i++) {
            System.out.println("\n>>> Shoot Ball " + (i + 1) + " <<<");
            simulateButtonPress(); // A button
            
            // Wait for spindexer
            int iterations = 0;
            while (spindexerIsMoving && iterations < 50) {
                spindexer.update();
                shooter.update();
                if (spindexer.isAtPosition()) {
                    spindexerIsMoving = false;
                    shooter.setPower(SHOOTER_POWER);
                    shooterNeedsToSpinUp = true;
                    System.out.println("[Iteration " + iterations + "] Spindexer at position, shooter spinning up...");
                }
                iterations++;
            }

            // Wait for shooter RPM
            iterations = 0;
            while (shooterNeedsToSpinUp && iterations < 100) {
                shooter.update();
                if (shooter.isAtTargetRPM()) {
                    shooterNeedsToSpinUp = false;
                    System.out.println("[Iteration " + iterations + "] Shooter at target RPM! Ready to shoot.");
                }
                iterations++;
            }
        }

        System.out.println("\n=== Automated Sequence Test Complete ===\n");
    }

    public void testManualControl() {
        System.out.println("=== Testing Manual Control Mode ===\n");

        manualControlMode = true;
        spindexer.manualMode = true;

        System.out.println("Manual Mode: ENABLED");
        System.out.println("Simulating D-Pad Right press...");
        
        // Simulate manual position advance
        spindexerPositionIndex = (spindexerPositionIndex + 1) % 3;
        spindexer.goToPosition(spindexerPositionIndex);
        spindexer.manualMode = false;
        spindexerIsMoving = true;

        System.out.println("Moving to position " + spindexerPositionIndex);

        int iterations = 0;
        while (spindexerIsMoving && iterations < 50) {
            spindexer.update();
            if (spindexer.isAtPosition()) {
                spindexerIsMoving = false;
                System.out.println("[Iteration " + iterations + "] Reached position in manual mode");
            }
            iterations++;
        }

        System.out.println("\n=== Manual Control Test Complete ===\n");
    }

    public void testAprilTagAlignment() {
        System.out.println("=== Testing AprilTag Alignment (Blue Alliance) ===\n");

        // Test 1: Correct tag detected
        System.out.println("Test 1: Blue Goal (Tag 20) detected");
        aprilTag.simulateDetection(20, 5.0); // 5 degrees off
        if (aprilTag.isTargetDetected()) {
            System.out.println("✓ Correctly detected Blue Goal (Tag 20)");
            System.out.println("  Yaw: " + aprilTag.yaw + "° - Should align");
        } else {
            System.out.println("✗ Failed to detect Blue Goal");
        }

        // Test 2: Wrong tag detected
        System.out.println("\nTest 2: Red Goal (Tag 24) detected (should ignore)");
        aprilTag.simulateDetection(24, 5.0);
        if (!aprilTag.isTargetDetected()) {
            System.out.println("✓ Correctly ignored Red Goal (Tag 24)");
        } else {
            System.out.println("✗ Incorrectly accepted Red Goal");
        }

        // Test 3: No tag detected
        System.out.println("\nTest 3: No tag detected");
        aprilTag.simulateDetection(0, 0.0);
        if (!aprilTag.isTargetDetected()) {
            System.out.println("✓ Correctly handled no detection");
        } else {
            System.out.println("✗ Incorrectly detected tag when none present");
        }

        System.out.println("\n=== AprilTag Alignment Test Complete ===\n");
    }

    public void testModeSwitching() {
        System.out.println("=== Testing Mode Switching ===\n");

        // Test intake to outtake (simulating D-Pad Down press)
        System.out.println("Switching from INTAKE to OUTTAKE");
        intakeMode = true;
        spindexerPositionIndex = 2;
        manualControlMode = false; // Ensure manual mode is off
        System.out.println("  Before: Mode=" + (intakeMode ? "INTAKE" : "OUTTAKE") + ", Position=" + spindexerPositionIndex);
        // Simulate D-Pad Down button press (from TeleOp code)
        if (!manualControlMode) {
            intakeMode = !intakeMode; // Toggle mode
            spindexerPositionIndex = 0; // Reset position
        }
        System.out.println("  After: Mode=" + (intakeMode ? "INTAKE" : "OUTTAKE") + ", Position=" + spindexerPositionIndex);
        if (spindexerPositionIndex == 0 && !intakeMode) {
            System.out.println("✓ Position reset correctly when switching to OUTTAKE mode");
        } else {
            System.out.println("✗ Position not reset correctly (Expected: 0, OUTTAKE. Got: " + spindexerPositionIndex + ", " + (intakeMode ? "INTAKE" : "OUTTAKE") + ")");
        }

        // Test outtake to intake
        System.out.println("\nSwitching from OUTTAKE to INTAKE");
        intakeMode = false;
        spindexerPositionIndex = 1;
        manualControlMode = false; // Ensure manual mode is off
        System.out.println("  Before: Mode=" + (intakeMode ? "INTAKE" : "OUTTAKE") + ", Position=" + spindexerPositionIndex);
        // Simulate D-Pad Down button press (from TeleOp code)
        if (!manualControlMode) {
            intakeMode = !intakeMode; // Toggle mode
            spindexerPositionIndex = 0; // Reset position
        }
        System.out.println("  After: Mode=" + (intakeMode ? "INTAKE" : "OUTTAKE") + ", Position=" + spindexerPositionIndex);
        if (spindexerPositionIndex == 0 && intakeMode) {
            System.out.println("✓ Position reset correctly when switching to INTAKE mode");
        } else {
            System.out.println("✗ Position not reset correctly (Expected: 0, INTAKE. Got: " + spindexerPositionIndex + ", " + (intakeMode ? "INTAKE" : "OUTTAKE") + ")");
        }

        System.out.println("\n=== Mode Switching Test Complete ===\n");
    }

    private void simulateButtonPress() {
        if (!manualControlMode) {
            spindexerPositionIndex = (spindexerPositionIndex + 1) % 3;
            spindexer.goToPosition(spindexerPositionIndex);
            spindexerIsMoving = true;
        }
    }

    public void runAllTests() {
        System.out.println("========================================");
        System.out.println("BLUE ALLIANCE TELEOP LOGIC TESTS");
        System.out.println("========================================\n");

        testAutomatedSequence();
        testManualControl();
        testAprilTagAlignment();
        testModeSwitching();

        System.out.println("========================================");
        System.out.println("ALL TESTS COMPLETE");
        System.out.println("========================================");
    }

    public static void main(String[] args) {
        BlueAllianceTeleOpLogicTest test = new BlueAllianceTeleOpLogicTest();
        test.runAllTests();
    }
}

