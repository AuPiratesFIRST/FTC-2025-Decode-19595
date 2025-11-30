package org.firstinspires.ftc.teamcode.SubSystems.test;

/**
 * Enhanced standalone Java test to verify and optimize the spindexer/shooter sequence logic.
 * 
 * Tests full sequence:
 * 1. INTAKE: Move to intake positions (slower speed) → Ball settling movement (0.25" counterclockwise)
 * 2. OUTTAKE: Move to outtake positions → Wait for position → Spin up shooter → Wait for RPM
 * 
 * Runs multiple iterations to find optimal timing and speeds.
 */
public class Decode20252026LogicTest {

    // Constants from Decode20252026
    private static final double TICKS_PER_REVOLUTION = 2150.0;
    private static final int POS_TICK_120 = (int) (TICKS_PER_REVOLUTION / 3.0); // 716
    private static final int POS_TICK_240 = (int) (TICKS_PER_REVOLUTION * 2.0 / 3.0); // 1433
    private static final int POSITION_TOLERANCE = 15;
    private static final double SHOOTER_POWER = 0.7;
    
    // Speed settings (optimizable)
    private static double INTAKE_SPINDEXER_POWER = 0.4; // Slower for ball entry
    private static double OUTTAKE_SPINDEXER_POWER = 0.8; // Faster for shooting
    
    // Intake positions
    private static final int[] INTAKE_POSITIONS = { 0, POS_TICK_120, POS_TICK_240 };
    
    // Outtake positions (normalized from measured values {-90, -265, -434})
    private static int normalizePosition(int rawPosition) {
        int normalized = rawPosition % (int) TICKS_PER_REVOLUTION;
        if (normalized < 0) normalized += (int) TICKS_PER_REVOLUTION;
        return normalized;
    }
    
    private static final int[] OUTTAKE_POSITIONS = {
        normalizePosition(-90),   // 2060
        normalizePosition(-265),  // 1885
        normalizePosition(-434)   // 1716
    };
    
    // Ball settling: 0.25 inches counterclockwise
    // Assuming spindexer radius ~2.5 inches: 0.25" / (2π * 2.5") * 2150 ticks ≈ 34 ticks
    private static final int BALL_SETTLING_TICKS = 34; // Adjust based on your spindexer size
    
    // Timing statistics
    private static class TimingStats {
        long totalIntakeTime = 0;
        long totalOuttakeTime = 0;
        long totalSettlingTime = 0;
        long totalShooterSpinUpTime = 0;
        int iterations = 0;
        
        void addIntakeTime(long time) { totalIntakeTime += time; iterations++; }
        void addOuttakeTime(long time) { totalOuttakeTime += time; }
        void addSettlingTime(long time) { totalSettlingTime += time; }
        void addShooterTime(long time) { totalShooterSpinUpTime += time; }
        
        void print() {
            if (iterations == 0) return;
            System.out.println("\n=== TIMING STATISTICS ===");
            System.out.printf("Average Intake Time: %.1f iterations\n", (double)totalIntakeTime / iterations);
            System.out.printf("Average Outtake Time: %.1f iterations\n", (double)totalOuttakeTime / iterations);
            System.out.printf("Average Settling Time: %.1f iterations\n", (double)totalSettlingTime / iterations);
            System.out.printf("Average Shooter Spin-Up Time: %.1f iterations\n", (double)totalShooterSpinUpTime / iterations);
            System.out.printf("Total Cycle Time (3 balls): %.1f iterations\n", 
                (double)(totalIntakeTime + totalOuttakeTime + totalSettlingTime + totalShooterSpinUpTime) / iterations);
        }
    }

    // Simulated motor state
    private static class SimulatedSpindexer {
        int currentPosition = 0;
        int targetPosition = 0;
        double power = 0.0;
        boolean isBusy = false;
        double currentSpeed = 0.0; // Current speed multiplier

        void setTargetPosition(int target) {
            this.targetPosition = target;
            this.isBusy = true;
        }

        void setPower(double power) {
            this.power = power;
            this.currentSpeed = power;
        }

        int getCurrentPosition() {
            return currentPosition;
        }

        boolean isBusy() {
            return isBusy;
        }

        // Simulate motor movement - speed depends on power setting
        void update() {
            if (isBusy && power > 0) {
                int distance = targetPosition - currentPosition;
                if (Math.abs(distance) <= POSITION_TOLERANCE) {
                    currentPosition = targetPosition;
                    isBusy = false;
                    power = 0.0;
                } else {
                    // Move speed depends on power (slower for intake, faster for outtake)
                    int moveAmount = (int) (50 * power * currentSpeed);
                    if (distance > 0) {
                        currentPosition = Math.min(currentPosition + moveAmount, targetPosition);
                    } else {
                        currentPosition = Math.max(currentPosition - moveAmount, targetPosition);
                    }
                }
            }
        }
    }

    // Simulated shooter state
    private static class SimulatedShooter {
        double currentRPM = 0.0;
        double targetRPM = 0.0;
        boolean isSpinningUp = false;

        void setPower(double power) {
            this.targetRPM = 6000.0 * power;
            this.isSpinningUp = true;
        }

        void stop() {
            this.targetRPM = 0.0;
            this.isSpinningUp = false;
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
            double tolerance = targetRPM * 0.02; // 2% tolerance
            return error <= tolerance;
        }

        void update() {
            if (isSpinningUp && targetRPM > 0) {
                double error = targetRPM - currentRPM;
                if (Math.abs(error) < 50) {
                    currentRPM = targetRPM;
                } else {
                    currentRPM += error * 0.1;
                }
            } else if (targetRPM == 0) {
                currentRPM *= 0.9;
            }
        }
    }

    // Test state
    private int spindexerPosition = 0;
    private int absoluteTarget = 0;
    private boolean spindexerIsMoving = false;
    private boolean shooterNeedsToSpinUp = false;
    private boolean intakeMode = true; // Start in INTAKE mode
    private boolean ballSettling = false; // Track if ball settling movement is active
    private int settlingTarget = 0;

    private SimulatedSpindexer spindexer = new SimulatedSpindexer();
    private SimulatedShooter shooter = new SimulatedShooter();
    private TimingStats stats = new TimingStats();

    // Test the full sequence: Intake 3 balls, then outtake/shoot 3 balls
    public void testFullSequence() {
        System.out.println("=== Testing Full Intake → Outtake Sequence ===\n");
        System.out.println("Configuration:");
        System.out.println("  Intake Speed: " + INTAKE_SPINDEXER_POWER);
        System.out.println("  Outtake Speed: " + OUTTAKE_SPINDEXER_POWER);
        System.out.println("  Ball Settling: " + BALL_SETTLING_TICKS + " ticks (0.25\" counterclockwise)\n");

        long intakeStartTime = 0;
        long outtakeStartTime = 0;
        long settlingStartTime = 0;
        long shooterStartTime = 0;
        int ballsIntaken = 0;
        int ballsShot = 0;

        // Simulate button presses and loop iterations
        for (int iteration = 0; iteration < 500; iteration++) {
            spindexer.update();
            shooter.update();

            // === INTAKE MODE LOGIC ===
            if (intakeMode) {
                // Check if spindexer reached position
                if (spindexerIsMoving) {
                    if (isSpindexerAtTarget()) {
                        spindexer.setPower(0);
                        spindexerIsMoving = false;
                        
                        // Start ball settling movement (counterclockwise = negative)
                        settlingTarget = spindexer.getCurrentPosition() - BALL_SETTLING_TICKS;
                        spindexer.setTargetPosition(settlingTarget);
                        spindexer.setPower(INTAKE_SPINDEXER_POWER);
                        ballSettling = true;
                        settlingStartTime = iteration;
                    }
                }
                
                // Check if ball settling is complete
                if (ballSettling) {
                    if (Math.abs(spindexer.getCurrentPosition() - settlingTarget) <= POSITION_TOLERANCE) {
                        spindexer.setPower(0);
                        ballSettling = false;
                        ballsIntaken++;
                        stats.addSettlingTime(iteration - settlingStartTime);
                        System.out.println("[Iteration " + iteration + "] Ball " + ballsIntaken + " settled. Ready for next intake.");
                        
                        // Switch to outtake after 3 balls
                        if (ballsIntaken >= 3) {
                            intakeMode = false;
                            spindexerPosition = 0;
                            System.out.println("\n>>> SWITCHING TO OUTTAKE MODE <<<\n");
                        }
                    }
                }
                
                // Simulate button press for intake (auto-advance)
                if (!spindexerIsMoving && !ballSettling && ballsIntaken < 3) {
                    if (intakeStartTime == 0) intakeStartTime = iteration;
                    moveToNextPosition(true);
                }
            }
            
            // === OUTTAKE MODE LOGIC ===
            else {
                // Check if spindexer reached position
                if (spindexerIsMoving) {
                    if (isSpindexerAtTarget()) {
                        spindexer.setPower(0);
                        spindexerIsMoving = false;
                        
                        // Start shooter spin-up
                        shooter.setPower(SHOOTER_POWER);
                        shooterNeedsToSpinUp = true;
                        shooterStartTime = iteration;
                        System.out.println("[Iteration " + iteration + "] Spindexer at shooting position. Shooter spinning up...");
                    }
                }
                
                // Check if shooter reached RPM
                if (shooterNeedsToSpinUp) {
                    if (shooter.isAtTargetRPM()) {
                        shooterNeedsToSpinUp = false;
                        ballsShot++;
                        stats.addShooterTime(iteration - shooterStartTime);
                        System.out.println("[Iteration " + iteration + "] Ball " + ballsShot + " shot! Ready for next.");
                    }
                }
                
                // Simulate button press for outtake (auto-advance)
                boolean canMove = !spindexerIsMoving && (!shooterNeedsToSpinUp || shooter.isAtTargetRPM());
                if (canMove && ballsShot < 3) {
                    if (outtakeStartTime == 0) outtakeStartTime = iteration;
                    moveToNextPosition(false);
                }
            }

            // Print status every 20 iterations
            if (iteration % 20 == 0) {
                printStatus(iteration, intakeMode, ballsIntaken, ballsShot);
            }
        }

        // Calculate final timing
        if (intakeStartTime > 0) {
            stats.addIntakeTime(500 - (int)intakeStartTime); // Approximate
        }
        if (outtakeStartTime > 0) {
            stats.addOuttakeTime(500 - (int)outtakeStartTime); // Approximate
        }

        System.out.println("\n=== Test Complete ===");
        System.out.println("Balls Intaken: " + ballsIntaken);
        System.out.println("Balls Shot: " + ballsShot);
        stats.print();
        
        // Print recommendations
        printRecommendations();
    }

    private void moveToNextPosition(boolean isIntake) {
        spindexerPosition = (spindexerPosition + 1) % 3;
        int[] positions = isIntake ? INTAKE_POSITIONS : OUTTAKE_POSITIONS;
        double speed = isIntake ? INTAKE_SPINDEXER_POWER : OUTTAKE_SPINDEXER_POWER;
        
        int nextNormalizedTarget = positions[spindexerPosition];
        int currentRawPosition = spindexer.getCurrentPosition();
        int currentNormalizedPosition = currentRawPosition % (int) TICKS_PER_REVOLUTION;
        if (currentNormalizedPosition < 0) currentNormalizedPosition += (int) TICKS_PER_REVOLUTION;
        
        int travelDifference = nextNormalizedTarget - currentNormalizedPosition;
        if (travelDifference < 0) {
            travelDifference += (int) TICKS_PER_REVOLUTION;
        }
        
        absoluteTarget = currentRawPosition + travelDifference;
        spindexer.setTargetPosition(absoluteTarget);
        spindexer.setPower(speed);
        spindexerIsMoving = true;
    }

    private boolean isSpindexerAtTarget() {
        int currentPos = spindexer.getCurrentPosition();
        int distanceToTarget = Math.abs(absoluteTarget - currentPos);
        return distanceToTarget <= POSITION_TOLERANCE;
    }

    private void printStatus(int iteration, boolean intakeMode, int ballsIntaken, int ballsShot) {
        System.out.println("--- Iteration " + iteration + " ---");
        System.out.println("  Mode: " + (intakeMode ? "INTAKE" : "OUTTAKE"));
        System.out.println("  Balls Intaken: " + ballsIntaken + " / 3");
        System.out.println("  Balls Shot: " + ballsShot + " / 3");
        System.out.println("  Spindexer Position: " + spindexer.getCurrentPosition());
        System.out.println("  Shooter RPM: " + String.format("%.0f", shooter.getCurrentRPM()) + 
                         " / " + String.format("%.0f", shooter.getTargetRPM()));
    }

    private void printRecommendations() {
        System.out.println("\n=== OPTIMIZATION RECOMMENDATIONS ===");
        System.out.println("Current Settings:");
        System.out.println("  Intake Speed: " + INTAKE_SPINDEXER_POWER);
        System.out.println("  Outtake Speed: " + OUTTAKE_SPINDEXER_POWER);
        System.out.println("  Ball Settling: " + BALL_SETTLING_TICKS + " ticks");
        System.out.println("\nSuggested Optimizations:");
        System.out.println("  1. If intake is too fast, reduce INTAKE_SPINDEXER_POWER to 0.3-0.35");
        System.out.println("  2. If outtake is too slow, increase OUTTAKE_SPINDEXER_POWER to 0.9-1.0");
        System.out.println("  3. Adjust BALL_SETTLING_TICKS based on actual spindexer radius:");
        System.out.println("     Formula: (0.25 inches) / (2π × radius) × 2150 ticks");
        System.out.println("     Example: 2.5\" radius = 34 ticks, 3\" radius = 28 ticks");
    }

    // Run multiple test iterations to find optimal settings
    public void runOptimizationTests() {
        System.out.println("=== Running Optimization Tests ===\n");
        
        double[] intakeSpeeds = {0.3, 0.35, 0.4, 0.45};
        double[] outtakeSpeeds = {0.7, 0.8, 0.9};
        
        for (double intakeSpeed : intakeSpeeds) {
            for (double outtakeSpeed : outtakeSpeeds) {
                INTAKE_SPINDEXER_POWER = intakeSpeed;
                OUTTAKE_SPINDEXER_POWER = outtakeSpeed;
                
                System.out.println("\n--- Testing: Intake=" + intakeSpeed + ", Outtake=" + outtakeSpeed + " ---");
                
                // Reset state
                spindexer = new SimulatedSpindexer();
                shooter = new SimulatedShooter();
                spindexerPosition = 0;
                absoluteTarget = 0;
                spindexerIsMoving = false;
                shooterNeedsToSpinUp = false;
                intakeMode = true;
                ballSettling = false;
                stats = new TimingStats();
                
                testFullSequence();
            }
        }
    }

    public static void main(String[] args) {
        Decode20252026LogicTest test = new Decode20252026LogicTest();
        
        // Run single test first
        test.testFullSequence();
        
        // Uncomment to run optimization tests (takes longer)
        // test.runOptimizationTests();
    }
}
