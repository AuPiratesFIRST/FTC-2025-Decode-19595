package org.firstinspires.ftc.teamcode.SubSystems.Funnel;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Funnel subsystem with two separate cams.
 * Since there are no gears, we handle the mirroring and synchronization here.
 * 
 * IMPORTANT: The servos face each other, so one must be reversed to ensure
 * both cams push the ball in the same direction (not opposing each other).
 */
public class FunnelSubsystem {

    private final Servo leftFunnelServo;
    private final Servo rightFunnelServo;
    private final Telemetry telemetry;

    // PHYSICAL CALIBRATION:
    // Cams rarely use the full 0.0 to 1.0 range. 
    // Adjust these values so the cams don't hit the robot frame.
    // Start with conservative values (0.4-0.6) during testing, then expand to full range.
    private static final double LEFT_RETRACT  = 0.1; 
    private static final double LEFT_EXTEND   = 0.9;
    
    private static final double RIGHT_RETRACT = 0.1; 
    private static final double RIGHT_EXTEND  = 0.9;
    
    private boolean isExtended = false;

    /**
     * Initialize the funnel subsystem with two independent cam servos.
     * 
     * @param hardwareMap Hardware map from OpMode
     * @param telemetry Telemetry for debugging
     */
    public FunnelSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        leftFunnelServo = hardwareMap.get(Servo.class, "leftFunnel");
        rightFunnelServo = hardwareMap.get(Servo.class, "rightFunnel");

        // MIRRORING: 
        // If the servos face each other, one must be reversed so that 
        // a single logical command moves both cams "Forward".
        // Adjust this if testing shows the cams move in opposite directions.
        rightFunnelServo.setDirection(Servo.Direction.REVERSE);

        retract();
    }

    /**
     * Moves both cams forward to push the ball out of the spindexer.
     * This is the active "firing" position.
     */
    public void extend() {
        leftFunnelServo.setPosition(LEFT_EXTEND);
        rightFunnelServo.setPosition(RIGHT_EXTEND);
        isExtended = true;
    }

    /**
     * Moves both cams back to the ready position.
     * This is the idle state where cams are clear of the ball path.
     */
    public void retract() {
        leftFunnelServo.setPosition(LEFT_RETRACT);
        rightFunnelServo.setPosition(RIGHT_RETRACT);
        isExtended = false;
    }

    /**
     * Toggle between extended and retracted positions.
     * If currently extended, retracts. If retracted, extends.
     */
    public void toggle() {
        if (isExtended) retract();
        else extend();
    }

    /**
     * Synchronization helper:
     * Moves both cams to the same relative percentage of their travel range.
     * Even without gears, we want them to move in sync.
     * 
     * @param normalized Position from 0.0 (fully retracted) to 1.0 (fully extended)
     */
    public void setSyncPosition(double normalized) {
        // Clamp input to valid range
        normalized = Math.max(0.0, Math.min(1.0, normalized));
        
        // Map 0.0-1.0 to the specific physical limits of each cam
        double lPos = LEFT_RETRACT + (normalized * (LEFT_EXTEND - LEFT_RETRACT));
        double rPos = RIGHT_RETRACT + (normalized * (RIGHT_EXTEND - RIGHT_RETRACT));
        
        leftFunnelServo.setPosition(lPos);
        rightFunnelServo.setPosition(rPos);
        isExtended = normalized > 0.5;
    }

    /**
     * Check if funnels are currently extended.
     * 
     * @return True if extended (pushing), false if retracted (idle)
     */
    public boolean isExtended() {
        return isExtended;
    }

    /**
     * Get current position of left servo.
     * 
     * @return Position value (0.0 to 1.0)
     */
    public double getLeftPosition() {
        return leftFunnelServo.getPosition();
    }

    /**
     * Get current position of right servo.
     * 
     * @return Position value (0.0 to 1.0)
     */
    public double getRightPosition() {
        return rightFunnelServo.getPosition();
    }

    /**
     * Update telemetry with funnel information.
     */
    public void updateTelemetry() {
        if (telemetry == null) return;
        telemetry.addData("Funnel State", isExtended ? "PUSHING" : "IDLE");
        telemetry.addData("Cam Positions", "L:%.2f R:%.2f", 
                leftFunnelServo.getPosition(), rightFunnelServo.getPosition());
    }
}

