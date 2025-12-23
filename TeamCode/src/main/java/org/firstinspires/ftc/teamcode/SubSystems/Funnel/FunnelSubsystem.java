package org.firstinspires.ftc.teamcode.SubSystems.Funnel;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Funnel subsystem with two synchronized servos that push balls out of the spindexer.
 * The servos are connected to companion gears and move together to push balls forward.
 */
public class FunnelSubsystem {

    private final Servo leftFunnelServo;
    private final Servo rightFunnelServo;
    private final Telemetry telemetry;

    // Servo positions
    private static final double RETRACTED_POSITION = 0.0;  // Retracted position (servos pulled back)
    private static final double EXTENDED_POSITION = 1.0;    // Extended position (servos pushed forward)
    
    // Current state
    private boolean isExtended = false;

    /**
     * Initialize the funnel subsystem with two servos.
     * 
     * @param hardwareMap Hardware map from OpMode
     * @param telemetry Telemetry for debugging
     */
    public FunnelSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        // Initialize servos - adjust names to match your configuration
        leftFunnelServo = hardwareMap.get(Servo.class, "leftFunnel");
        rightFunnelServo = hardwareMap.get(Servo.class, "rightFunnel");

        // Set initial position to retracted
        retract();
    }

    /**
     * Extend both servos forward to push balls out of the spindexer.
     * Servos are synchronized to move together.
     */
    public void extend() {
        leftFunnelServo.setPosition(EXTENDED_POSITION);
        rightFunnelServo.setPosition(EXTENDED_POSITION);
        isExtended = true;

        if (telemetry != null) {
            telemetry.addData("Funnel", "Extended");
        }
    }

    /**
     * Retract both servos back to their starting position.
     * Servos are synchronized to move together.
     */
    public void retract() {
        leftFunnelServo.setPosition(RETRACTED_POSITION);
        rightFunnelServo.setPosition(RETRACTED_POSITION);
        isExtended = false;

        if (telemetry != null) {
            telemetry.addData("Funnel", "Retracted");
        }
    }

    /**
     * Toggle between extended and retracted positions.
     * If currently extended, retracts. If retracted, extends.
     */
    public void toggle() {
        if (isExtended) {
            retract();
        } else {
            extend();
        }
    }

    /**
     * Set both servos to a specific position (0.0 to 1.0).
     * Servos are synchronized to move together.
     * 
     * @param position Position value (0.0 = retracted, 1.0 = extended)
     */
    public void setPosition(double position) {
        position = Math.max(0.0, Math.min(1.0, position)); // Clamp to valid range
        leftFunnelServo.setPosition(position);
        rightFunnelServo.setPosition(position);
        isExtended = position > 0.5; // Consider extended if past halfway

        if (telemetry != null) {
            telemetry.addData("Funnel Position", "%.2f", position);
        }
    }

    /**
     * Set individual servo positions (for fine-tuning if needed).
     * 
     * @param leftPosition Left servo position (0.0 to 1.0)
     * @param rightPosition Right servo position (0.0 to 1.0)
     */
    public void setPositions(double leftPosition, double rightPosition) {
        leftPosition = Math.max(0.0, Math.min(1.0, leftPosition));
        rightPosition = Math.max(0.0, Math.min(1.0, rightPosition));
        
        leftFunnelServo.setPosition(leftPosition);
        rightFunnelServo.setPosition(rightPosition);
        isExtended = (leftPosition + rightPosition) / 2.0 > 0.5;

        if (telemetry != null) {
            telemetry.addData("Funnel Left", "%.2f", leftPosition);
            telemetry.addData("Funnel Right", "%.2f", rightPosition);
        }
    }

    /**
     * Check if funnels are currently extended.
     * 
     * @return True if extended, false if retracted
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
        if (telemetry != null) {
            telemetry.addData("Funnel Status", isExtended ? "Extended" : "Retracted");
            telemetry.addData("Funnel Left Position", "%.2f", getLeftPosition());
            telemetry.addData("Funnel Right Position", "%.2f", getRightPosition());
        }
    }
}

