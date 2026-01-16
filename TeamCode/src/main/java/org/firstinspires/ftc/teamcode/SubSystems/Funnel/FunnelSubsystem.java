package org.firstinspires.ftc.teamcode.SubSystems.Funnel;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl; // Enable custom PWM range (REV Smart Servo 270° mode)
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx; // Access advanced servo controls
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Funnel subsystem with two separate cams.
 * NOW ENABLED FOR 270 DEGREE MODE ON REV SMART SERVOS.
 * IMPORTANT: With the wider PWM range, 1.0 is ~270°; re-tune EXTEND so the cams do not hit the frame.
 */
public class FunnelSubsystem {

    // Use Servo for runtime control, but configure via ServoImplEx to unlock PWM range.
    private final Servo leftCam;
    private final Servo rightCam;
    private final Telemetry telemetry;

    // PHYSICAL CALIBRATION:
    // With PWM 500-2500 µs, 1.0 ≈ 270°. If your cams collide near 180°, drop EXTEND to ~0.66.
    private static final double LEFT_RETRACT  = 1.0;
    private static final double LEFT_EXTEND   = 0.5; // Adjust if 270° is too far
    
    private static final double RIGHT_RETRACT = 1.0;
    private static final double RIGHT_EXTEND  = 0.5; // Adjust if 270° is too far
    
    private boolean isExtended = false;

    /**
     * Initialize the funnel subsystem with two independent cam servos.
     * Enables 270° mode and starts retracted for safety.
     */
    public FunnelSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        // 1) Get servos as ServoImplEx to access PWM controls
        leftCam = hardwareMap.get(ServoImplEx.class, "leftCam");
        rightCam = hardwareMap.get(ServoImplEx.class, "rightCam");

        // 2) Enable 270° mode (REV Smart Servo): widen PWM pulse range
        ((ServoImplEx) leftCam).setPwmRange(new PwmControl.PwmRange(500, 2500));
        ((ServoImplEx) rightCam).setPwmRange(new PwmControl.PwmRange(500, 2500));

        // 3) Mirroring: servos face each other, so reverse one to push in the same direction
        rightCam.setDirection(Servo.Direction.REVERSE);

        // 4) Start safe: ensure both cams are retracted on init
        retract();
    }

    /** Moves both cams forward to push the ball out of the spindexer. */
    public void extend() {
        leftCam.setPosition(LEFT_EXTEND);
        rightCam.setPosition(RIGHT_EXTEND);
        isExtended = true;
    }

    /** Moves both cams back to the ready position (idle). */
    public void retract() {
        leftCam.setPosition(LEFT_RETRACT);
        rightCam.setPosition(RIGHT_RETRACT);
        isExtended = false;
    }

    /** Toggle between extended and retracted positions. */
    public void toggle() {
        if (isExtended) retract();
        else extend();
    }

    /** Synchronize both cams to the same normalized position (0.0–1.0). */
    public void setSyncPosition(double normalized) {
        normalized = Math.max(0.0, Math.min(1.0, normalized));

        double lPos = LEFT_RETRACT + (normalized * (LEFT_EXTEND - LEFT_RETRACT));
        double rPos = RIGHT_RETRACT + (normalized * (RIGHT_EXTEND - RIGHT_RETRACT));

        leftCam.setPosition(lPos);
        rightCam.setPosition(rPos);
        isExtended = normalized > 0.5;
    }

    /** Overload: set individual normalized positions for left and right cams. */
    public void setSyncPosition(double leftNormalized, double rightNormalized) {
        setPositions(leftNormalized, rightNormalized);
    }

    /** Convenience alias for setSyncPosition. */
    public void setPosition(double normalized) {
        setSyncPosition(normalized);
    }

    /** Set individual normalized positions for left and right cams. */
    public void setPositions(double leftNormalized, double rightNormalized) {
        leftNormalized = Math.max(0.0, Math.min(1.0, leftNormalized));
        rightNormalized = Math.max(0.0, Math.min(1.0, rightNormalized));

        double lPos = LEFT_RETRACT + (leftNormalized * (LEFT_EXTEND - LEFT_RETRACT));
        double rPos = RIGHT_RETRACT + (rightNormalized * (RIGHT_EXTEND - RIGHT_RETRACT));

        leftCam.setPosition(lPos);
        rightCam.setPosition(rPos);
        isExtended = (leftNormalized > 0.5 || rightNormalized > 0.5);
    }

    public boolean isExtended() {
        return isExtended;
    }

    public double getLeftPosition() {
        return leftCam.getPosition();
    }

    public double getRightPosition() {
        return rightCam.getPosition();
    }

    public void updateTelemetry() {
        if (telemetry == null) return;
        telemetry.addData("Funnel State", isExtended ? "PUSHING" : "IDLE");
        telemetry.addData("Cam Positions", "L:%.2f R:%.2f",
                leftCam.getPosition(), rightCam.getPosition());
    }
}

