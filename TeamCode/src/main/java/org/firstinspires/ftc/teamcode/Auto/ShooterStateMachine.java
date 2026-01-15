package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SubSystems.Shooter.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Spindexer.OldSpindexerSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Funnel.FunnelSubsystem;

/**
 * Shooter state machine matching SimpleBlueAuto behavior.
 */
public class ShooterStateMachine {

    private final ShooterSubsystem shooter;
    private final OldSpindexerSubsystem spindexer;
    private final FunnelSubsystem funnel;
    private final Telemetry telemetry;

    private enum ShooterState {
        IDLE,
        SPIN_UP,
        FUNNEL_EXTENDING,
        FUNNEL_HOLD,
        FUNNEL_RETRACTING
    }

    private ShooterState state = ShooterState.IDLE;
    private final ElapsedTime stateTimer = new ElapsedTime();

    // Shot tracking
    private int shotsRemaining = 0;
    private int spindexerIndex = 0;

    // Constants (MATCH AUTO)
    private static final double TARGET_RPM = 5225;
    private static final double RPM_TOLERANCE = 100;

    private static final long FUNNEL_EXTEND_MS = 400;
    private static final long FUNNEL_HOLD_MS   = 300;
    private static final long FUNNEL_RETRACT_MS = 400;

    public ShooterStateMachine(
            ShooterSubsystem shooter,
            OldSpindexerSubsystem spindexer,
            FunnelSubsystem funnel,
            Telemetry telemetry) {

        this.shooter = shooter;
        this.spindexer = spindexer;
        this.funnel = funnel;
        this.telemetry = telemetry;

        spindexer.setIntakeMode(false);
        funnel.retract();
    }

    /** Call every loop */
    public void update() {

        spindexer.update();
        shooter.updateVoltageCompensation();

        switch (state) {

            case IDLE:
                if (shotsRemaining > 0) {
                    shooter.setTargetRPM(TARGET_RPM);
                    spindexer.goToPositionForCurrentMode(spindexerIndex);
                    state = ShooterState.SPIN_UP;
                    stateTimer.reset();
                }
                break;

            case SPIN_UP:
                shooter.setTargetRPM(TARGET_RPM);

                boolean rpmReady =
                        shooter.isAtTargetRPM() ||
                                Math.abs(shooter.getCurrentRPM() - TARGET_RPM) < RPM_TOLERANCE;

                if (spindexer.isAtPosition() && rpmReady) {
                    funnel.extend();
                    state = ShooterState.FUNNEL_EXTENDING;
                    stateTimer.reset();
                }
                break;

            case FUNNEL_EXTENDING:
                if (stateTimer.milliseconds() >= FUNNEL_EXTEND_MS) {
                    state = ShooterState.FUNNEL_HOLD;
                    stateTimer.reset();
                }
                break;

            case FUNNEL_HOLD:
                if (stateTimer.milliseconds() >= FUNNEL_HOLD_MS) {
                    funnel.retract();
                    state = ShooterState.FUNNEL_RETRACTING;
                    stateTimer.reset();
                }
                break;

            case FUNNEL_RETRACTING:
                if (stateTimer.milliseconds() >= FUNNEL_RETRACT_MS) {

                    shotsRemaining--;
                    spindexerIndex++;

                    if (shotsRemaining > 0) {
                        spindexer.goToPositionForCurrentMode(spindexerIndex);
                        state = ShooterState.SPIN_UP;
                    } else {
                        shooter.stop();
                        state = ShooterState.IDLE;
                    }
                    stateTimer.reset();
                }
                break;
        }
    }

    /** Start firing N shots (1–3) */
    public void fireShots(int shots) {
        if (state == ShooterState.IDLE) {
            shotsRemaining = Math.max(1, Math.min(3, shots));
            spindexerIndex = 0;
        }
    }

    public boolean isBusy() {
        return state != ShooterState.IDLE;
    }

    public void stop() {
        shooter.stop();
        funnel.retract();
        shotsRemaining = 0;
        state = ShooterState.IDLE;
    }

    public void updateTelemetry() {
        if (telemetry != null) {
            telemetry.addData("ShooterState", state);
            telemetry.addData("ShotsRemaining", shotsRemaining);
            telemetry.addData("SpindexerIndex", spindexerIndex);
            telemetry.addData("RPM", "%.0f / %.0f",
                    shooter.getCurrentRPM(), shooter.getTargetRPM());
        }
    }
}
