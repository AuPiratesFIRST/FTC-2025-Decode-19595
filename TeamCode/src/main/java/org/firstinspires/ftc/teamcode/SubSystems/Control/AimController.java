package org.firstinspires.ftc.teamcode.SubSystems.Control;

import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.SubSystems.Drive.DriveSubsystem;

/**
 * AIM CONTROLLER — TILE + IMU ONLY
 *
 * Deterministic alignment controller using:
 *  - Tile / odometry position for X/Y
 *  - IMU for heading
 *  - Correct triangular goal geometry (45° faces)
 *
 * No vision. No sensor fusion. No jitter.
 */
public class AimController {

    /* ===================== ALLIANCE ===================== */

    public enum AllianceColor {
        BLUE,
        RED
    }

    private AllianceColor alliance = AllianceColor.BLUE;

    /* ===================== DEPENDENCIES ===================== */

    private final DriveSubsystem drive;
    private final Telemetry telemetry;

    /* ===================== GOAL GEOMETRY ===================== */

    // Center of triangular goal scoring faces (inches)
    private static final double GOAL_X_BLUE = 12.0;
    private static final double GOAL_Y_BLUE = 132.0;

    private static final double GOAL_X_RED  = 132.0;
    private static final double GOAL_Y_RED  = 132.0;

    // Goal face angles
    private static final double ANGLE_BLUE = 45.0;
    private static final double ANGLE_RED  = 135.0;

    /* ===================== TUNING ===================== */

    private double kpForward = 0.035;
    private double kpStrafe  = 0.035;
    private double kpRotate  = 0.015;

    private double maxPower = 0.45;

    private double posDeadband   = 0.75;  // inches
    private double angleDeadband = 1.5;   // degrees

    /* ===================== CONSTRUCTOR ===================== */

    public AimController(DriveSubsystem drive, Telemetry telemetry) {
        this.drive = drive;
        this.telemetry = telemetry;
    }

    /* ===================== CONFIG ===================== */

    public void setAlliance(AllianceColor alliance) {
        this.alliance = alliance;
    }

    private double goalX() {
        return (alliance == AllianceColor.BLUE) ? GOAL_X_BLUE : GOAL_X_RED;
    }

    private double goalY() {
        return (alliance == AllianceColor.BLUE) ? GOAL_Y_BLUE : GOAL_Y_RED;
    }

    private double goalAngle() {
        return (alliance == AllianceColor.BLUE) ? ANGLE_BLUE : ANGLE_RED;
    }

    /* ===================== RESULT ===================== */

    public static class AlignmentResult {
        public double strafe;
        public double forward;
        public double turn;
        public boolean aligned;

        public AlignmentResult(double s, double f, double t, boolean a) {
            strafe = s;
            forward = f;
            turn = t;
            aligned = a;
        }
    }

    /* ===================== MAIN UPDATE ===================== */

    public AlignmentResult update() {

        /* ---- Robot pose ---- */
        double robotX = drive.getX();
        double robotY = drive.getY();

        /* ---- Vector to goal ---- */
        double dx = goalX() - robotX;
        double dy = goalY() - robotY;

        /* ---- Goal face normal ---- */
        double faceRad = Math.toRadians(goalAngle());
        double nx = Math.cos(faceRad);
        double ny = Math.sin(faceRad);

        /* ---- Project error ---- */
        double forwardError =  dx * nx + dy * ny;
        double strafeError  = -dx * ny + dy * nx;

        /* ---- Heading control (IMU only) ---- */
        double headingDeg = Math.toDegrees(drive.getHeading());
        double angleError = AngleUnit.normalizeDegrees(goalAngle() - headingDeg);

        /* ---- Check aligned ---- */
        boolean aligned =
                Math.abs(forwardError) < posDeadband &&
                Math.abs(strafeError)  < posDeadband &&
                Math.abs(angleError)   < angleDeadband;

        if (aligned) {
            return new AlignmentResult(0, 0, 0, true);
        }

        /* ---- Controller outputs ---- */
        double forward = Range.clip(forwardError * kpForward, -maxPower, maxPower);
        double strafe  = Range.clip(strafeError  * kpStrafe,  -maxPower, maxPower);
        double turn    = Range.clip(angleError   * kpRotate, -maxPower, maxPower);

        /* ---- Telemetry (optional but useful) ---- */
        telemetry.addData("Goal", alliance);
        telemetry.addData("dx / dy", "%.1f , %.1f", dx, dy);
        telemetry.addData("F / S err", "%.2f , %.2f", forwardError, strafeError);
        telemetry.addData("Angle err", "%.2f", angleError);

        return new AlignmentResult(strafe, forward, turn, false);
    }
}
