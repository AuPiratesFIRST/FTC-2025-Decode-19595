package org.firstinspires.ftc.teamcode.SubSystems.Drive;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DriveSubsystem {

    public final DcMotorEx leftFront, leftRear, rightRear, rightFront;

    private static final double TICKS_PER_REVOLUTION = 560.0;
    private static final double WHEEL_DIAMETER = 4.0; // inches
    private static final double WHEEL_BASE = 16.0; // inches between wheels

    // Tile-based navigation
    private TileCoordinate currentPosition;
    private double currentHeading; // in radians
    private Telemetry telemetry;

    public DriveSubsystem(HardwareMap hardwareMap) {
        this(hardwareMap, null);
    }

    public DriveSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        // Initialize motors
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        configureMotors();

        // Initialize tile-based navigation
        currentPosition = new TileCoordinate(0, 0); // Start at origin
        currentHeading = 0; // Facing right
    }

    private void configureMotors() {
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);

        // Use RUN_USING_ENCODER for closed-loop control
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void drive(double forward, double strafe, double turn) {
        double fl = forward + strafe + turn;
        double fr = forward - strafe - turn;
        double bl = forward - strafe + turn;
        double br = forward + strafe - turn;

        // Clip values to avoid exceeding motor power limits
        leftFront.setPower(Range.clip(fl, -1, 1));
        rightFront.setPower(Range.clip(fr, -1, 1));
        leftRear.setPower(Range.clip(bl, -1, 1));
        rightRear.setPower(Range.clip(br, -1, 1));
    }

    public void stop() {
        drive(0, 0, 0);
    }

    // ==================== TILE-BASED NAVIGATION METHODS ====================

    /**
     * Set the robot's current position in tile coordinates
     * 
     * @param position Current tile coordinate position
     */
    public void setPosition(TileCoordinate position) {
        this.currentPosition = position;
    }

    /**
     * Set the robot's current heading
     * 
     * @param heading Heading in radians (0 = right, π/2 = up, π = left, 3π/2 =
     *                down)
     */
    public void setHeading(double heading) {
        this.currentHeading = heading;
    }

    /**
     * Get the robot's current tile position
     * 
     * @return Current tile coordinate
     */
    public TileCoordinate getCurrentPosition() {
        return currentPosition;
    }

    /**
     * Get the robot's current heading
     * 
     * @return Current heading in radians
     */
    public double getCurrentHeading() {
        return currentHeading;
    }

    /**
     * Move to a specific tile with offset
     * 
     * @param column  Tile column (A-F)
     * @param row     Tile row (1-6)
     * @param offsetX Offset within tile (0-24 inches)
     * @param offsetY Offset within tile (0-24 inches)
     * @param power   Motor power (0-1)
     */
    public void moveToTile(char column, int row, double offsetX, double offsetY, double power) {
        TileCoordinate target = new TileCoordinate(column, row, offsetX, offsetY);
        moveToPosition(target, power);
    }

    /**
     * Move to a tab-line intersection
     * 
     * @param column Tab column (V-Z)
     * @param row    Tab row (1-5)
     * @param power  Motor power (0-1)
     */
    public void moveToTabIntersection(char column, int row, double power) {
        TileCoordinate target = new TileCoordinate(column, row);
        moveToPosition(target, power);
    }

    /**
     * Move to a specific tile coordinate position
     * 
     * @param target Target tile coordinate
     * @param power  Motor power (0-1)
     */
    public void moveToPosition(TileCoordinate target, double power) {
        double distance = currentPosition.distanceTo(target);
        double angle = currentPosition.angleTo(target);

        // Calculate required movement
        double forward = Math.cos(angle) * power;
        double strafe = Math.sin(angle) * power;

        // Calculate turn needed
        double turnAngle = angle - currentHeading;
        double turn = Math.sin(turnAngle) * power * 0.5; // Scale down turn power

        // Apply movement
        drive(forward, strafe, turn);

        // Update position estimate (simplified - in real implementation, use odometry)
        currentPosition = target;
        currentHeading = angle;

        if (telemetry != null) {
            telemetry.addData("Tile Navigation", "Moving to %s", target.getTilePosition());
            telemetry.addData("Distance", "%.1f inches", distance);
            telemetry.addData("Angle", "%.1f degrees", Math.toDegrees(angle));
            telemetry.update();
        }
    }

    /**
     * Move to center of a specific tile
     * 
     * @param column Tile column (A-F)
     * @param row    Tile row (1-6)
     * @param power  Motor power (0-1)
     */
    public void moveToTileCenter(char column, int row, double power) {
        moveToTile(column, row, TileCoordinate.TILE_SIZE / 2, TileCoordinate.TILE_SIZE / 2, power);
    }

    /**
     * Move relative to current position by tile units
     * 
     * @param tilesForward Number of tiles forward (negative for backward)
     * @param tilesRight   Number of tiles right (negative for left)
     * @param power        Motor power (0-1)
     */
    public void moveRelativeTiles(double tilesForward, double tilesRight, double power) {
        double forward = tilesForward * TileCoordinate.TILE_SIZE;
        double strafe = tilesRight * TileCoordinate.TILE_SIZE;

        // Calculate new position
        double newX = currentPosition.getX() + forward * Math.cos(currentHeading) - strafe * Math.sin(currentHeading);
        double newY = currentPosition.getY() + forward * Math.sin(currentHeading) + strafe * Math.cos(currentHeading);

        TileCoordinate target = new TileCoordinate(newX, newY);
        moveToPosition(target, power);
    }

    /**
     * Turn to face a specific tile
     * 
     * @param target Target tile coordinate
     * @param power  Turn power (0-1)
     */
    public void turnToTile(TileCoordinate target, double power) {
        double angle = currentPosition.angleTo(target);
        double turnAngle = angle - currentHeading;

        // Normalize turn angle to [-π, π]
        while (turnAngle > Math.PI)
            turnAngle -= 2 * Math.PI;
        while (turnAngle < -Math.PI)
            turnAngle += 2 * Math.PI;

        double turn = Math.signum(turnAngle) * power;
        drive(0, 0, turn);

        currentHeading = angle;

        if (telemetry != null) {
            telemetry.addData("Turn to Tile", "Facing %s", target.getTilePosition());
            telemetry.addData("Turn Angle", "%.1f degrees", Math.toDegrees(turnAngle));
            telemetry.update();
        }
    }

    /**
     * Get distance to a specific tile
     * 
     * @param target Target tile coordinate
     * @return Distance in inches
     */
    public double getDistanceToTile(TileCoordinate target) {
        return currentPosition.distanceTo(target);
    }

    /**
     * Get angle to a specific tile
     * 
     * @param target Target tile coordinate
     * @return Angle in radians
     */
    public double getAngleToTile(TileCoordinate target) {
        return currentPosition.angleTo(target);
    }

    /**
     * Check if robot is at a specific tile (within tolerance)
     * 
     * @param target    Target tile coordinate
     * @param tolerance Tolerance in inches
     * @return True if within tolerance
     */
    public boolean isAtTile(TileCoordinate target, double tolerance) {
        return getDistanceToTile(target) <= tolerance;
    }

    /**
     * Update telemetry with tile-based information
     */
    public void updateTileTelemetry() {
        if (telemetry != null) {
            telemetry.addData("Current Tile", currentPosition.getTilePosition());
            telemetry.addData("Current Tab", currentPosition.getTabPosition());
            telemetry.addData("Position (inches)", "X: %.1f, Y: %.1f", currentPosition.getX(), currentPosition.getY());
            telemetry.addData("Heading", "%.1f degrees", Math.toDegrees(currentHeading));

            double[] offset = currentPosition.getTileOffset();
            telemetry.addData("Tile Offset", "X: %.1f, Y: %.1f", offset[0], offset[1]);
        }
    }
}
