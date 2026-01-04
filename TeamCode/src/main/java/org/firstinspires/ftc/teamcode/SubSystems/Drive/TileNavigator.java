package org.firstinspires.ftc.teamcode.SubSystems.Drive;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Handles high-level tile-based navigation logic for DECODE field.
 * 
 * This class translates field coordinates and tile positions into motor powers.
 * It uses the DriveSubsystem for low-level hardware control (motors, IMU).
 * 
 * Responsibilities:
 * - Position tracking (current tile coordinate)
 * - Coordinate transformations (field-to-robot conversions)
 * - Navigation algorithms (move to tile, turn to tile, etc.)
 * - Trigonometric calculations for pathfinding
 * 
 * Single Responsibility: Navigation logic only, no hardware control.
 */
public class TileNavigator {
    private final DriveSubsystem drive;
    private final Telemetry telemetry;
    
    // Internal State: Current robot position in field coordinates
    private TileCoordinate currentPosition;

    public TileNavigator(DriveSubsystem drive, Telemetry telemetry) {
        this.drive = drive;
        this.telemetry = telemetry;
        this.currentPosition = new TileCoordinate(0, 0); // Start at origin (V1/A1)
    }

    // ==================== POSITION TRACKING ====================

    /**
     * Set the robot's current position in tile coordinates.
     * Call this after AprilTag localization to update "ground truth".
     *
     * @param position Current tile coordinate position
     */
    public void setPosition(TileCoordinate position) {
        this.currentPosition = position;
    }

    /**
     * Get the robot's current tile position.
     *
     * @return Current tile coordinate
     */
    public TileCoordinate getCurrentPosition() {
        return currentPosition;
    }

    // ==================== NAVIGATION METHODS ====================

    /**
     * Move to a specific tile coordinate position.
     *
     * Calculates movement vector from current position to target:
     * - Distance: Euclidean distance using distanceTo()
     * - Angle: atan2(dy, dx) from currentPosition.angleTo()
     * - Turn component: sin(angle - heading) × power × 0.5 (aligns robot to target)
     *
     * CRITICAL: Uses relative angle calculation to convert field-absolute angle
     * to robot-relative movement for the robot-centric drive() method.
     *
     * @param target Target tile coordinate
     * @param power  Motor power (0-1)
     */
    public void moveToPosition(TileCoordinate target, double power) {
        double distance = currentPosition.distanceTo(target);
        double angle = currentPosition.angleTo(target);
        double currentHeading = drive.getHeading();

        // Calculate turn needed to face target
        double turnAngle = angle - currentHeading;
        double turn = Math.sin(turnAngle) * power * 0.5; // 0.5 reduces turn aggressiveness

        // CRITICAL FIX: Calculate relative angle for robot-centric drive
        // The drive() method is robot-centric, so we must subtract current heading
        // to convert field-absolute angle to robot-relative movement
        double relativeAngle = angle - currentHeading;
        
        // Calculate movement components using trigonometry
        // Forward = cos(relativeAngle) × power (movement in robot's forward direction)
        // Strafe = sin(relativeAngle) × power (perpendicular movement)
        double forward = Math.cos(relativeAngle) * power;
        double strafe = Math.sin(relativeAngle) * power;

        // Apply movement via DriveSubsystem
        drive.drive(forward, strafe, turn);

        // Update position estimate (simplified - in real implementation, use odometry)
        // NOTE: Heading is NOT tracked here - DriveSubsystem IMU provides real-time heading
        currentPosition = target;

        if (telemetry != null) {
            telemetry.addData("Tile Navigation", "Moving to %s", target.getTilePosition());
            telemetry.addData("Distance", "%.1f inches", distance);
            telemetry.addData("Angle", "%.1f degrees", Math.toDegrees(angle));
            telemetry.update();
        }
    }

    /**
     * Move to a specific tile with offset.
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
     * Move to center of a specific tile.
     *
     * @param column Tile column (A-F)
     * @param row    Tile row (1-6)
     * @param power  Motor power (0-1)
     */
    public void moveToTileCenter(char column, int row, double power) {
        moveToTile(column, row, TileCoordinate.TILE_SIZE / 2, TileCoordinate.TILE_SIZE / 2, power);
    }

    /**
     * Move to a tab-line intersection.
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
     * Move relative to current position by tile units.
     *
     * Performs coordinate transformation to move relative to robot's current heading.
     * Converts relative movement (forward/right) to absolute field coordinates:
     *
     * newX = currentX + forward×cos(heading) - strafe×sin(heading)
     * newY = currentY + forward×sin(heading) + strafe×cos(heading)
     *
     * This rotation matrix accounts for robot orientation, so "forward" means
     * in the direction the robot is facing, not always field-north.
     *
     * @param tilesForward Number of tiles forward (negative for backward)
     * @param tilesRight   Number of tiles right (negative for left)
     * @param power        Motor power (0-1)
     */
    public void moveRelativeTiles(double tilesForward, double tilesRight, double power) {
        // Convert tiles to inches
        double forward = tilesForward * TileCoordinate.TILE_SIZE;
        double strafe = tilesRight * TileCoordinate.TILE_SIZE;

        // Coordinate transformation: rotate relative movement by current heading
        // Uses rotation matrix to convert robot-relative to field-absolute coordinates
        double currentHeading = drive.getHeading();
        double newX = currentPosition.getX() + forward * Math.cos(currentHeading) - strafe * Math.sin(currentHeading);
        double newY = currentPosition.getY() + forward * Math.sin(currentHeading) + strafe * Math.cos(currentHeading);

        TileCoordinate target = new TileCoordinate(newX, newY);
        moveToPosition(target, power);
    }

    /**
     * Turn to face a specific tile.
     *
     * Calculates shortest rotation angle to face target:
     * - turnAngle = targetAngle - currentHeading
     * - Normalized to [-π, π] range to ensure shortest rotation path
     * - Uses signum() to determine turn direction (positive = CCW, negative = CW)
     *
     * @param target Target tile coordinate
     * @param power  Turn power (0-1)
     */
    public void turnToTile(TileCoordinate target, double power) {
        double angle = currentPosition.angleTo(target);
        double currentHeading = drive.getHeading();
        double turnAngle = angle - currentHeading;

        // Normalize turn angle to [-π, π] range for shortest rotation
        while (turnAngle > Math.PI) turnAngle -= 2 * Math.PI;
        while (turnAngle < -Math.PI) turnAngle += 2 * Math.PI;

        // Use signum to get direction: +1 for CCW, -1 for CW
        double turn = Math.signum(turnAngle) * power;
        drive.drive(0, 0, turn);

        if (telemetry != null) {
            telemetry.addData("Turn to Tile", "Facing %s", target.getTilePosition());
            telemetry.addData("Turn Angle", "%.1f degrees", Math.toDegrees(turnAngle));
            telemetry.update();
        }
    }

    // ==================== QUERY METHODS ====================

    /**
     * Get distance to a specific tile.
     *
     * @param target Target tile coordinate
     * @return Distance in inches
     */
    public double getDistanceToTile(TileCoordinate target) {
        return currentPosition.distanceTo(target);
    }

    /**
     * Get angle to a specific tile.
     *
     * @param target Target tile coordinate
     * @return Angle in radians
     */
    public double getAngleToTile(TileCoordinate target) {
        return currentPosition.angleTo(target);
    }

    /**
     * Check if robot is at a specific tile (within tolerance).
     *
     * @param target    Target tile coordinate
     * @param tolerance Tolerance in inches
     * @return True if within tolerance
     */
    public boolean isAtTile(TileCoordinate target, double tolerance) {
        return getDistanceToTile(target) <= tolerance;
    }

    // ==================== TELEMETRY ====================

    /**
     * Update telemetry with tile-based navigation information.
     */
    public void updateTelemetry() {
        if (telemetry != null) {
            telemetry.addData("Current Tile", currentPosition.getTilePosition());
            telemetry.addData("Current Tab", currentPosition.getTabPosition());
            telemetry.addData("Position (inches)", "X: %.1f, Y: %.1f", currentPosition.getX(), currentPosition.getY());
            telemetry.addData("Heading", "%.1f degrees", Math.toDegrees(drive.getHeading()));

            double[] offset = currentPosition.getTileOffset();
            telemetry.addData("Tile Offset", "X: %.1f, Y: %.1f", offset[0], offset[1]);
        }
    }
}
