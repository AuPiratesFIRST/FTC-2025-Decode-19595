package org.firstinspires.ftc.teamcode.SubSystems.Drive;

/**
 * Tile-based coordinate system for FTC field navigation.
 * Based on FTC Competition Manual Section 9.4 TILE Coordinates.
 * 
 * Two coordinate systems are supported:
 * 1. Tab-line intersections: V-Z (columns), 1-5 (rows) - where tiles interlock
 * 2. Tile grid system: A-F (columns), 1-6 (rows) - individual tile locations
 * 
 * @author Pedro Pathing Team
 * @version 1.0
 */
public class TileCoordinate {

    // Field dimensions in inches (standard FTC field)
    public static final double FIELD_WIDTH = 144.0; // inches
    public static final double FIELD_HEIGHT = 144.0; // inches

    // Tile dimensions (6x6 grid of 24" tiles)
    public static final double TILE_SIZE = 24.0; // inches per tile
    public static final int TILES_WIDE = 6;
    public static final int TILES_TALL = 6;

    // Tab-line intersection grid (5x5 intersections)
    public static final int TAB_INTERSECTIONS_WIDE = 5;
    public static final int TAB_INTERSECTIONS_TALL = 5;

    private double x, y; // Position in inches from field origin (0,0) at bottom-left

    /**
     * Create a tile coordinate from field position in inches
     * 
     * @param x X position in inches (0-144)
     * @param y Y position in inches (0-144)
     */
    public TileCoordinate(double x, double y) {
        this.x = Math.max(0, Math.min(FIELD_WIDTH, x));
        this.y = Math.max(0, Math.min(FIELD_HEIGHT, y));
    }

    /**
     * Create a tile coordinate from tile grid position
     * 
     * @param column  Column (A=0, B=1, ..., F=5)
     * @param row     Row (1-6, where 1 is bottom)
     * @param offsetX Offset within tile (0-24 inches)
     * @param offsetY Offset within tile (0-24 inches)
     */
    public TileCoordinate(int column, int row, double offsetX, double offsetY) {
        this.x = (column * TILE_SIZE) + offsetX;
        this.y = ((row - 1) * TILE_SIZE) + offsetY;
    }

    /**
     * Create a tile coordinate from tab-line intersection
     * 
     * @param column Column (V=0, W=1, X=2, Y=3, Z=4)
     * @param row    Row (1-5, where 1 is bottom)
     */
    public TileCoordinate(char column, int row) {
        int colIndex = column - 'V';
        this.x = colIndex * TILE_SIZE;
        this.y = (row - 1) * TILE_SIZE;
    }

    // Getters
    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    /**
     * Get the tile grid column (A-F)
     * 
     * @return Column character (A-F)
     */
    public char getTileColumn() {
        int col = (int) (x / TILE_SIZE);
        return (char) ('A' + Math.max(0, Math.min(5, col)));
    }

    /**
     * Get the tile grid row (1-6)
     * 
     * @return Row number (1-6)
     */
    public int getTileRow() {
        return (int) (y / TILE_SIZE) + 1;
    }

    /**
     * Get the tab-line intersection column (V-Z)
     * 
     * @return Column character (V-Z)
     */
    public char getTabColumn() {
        int col = (int) (x / TILE_SIZE);
        return (char) ('V' + Math.max(0, Math.min(4, col)));
    }

    /**
     * Get the tab-line intersection row (1-5)
     * 
     * @return Row number (1-5)
     */
    public int getTabRow() {
        return (int) (y / TILE_SIZE) + 1;
    }

    /**
     * Get offset within current tile
     * 
     * @return Array [xOffset, yOffset] in inches
     */
    public double[] getTileOffset() {
        double xOffset = x % TILE_SIZE;
        double yOffset = y % TILE_SIZE;
        return new double[] { xOffset, yOffset };
    }

    /**
     * Calculate distance to another tile coordinate
     * 
     * @param other Other tile coordinate
     * @return Distance in inches
     */
    public double distanceTo(TileCoordinate other) {
        double dx = other.x - this.x;
        double dy = other.y - this.y;
        return Math.sqrt(dx * dx + dy * dy);
    }

    /**
     * Calculate angle to another tile coordinate
     * 
     * @param other Other tile coordinate
     * @return Angle in radians (0 = right, π/2 = up, π = left, 3π/2 = down)
     */
    public double angleTo(TileCoordinate other) {
        double dx = other.x - this.x;
        double dy = other.y - this.y;
        return Math.atan2(dy, dx);
    }

    /**
     * Move to a specific tile with offset
     * 
     * @param column  Tile column (A-F)
     * @param row     Tile row (1-6)
     * @param offsetX Offset within tile (0-24 inches)
     * @param offsetY Offset within tile (0-24 inches)
     */
    public void moveToTile(char column, int row, double offsetX, double offsetY) {
        int colIndex = column - 'A';
        this.x = (colIndex * TILE_SIZE) + Math.max(0, Math.min(TILE_SIZE, offsetX));
        this.y = ((row - 1) * TILE_SIZE) + Math.max(0, Math.min(TILE_SIZE, offsetY));
    }

    /**
     * Move to a tab-line intersection
     * 
     * @param column Tab column (V-Z)
     * @param row    Tab row (1-5)
     */
    public void moveToTabIntersection(char column, int row) {
        int colIndex = column - 'V';
        this.x = colIndex * TILE_SIZE;
        this.y = (row - 1) * TILE_SIZE;
    }

    /**
     * Get string representation of tile position
     * 
     * @return String like "A1" or "V1"
     */
    public String getTilePosition() {
        return String.format("%c%d", getTileColumn(), getTileRow());
    }

    /**
     * Get string representation of tab intersection
     * 
     * @return String like "V1" or "Z5"
     */
    public String getTabPosition() {
        return String.format("%c%d", getTabColumn(), getTabRow());
    }

    @Override
    public String toString() {
        double[] offset = getTileOffset();
        return String.format("TileCoordinate(x=%.1f, y=%.1f, tile=%c%d, offset=[%.1f,%.1f])",
                x, y, getTileColumn(), getTileRow(), offset[0], offset[1]);
    }

    @Override
    public boolean equals(Object obj) {
        if (this == obj)
            return true;
        if (obj == null || getClass() != obj.getClass())
            return false;
        TileCoordinate that = (TileCoordinate) obj;
        return Math.abs(this.x - that.x) < 0.1 && Math.abs(this.y - that.y) < 0.1;
    }

    @Override
    public int hashCode() {
        return Double.hashCode(x) + Double.hashCode(y);
    }
}
