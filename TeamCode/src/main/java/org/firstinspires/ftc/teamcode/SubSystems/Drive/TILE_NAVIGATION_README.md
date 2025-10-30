# Tile-Based Navigation System

This tile-based navigation system implements the FTC Competition Manual Section 9.4 TILE Coordinates for precise field navigation.

## Overview

The system provides two coordinate systems as defined in the FTC manual:

1. **Tab-line intersections**: V-Z (columns), 1-5 (rows) - where tiles interlock
2. **Tile grid system**: A-F (columns), 1-6 (rows) - individual tile locations

## Files

- `TileCoordinate.java` - Core tile coordinate class
- `DriveSubsystem.java` - Enhanced with tile-based navigation methods
- `ExampleTileNavigation.java` - Example autonomous opmode

## Field Layout

```
Field Dimensions: 144" x 144" (6x6 tiles of 24" each)

Tile Grid (A-F, 1-6):
     A   B   C   D   E   F
6  [F6][F6][F6][F6][F6][F6]
5  [F5][F5][F5][F5][F5][F5]
4  [F4][F4][F4][F4][F4][F4]
3  [F3][F3][F3][F3][F3][F3]
2  [F2][F2][F2][F2][F2][F2]
1  [F1][F1][F1][F1][F1][F1]

Tab Intersections (V-Z, 1-5):
     V   W   X   Y   Z
5  [V5][W5][X5][Y5][Z5]
4  [V4][W4][X4][Y4][Z4]
3  [V3][W3][X3][Y3][Z3]
2  [V2][W2][X2][Y2][Z2]
1  [V1][W1][X1][Y1][Z1]
```

## Usage Examples

### Basic Tile Navigation

```java
// Initialize drive subsystem with telemetry
DriveSubsystem drive = new DriveSubsystem(hardwareMap, telemetry);

// Set starting position
drive.setPosition(new TileCoordinate('A', 1, 12, 12)); // A1 tile, 12" offset
drive.setHeading(Math.toRadians(0)); // Facing right

// Move to center of tile C3
drive.moveToTileCenter('C', 3, 0.5);

// Move to specific position within tile E2
drive.moveToTile('E', 2, 18, 6, 0.5); // 18" right, 6" up from corner

// Move to tab-line intersection
drive.moveToTabIntersection('V', 3, 0.5);
```

### Relative Movement

```java
// Move 2 tiles forward, 1 tile right
drive.moveRelativeTiles(2, 1, 0.5);

// Turn to face a specific tile
TileCoordinate target = new TileCoordinate('F', 6);
drive.turnToTile(target, 0.3);
```

### Position Tracking

```java
// Get current position
TileCoordinate current = drive.getCurrentPosition();
String tilePos = current.getTilePosition(); // "C3"
String tabPos = current.getTabPosition();   // "X3"

// Check distance and angle to target
double distance = drive.getDistanceToTile(target);
double angle = drive.getAngleToTile(target);

// Check if at specific tile (within tolerance)
boolean isAtTarget = drive.isAtTile(target, 6.0); // Within 6 inches
```

### Telemetry

```java
// Update telemetry with tile information
drive.updateTileTelemetry();
// Displays: Current Tile, Current Tab, Position (inches), Heading, Tile Offset
```

## TileCoordinate Class Methods

### Constructors
- `TileCoordinate(double x, double y)` - From field position in inches
- `TileCoordinate(int column, int row, double offsetX, double offsetY)` - From tile grid
- `TileCoordinate(char column, int row)` - From tab-line intersection

### Position Methods
- `getX()`, `getY()` - Get field position in inches
- `getTileColumn()`, `getTileRow()` - Get tile grid position (A-F, 1-6)
- `getTabColumn()`, `getTabRow()` - Get tab intersection (V-Z, 1-5)
- `getTileOffset()` - Get offset within current tile

### Navigation Methods
- `distanceTo(TileCoordinate other)` - Calculate distance to another position
- `angleTo(TileCoordinate other)` - Calculate angle to another position
- `moveToTile(char column, int row, double offsetX, double offsetY)` - Move to specific tile
- `moveToTabIntersection(char column, int row)` - Move to tab intersection

## DriveSubsystem Tile Methods

### Position Management
- `setPosition(TileCoordinate position)` - Set current position
- `setHeading(double heading)` - Set current heading
- `getCurrentPosition()` - Get current tile position
- `getCurrentHeading()` - Get current heading

### Movement Methods
- `moveToTile(char column, int row, double offsetX, double offsetY, double power)`
- `moveToTileCenter(char column, int row, double power)`
- `moveToTabIntersection(char column, int row, double power)`
- `moveToPosition(TileCoordinate target, double power)`
- `moveRelativeTiles(double tilesForward, double tilesRight, double power)`
- `turnToTile(TileCoordinate target, double power)`

### Utility Methods
- `getDistanceToTile(TileCoordinate target)`
- `getAngleToTile(TileCoordinate target)`
- `isAtTile(TileCoordinate target, double tolerance)`
- `updateTileTelemetry()`

## Integration with Pedro Pathing

This tile-based system is designed to work alongside the existing Pedro Pathing system. You can:

1. Use tile coordinates for high-level path planning
2. Convert between tile coordinates and Pedro's pose system
3. Use tile-based navigation for precise positioning
4. Combine with Pedro's path following for smooth movement

## Field Setup Reference

When setting up your field, refer to the FTC Competition Manual Section 9.4 for:
- Exact tile placement
- Tab-line intersection locations
- Field orientation and coordinate system
- Specific game element positions

## Notes

- All angles are in radians (0 = right, π/2 = up, π = left, 3π/2 = down)
- Field origin (0,0) is at bottom-left corner
- Tile size is 24" x 24"
- Position updates are simplified - for production use, integrate with proper odometry
- Power values should be between 0 and 1
