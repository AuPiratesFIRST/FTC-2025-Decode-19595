# Scoring System for DECODE Field

This directory contains the scoring subsystem that integrates with AprilTag localization to enable precise autonomous shooting into the GOAL.

## Overview

The scoring system consists of two main components:
1. **ShooterSubsystem** - Physical shooting mechanism with distance-based calibration
2. **PatternScorer** - Game logic for scoring patterns (motif matching)

## How AprilTag Integration Works

### 1. Localization → Position → Distance → Shot Parameters

```
AprilTag Detection → Robot Position → Distance to Goal → Motor Power + Angle
```

**Step-by-step process:**
1. `AprilTagNavigator.updateRobotPositionFromTriangulation()` detects tags 20/24 and calculates robot's field position
2. `ShooterSubsystem.shootArtifact(robotPosition, isBlueAlliance)` is called
3. System calculates horizontal distance from robot to goal using `TileCoordinate.distanceTo()`
4. Distance is used to interpolate motor power and launch angle from calibrated `SHOT_PROFILE`
5. Shooter motor and angle servo are set accordingly

### 2. Goal Position Constants

Goals are positioned at AprilTag locations:
- **Blue Goal (Tag 20)**: `(0, 72)` inches - left edge of field
- **Red Goal (Tag 24)**: `(144, 72)` inches - right edge of field  
- **Goal Height**: `38.75` inches from floor to top lip

### 3. Distance Calculation

```java
// Get robot position from AprilTag localization
TileCoordinate robotPos = driveSubsystem.getCurrentPosition();

// Calculate distance to appropriate goal
TileCoordinate goal = ShooterSubsystem.getGoalPosition(isBlueAlliance);
double distance = robotPos.distanceTo(goal); // Euclidean distance in inches
```

## ShooterSubsystem

### Hardware Requirements
- **Shooter Motor** (DcMotorEx): `"shooter"` - controls projectile velocity
- **Launch Angle Servo** (Servo): `"launchAngle"` - optional, controls trajectory angle

### Shot Profile Calibration

The system uses a lookup table with linear interpolation:

```java
// Format: {distance_inches, motor_power_0to1, servo_angle_0to1}
private static final double[][] SHOT_PROFILE = {
    {24, 0.45, 0.45},  // At 24", use 45% power, 45% angle
    {36, 0.60, 0.50},  // At 36", use 60% power, 50% angle
    {48, 0.70, 0.55},  // At 48", use 70% power, 55% angle
    {60, 0.80, 0.60}   // At 60", use 80% power, 60% angle
};
```

**Calibration Process:**
1. Place robot at known distances (24", 36", 48", 60") from goal
2. Adjust motor power and servo angle until consistent scoring
3. Record successful `(distance, power, angle)` combinations
4. Update `SHOT_PROFILE` array with measured values
5. System interpolates between data points for intermediate distances

### Key Methods

#### `shootArtifact(TileCoordinate robotPosition, boolean isBlueAlliance)`
Main shooting method that:
- Calculates distance to goal
- Interpolates motor power and angle from profile
- Sets hardware accordingly
- Displays telemetry

#### `calculateMotorPower(double distance, double height)`
Returns motor power (0.0-1.0) based on horizontal distance using linear interpolation.

#### `calculateLaunchAngle(double distance, double height)`  
Returns servo angle (0.0-1.0) based on horizontal distance using linear interpolation.

## PatternScorer (Game Logic)

### Motif-Based Scoring
The DECODE game uses a 3-color motif that repeats to create a 9-position pattern:

```java
// Example: Motif "GPP" creates pattern "GPPGPPGPP"
ArtifactColor[] motif = {GREEN, PURPLE, PURPLE};
PatternScorer scorer = new PatternScorer(motif);
// Pattern becomes: [G, P, P, G, P, P, G, P, P]
```

### Scoring Logic
- Compares detected ramp artifacts against the generated pattern
- Returns count of matches (0-9 points possible)
- Only scores when gate is closed

### Usage Example
```java
// Create scorer with motif
ArtifactColor[] motif = {GREEN, PURPLE, PURPLE};
PatternScorer scorer = new PatternScorer(motif);

// Score detected artifacts
ArtifactColor[] detected = {GREEN, PURPLE, PURPLE, GREEN, PURPLE, PURPLE};
int score = scorer.scorePattern(detected); // Returns 6 (perfect match for first 6)
```

## Integration with Vision System

### Complete Autonomous Flow
```java
// 1. Initialize subsystems
DriveSubsystem drive = new DriveSubsystem(hardwareMap, telemetry);
AprilTagNavigator april = new AprilTagNavigator(drive, hardwareMap, telemetry);
ShooterSubsystem shooter = new ShooterSubsystem(hardwareMap, telemetry);

// 2. Localize using AprilTags
boolean localized = april.updateRobotPositionFromTriangulation();

// 3. Drive to shooting position
drive.moveToTileCenter('C', 3, 0.5);

// 4. Shoot using current position
TileCoordinate currentPos = drive.getCurrentPosition();
shooter.shootArtifact(currentPos, isBlueAlliance);

// 5. Actuate firing mechanism (feeder/trigger)
// (This must be implemented in your hardware)
```

## Telemetry and Debugging

### Shooter Telemetry
```java
telemetry.addData("Shooter", "distance=%.1f in, power=%.2f", distance, power);
telemetry.addData("LaunchAngle", "%.2f", angle);
```

### AprilTag Telemetry
```java
april.updateDECODELocalizationTelemetry();
// Shows: detections, robot position, heading, alliance goals
```

## Hardware Configuration

### Required Hardware Names
- `"shooter"` - DcMotorEx for projectile velocity
- `"launchAngle"` - Servo for trajectory angle (optional)
- `"Webcam 1"` - Camera for AprilTag detection

### Field Setup
- AprilTags 20 and 24 positioned at goal locations
- Goals at correct height (38.75" from floor)
- Adequate lighting for camera detection
- Calibrated shot profile for your specific robot

## Troubleshooting

### Poor Shot Accuracy
- **Check**: Shot profile calibration data
- **Verify**: Goal height constant matches field setup
- **Adjust**: Add more calibration points at intermediate distances

### No AprilTag Detection
- **Check**: Camera connection and lighting
- **Verify**: Tag placement matches field constants
- **Adjust**: Detection confidence thresholds

### Distance Calculation Errors
- **Check**: AprilTag localization accuracy
- **Verify**: Goal position constants
- **Debug**: Use telemetry to monitor calculated distances

## Files in This Directory

- `ShooterSubsystem.java` - Main shooting mechanism with calibration
- `PatternScorer.java` - Game logic for motif-based scoring
- `ArtifactColor.java` - Color enumeration (GREEN/PURPLE)
- `PatternScorerTest.java` - Unit tests for pattern scoring logic

## See Also

- `../Vision/AprilTagNavigator.java` - AprilTag localization system
- `../Drive/TileCoordinate.java` - Position and distance calculations
- `../../Constants/FieldConstants.java` - Field and goal constants
- `../../AUTO/FullSystemTestOpMode.java` - Complete integration example
