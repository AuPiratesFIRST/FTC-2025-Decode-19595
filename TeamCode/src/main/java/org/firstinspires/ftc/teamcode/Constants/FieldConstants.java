package org.firstinspires.ftc.teamcode.Constants;

/**
 * Centralized field- and game-specific constants.
 */
public final class FieldConstants {

    private FieldConstants() {}

    // Field dimensions
    public static final double FIELD_WIDTH_INCHES = 144.0;
    public static final double FIELD_HEIGHT_INCHES = 144.0;
    public static final double TILE_SIZE_INCHES = 24.0;

    // AprilTag specifications (DECODE field per manual Section 9.10)
    public static final double APRILTAG_SIZE_INCHES = 8.125;
    public static final double APRILTAG_SIZE_CM = 20.65;

    // Alliance goal tag positions: {tagId, x_inches, y_inches, heading_degrees}
    public static final double[][] APRILTAG_POSITIONS = new double[][]{
        {20, 0.0, 72.0, 0.0},
        {24, 144.0, 72.0, 180.0}
        // Note: 21/22/23 are on the obelisk outside the field
    };

    // Goal height from floor (inches) - Peak of the roof
    // Total goal height is approximately 54 inches (137.15 cm)
    public static final double GOAL_HEIGHT_INCHES = 54.0;

    // Goal lip height (inches) - Bottom of the opening you must shoot over
    // Per DECODE Manual Figure 9-9, page 110
    public static final double GOAL_LIP_HEIGHT = 38.75;

    // Scoring target height (inches) - Aim for middle of goal opening
    // Approximately 7.5" above lip for consistent scoring
    public static final double SCORING_TARGET_HEIGHT = 46.0;

    // Convenience goal centers in field coordinates (inches)
    // Blue Goal is on the LEFT wall (X=0)
    public static final double GOAL_X_BLUE = 0.0;
    public static final double GOAL_Y_BLUE = 72.0;

    // Red Goal is on the RIGHT wall (X=144)
    public static final double GOAL_X_RED = 144.0;
    public static final double GOAL_Y_RED = 72.0;

    // ==================== DECODE STARTING POSITIONS ====================
    // Robot starts parallel to walls, facing downfield (away from driver/audience)
    // Initial heading: 90 degrees (facing toward back wall)
    //
    // CRITICAL: This is the "blind" start position. The robot must turn to face
    // the goal before attempting AprilTag alignment, otherwise the goal will be
    // outside the camera's 60-degree field of view.

    // Red Alliance Start Position: Tile D1
    // Robot facing: 90 degrees (straight away from driver)
    public static final double START_X_RED = 84.0;      // Center of tile D (D=column 4, x=4*24-12=84")
    public static final double START_Y_RED = 12.0;      // Front edge of tile 1 + 9" to robot center
    public static final double START_HEADING_DEGREES = 90.0; // Facing away from driver

    // Blue Alliance Start Position: Tile C1
    // Robot facing: 90 degrees (straight away from driver)
    public static final double START_X_BLUE = 60.0;     // Center of tile C (C=column 3, x=3*24-12=60")
    public static final double START_Y_BLUE = 12.0;     // Front edge of tile 1 + 9" to robot center
    // Same heading as Red (90 degrees)

    // ==================== AUTONOMOUS INITIALIZATION GUIDE ====================
    // 
    // "BLIND START" PROBLEM:
    // - Robot starts parallel to walls, facing 90° (away from driver)
    // - Goal is at 45° angle from robot's perspective
    // - Webcam has only ~60° field of view
    // - Result: AprilTag is OUTSIDE the camera's view at startup
    //
    // SOLUTION: "Turn, Then Look" Strategy
    // 1. Initialize robot position with START_X/Y and START_HEADING_DEGREES
    // 2. Perform blind turn (using IMU) to face goal (approximately 45° for 9" offset)
    // 3. THEN attempt AprilTag alignment/vision lock
    //
    // IMPLEMENTATION EXAMPLE:
    // --------
    // In your autonomous runOpMode():
    //
    //   // STEP 1: Initialize with blind start position
    //   boolean isRedAlliance = true; // from alliance selector
    //   TileCoordinate startPos;
    //   double startHeading;
    //
    //   if (isRedAlliance) {
    //       startPos = new TileCoordinate(START_X_RED, START_Y_RED);
    //       startHeading = Math.toRadians(START_HEADING_DEGREES);
    //   } else {
    //       startPos = new TileCoordinate(START_X_BLUE, START_Y_BLUE);
    //       startHeading = Math.toRadians(START_HEADING_DEGREES);
    //   }
    //   
    //   driveSubsystem.setPosition(startPos);
    //   driveSubsystem.setHeading(startHeading);
    //
    //   waitForStart();
    //
    //   // STEP 2: Turn to face the goal (blind turn using IMU)
    //   TileCoordinate goalLocation;
    //   if (isRedAlliance) {
    //       goalLocation = new TileCoordinate(GOAL_X_RED, GOAL_Y_RED);
    //   } else {
    //       goalLocation = new TileCoordinate(GOAL_X_BLUE, GOAL_Y_BLUE);
    //   }
    //
    //   // This calculates: angle = atan2(goalY - robotY, goalX - robotX)
    //   // For Red at D1: atan2(72-12, 144-84) = atan2(60, 60) ≈ 45°
    //   navigator.turnToTile(goalLocation, 0.5); // Power = 0.5
    //
    //   // STEP 3: Now the goal should be in view. Lock on with vision!
    //   if (vision.updateRobotPosition()) {
    //       telemetry.addData("Status", "Locked on Goal!");
    //       // Proceed with aiming and shooting
    //   } else {
    //       telemetry.addData("Status", "Vision lock failed");
    //       // Use dead reckoning as fallback
    //   }
    // --------
}


