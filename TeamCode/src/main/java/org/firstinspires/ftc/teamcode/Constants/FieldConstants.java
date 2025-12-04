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

    // Goal height from floor (inches)
    // Total goal height is approximately 54 inches (137.15 cm)
    public static final double GOAL_HEIGHT_INCHES = 54.0;

    // Convenience goal centers in field coordinates (inches)
    public static final double GOAL_X_BLUE = 0.0;
    public static final double GOAL_Y_BLUE = 72.0;
    public static final double GOAL_X_RED = 144.0;
    public static final double GOAL_Y_RED = 72.0;
}


