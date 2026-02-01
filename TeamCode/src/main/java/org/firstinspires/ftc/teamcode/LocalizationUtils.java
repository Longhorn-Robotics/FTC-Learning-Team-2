package org.firstinspires.ftc.teamcode;

/**
 * Utility methods for field-centric localization.
 */
public class LocalizationUtils {
    // Height of the AprilTag center relative to the robot's camera center (inches)
    public static final double TAG_HEIGHT_OFFSET = 25.0;

    /**
     * Calculates the ground-level distance from the camera to the tag.
     * Projects the 3D range onto the 2D field plane.
     * @param cameraRange The 3D distance reported by the AprilTag processor.
     * @return The 2D ground distance in inches.
     */
    public static double getGroundDistance(double cameraRange) {
        if (cameraRange <= TAG_HEIGHT_OFFSET) return 0.0;
        return Math.sqrt(Math.pow(cameraRange, 2) - Math.pow(TAG_HEIGHT_OFFSET, 2));
    }

    // Field Constants
    // Based on the center of the field being 78.5 inches from the corner tags.
    // Assuming a symmetrical layout where X_tag = Y_tag = 78.5 / sqrt(2) approx 55.5
    public static final double TAG_X = 55.5; 
    public static final double TAG_Y = 55.5;

    public static final int RED_TAG_ID = 24;
    public static final int BLUE_TAG_ID = 20;

    /**
     * Calculates the robot's absolute field position based on an AprilTag detection.
     * @param tagId The ID of the detected tag.
     * @param range The 3D camera range to the tag.
     * @param bearing The bearing to the tag relative to the robot's front (degrees).
     * @param robotHeading The robot's absolute field heading (degrees, 0 = Facing Goal Wall).
     * @return A Pose object representing the robot's absolute field position (X, Y, Heading).
     */
    public static Pose calculateFieldPose(int tagId, double range, double bearing, double robotHeading) {
        double groundDist = getGroundDistance(range);
        
        // Angle to tag in field coordinates
        double phi = Math.toRadians(robotHeading + bearing);
        
        // Relative displacement from robot to tag
        double dx = groundDist * Math.sin(phi);
        double dy = groundDist * Math.cos(phi);
        
        double absoluteX, absoluteY;
        
        if (tagId == RED_TAG_ID) {
            // Tag 24 is at Top-Right (+X, +Y)
            absoluteX = TAG_X - dx;
            absoluteY = TAG_Y - dy;
        } else if (tagId == BLUE_TAG_ID) {
            // Tag 20 is at Top-Left (-X, +Y)
            absoluteX = -TAG_X - dx;
            absoluteY = TAG_Y - dy;
        } else {
            return null; // Unknown tag
        }
        
        return new Pose(absoluteX, absoluteY, robotHeading);
    }
}
