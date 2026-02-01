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
}
