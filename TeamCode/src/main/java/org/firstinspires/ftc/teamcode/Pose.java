package org.firstinspires.ftc.teamcode;

/**
 * Represents a field-centric position and orientation (Pose).
 */
public class Pose {
    public double x;
    public double y;
    public double heading; // Degrees

    public Pose(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public Pose() {
        this(0, 0, 0);
    }

    @Override
    public String toString() {
        return String.format("(X: %.1f, Y: %.1f, H: %.1f°)", x, y, heading);
    }
}
