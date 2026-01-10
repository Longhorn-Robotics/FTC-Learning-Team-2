package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import android.util.Size;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.SortOrder;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "Revised Auto Control", group = "Robot")
public class RobotAutonomous3 extends LinearOpMode {

    /* --- ROBOT CONFIGURATION --- */
    // Change these to match your actual robot measurements
    final double CAMERA_HEIGHT_INCHES = 10.0; // Height of camera lens from floor
    final double CAMERA_TILT_DEGREES = 15.0;  // Angle camera is tilted DOWN from horizontal
    final double FIELD_LIMIT_INCHES = 144.0;  // Field boundary

    // PID / Gain Constants
//    final double SPEED_GAIN = 0.03;
//    final double TURN_GAIN = 0.015;
//    final double MAX_AUTO_SPEED = 0.6;
//    final double MAX_AUTO_TURN = 0.3;


    final double SPEED_GAIN =   0.02 ;   //  Speed Control "Gain". e.g. Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    //was 0.01
    final double TURN_GAIN  =   0.01 ;   //  Turn Control "Gain".  e.g. Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)


    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.25;  //  Clip the turn speed to this max value (adjust for your robot)

    /* --- STATE MACHINE ENUM --- */
    private enum State {
        INITIAL_ALIGN,
        LAUNCH_PRELOAD,
        DRIVE_TO_ROWS,
        SEARCH_FOR_BALL,
        COLLECT_BALL,
        RETURN_TO_GOAL,
        ALIGN_AND_SCORE,
        PARK,
        IDLE
    }

    private State currentState = State.INITIAL_ALIGN;

    /* --- HARDWARE & VISION --- */
    RobotHardware robot = new RobotHardware();
    private AprilTagProcessor aprilTag;
    private ColorBlobLocatorProcessor colorLocatorPurple;
    private ColorBlobLocatorProcessor colorLocatorGreen;
    private VisionPortal visionPortal;

    // Camera Constants
    private final int camW = 320;
    private final int camH = 240;
    private final double focalLengthY = 220.9;

    @Override
    public void runOpMode() {
        initVision();
        robot.init(hardwareMap);

        telemetry.addData("Status", "Ready for Launch");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Status", currentState);
            switch (currentState) {
                case INITIAL_ALIGN:
                    // Use AprilTag to zero our position relative to the goal
                    if (alignWithTag(12.0)) {
                        currentState = State.LAUNCH_PRELOAD;
                    }
                    break;

                case LAUNCH_PRELOAD:
                    stopRobot();
                    // --- INSERT YOUR LAUNCHER CODE HERE ---
                    sleep(1000);
                    currentState = State.DRIVE_TO_ROWS;
                    break;

                case DRIVE_TO_ROWS:
                    // Drive blind using encoders to the general area of the balls
                    // This prevents picking up background noise near the goal
                    driveByEncoder(-0.4, 36.0); // Drive back 36 inches
                    currentState = State.SEARCH_FOR_BALL;
                    break;

                case SEARCH_FOR_BALL:
                    if (findAndLockBall()) {
                        currentState = State.COLLECT_BALL;
                    } else {
                        // Slowly rotate to find a ball if none seen
                        robot.moveRobot(0.2, -0.2);
                    }
                    break;

                case COLLECT_BALL:
                    if (approachAndIntakeBall()) {
                        currentState = State.RETURN_TO_GOAL;
                    }
                    break;

                case RETURN_TO_GOAL:
                    // Drive back toward the goal using encoders/IMU
                    driveByEncoder(0.5, 30.0);
                    currentState = State.ALIGN_AND_SCORE;
                    break;

                case ALIGN_AND_SCORE:
                    // Re-acquire AprilTag to fix any encoder drift
                    if (alignWithTag(15.0)) {
                        // --- INSERT YOUR LAUNCHER CODE HERE ---
                        currentState = State.PARK;
                    }
                    break;

                case PARK:
                    stopRobot();
                    currentState = State.IDLE;
                    break;

                case IDLE:
                    stopRobot();
                    break;
            }
            telemetry.update();
        }
    }

    /**
     * Calculates distance to an object based on its Y-pixel coordinate (Floor-Plane Math).
     * This is much more robust against "Large Distant Objects" than radius math.
     */
    private double calculateFloorDistance(double yPixel) {
        // Normalize yPixel relative to center
        double pixelAngle = Math.toDegrees(Math.atan((yPixel - (camH/2.0)) / focalLengthY));
        double totalAngle = CAMERA_TILT_DEGREES + pixelAngle;

        if (totalAngle <= 0) return 200; // Object is at or above horizon

        return CAMERA_HEIGHT_INCHES / Math.tan(Math.toRadians(totalAngle));
    }

    private boolean findAndLockBall() {
        List<int[]> artifacts = findArtifacts();
        if (artifacts.isEmpty()) return false;

        // Get the bottom-most point of the blob for floor-plane math
        int[] bestBall = artifacts.get(0);
        double dist = calculateFloorDistance(bestBall[1] + bestBall[2]); // y + radius = bottom

        // GATE: Reject anything that doesn't make sense for a ball on the floor
        if (dist > 60.0 || dist < 2.0) return false;

        return true;
    }

    private boolean approachAndIntakeBall() {
        List<int[]> artifacts = findArtifacts();
        if (artifacts.isEmpty()) {
            // Finish movement via encoders if ball is lost (likely under intake)
            robot.moveRobot(0.3, 0.3);
            sleep(500);
            // --- INSERT YOUR INTAKE CODE HERE ---
            return true;
        }

        int[] ball = artifacts.get(0);
        double dist = calculateFloorDistance(ball[1] + ball[2]);
        double heading = (ball[0] - (camW/2.0)) * 0.1; // Simple heading error

        double drivePower = Range.clip(dist * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
        double turnPower = Range.clip(heading * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);

        robot.moveRobot(drivePower - turnPower, drivePower + turnPower);

        if (dist < 4.0) {
            // Close enough to intake
            stopRobot();
            // --- INSERT YOUR INTAKE CODE HERE ---
            return true;
        }
        return false;
    }

    private boolean alignWithTag(double targetDist) {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        AprilTagDetection target = null;

        for (AprilTagDetection d : detections) {
            if (d.metadata != null) { target = d; break; }
        }

        if (target == null) return false;

        double rangeError = target.ftcPose.range - targetDist;
        double bearingError = target.ftcPose.bearing;
        // normally range error of 1 and bearing of 2
        if (Math.abs(rangeError) < 7.0 && Math.abs(bearingError) < 10.0) {
            stopRobot();
            return true;
        }

        double driveP = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
        double turnP = Range.clip(bearingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);

        robot.moveRobot(driveP - turnP, driveP + turnP);
        return false;
    }

    private void driveByEncoder(double power, double inches) {
        // Basic encoder movement (Assumes RobotHardware has a method for this)
        // If not, you would use motor.setTargetPosition here.
        robot.moveRobot(power, power);
        sleep((long)(Math.abs(inches) * 50)); // Placeholder for actual encoder logic
        stopRobot();
    }

    private void stopRobot() {
        robot.moveRobot(0, 0);
    }

    private void initVision() {
        aprilTag = new AprilTagProcessor.Builder().build();

        colorLocatorPurple = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.ARTIFACT_PURPLE)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.5, 1, 1, -1)) // ROI: Look only at bottom half
                .setDrawContours(true)
                .setBlurSize(5)
                .build();

        colorLocatorGreen = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.ARTIFACT_GREEN)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.5, 1, 1, -1))
                .build();

        visionPortal = new VisionPortal.Builder()
                .addProcessor(colorLocatorPurple)
                .addProcessor(colorLocatorGreen)
                .addProcessor(aprilTag)
                .setCameraResolution(new Size(camW, camH))
                .setCamera(BuiltinCameraDirection.FRONT)
                .build();
    }

    public List<int[]> findArtifacts() {
        List<ColorBlobLocatorProcessor.Blob> blobs = new ArrayList<>();
        blobs.addAll(colorLocatorPurple.getBlobs());
        blobs.addAll(colorLocatorGreen.getBlobs());

        // Filter by Circularity and Size to avoid noise
        ColorBlobLocatorProcessor.Util.filterByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, 100, 10000, blobs);
        ColorBlobLocatorProcessor.Util.filterByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CIRCULARITY, 0.75, 1.0, blobs);
        ColorBlobLocatorProcessor.Util.sortByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, SortOrder.DESCENDING, blobs);

        List<int[]> results = new ArrayList<>();
        for (ColorBlobLocatorProcessor.Blob b : blobs) {
            results.add(new int[]{(int)b.getCircle().getX(), (int)b.getCircle().getY(), (int)b.getCircle().getRadius()});
        }
        return results;
    }
}