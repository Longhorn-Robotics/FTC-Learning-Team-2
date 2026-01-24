package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import android.util.Size;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.SortOrder;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.opencv.Circle;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "Robot Auto 4 - Instructions", group = "Robot")
public class RobotAutonomous4 extends LinearOpMode {

    // --- HARDWARE ---
    // We must use RobotHardware as per instructions
    private RobotHardware robot = new RobotHardware();
    private IMU imu;
    private DcMotor leftEncoder, rightEncoder;

    // --- VISION ---
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private ColorBlobLocatorProcessor colorLocatorPurple;
    private ColorBlobLocatorProcessor colorLocatorGreen;

    // --- CONSTANTS ---
    // Calibrated from RobotAutonomous3
    private static final double COUNTS_PER_INCH = 45.28;
    private static final double MAX_SPEED = 0.5;
    private static final double MAX_TURN = 0.3;
    private static final double SPEED_GAIN = 0.03;
    private static final double TURN_GAIN = 0.02;
    private static final double HEADING_THRESHOLD = 2.0; // Degrees
    private static final double DISTANCE_THRESHOLD = 1.0; // Inches

    // Camera Intrinsics (Logitech C270 - from RobotAutonomous)
    private static final double FX = 357.1;
    private static final double FY = 357.1;
    private static final double CX = 159.5;
    private static final double CY = 119.5;

    // --- STATE MACHINE ---
    private enum State {
        INIT,
        ALIGN_TO_TAG_START,
        POSITION_FOR_ROW_START,
        TURN_UP_AND_REVERSE,
        TURN_CW_TO_BALLS,
        COLLECT_BALLS,
        REVERSE_FROM_BALLS,
        TURN_CCW_AND_RETURN,
        FACE_TAG_FINISH,
        DONE
    }

    private State currentState = State.INIT;
    private int currentRow = 0;
    // Distances to reverse down the court for each row (approximate, tune as needed)
    private double[] rowDistances = {24.0, 36.0, 48.0, 60.0};
    private double currentTargetRowDistance = 0;

    // --- LOCALIZATION MEMORY ---
    private boolean tagVisible = false;
    private double lastTagRange = 0;
    private double lastTagBearing = 0;
    private double startEncoderPos = 0; // For relative moves

    @Override
    public void runOpMode() {
        // 1. Initialize Hardware
        robot.AutoInit(hardwareMap);

        // Access encoders directly via HardwareMap since RobotHardware doesn't expose them publicly
        // This allows us to use encoders while still using the RobotHardware class for movement
        leftEncoder = hardwareMap.get(DcMotor.class, "ld");
        rightEncoder = hardwareMap.get(DcMotor.class, "rd");
        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Initialize IMU
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        imu.resetYaw();

        // Initialize Vision
        initVision();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            currentState = State.ALIGN_TO_TAG_START;
        }

        while (opModeIsActive()) {
            updateLocalization();
            telemetry.addData("State", currentState);
            telemetry.addData("Row", currentRow + 1);
            telemetry.addData("Tag Visible", tagVisible);
            telemetry.addData("Heading", "%.2f", getHeading());

            switch (currentState) {
                case ALIGN_TO_TAG_START:
                    // Step 1: Align with AprilTag
                    // Target 12 inches from tag to start
                    if (alignToTag(12.0)) {
                        currentState = State.POSITION_FOR_ROW_START;
                    }
                    break;

                case POSITION_FOR_ROW_START:
                    // Step 2: Prepare for the run.
                    // We are aligned to the tag. Now we set up variables for the specific row.
                    currentTargetRowDistance = rowDistances[currentRow];
                    resetRelativeEncoder();
                    currentState = State.TURN_UP_AND_REVERSE;
                    break;

                case TURN_UP_AND_REVERSE:
                    // Step 3: Turn facing upwards (0 deg) and reverse to ball row
                    // "Upwards" = 0 degrees. "Reverse" = moving negative relative to robot front.
                    // We use the AprilTag as an anchor implicitly by starting from it.
                    // If we lose the tag, driveStraight uses IMU/Encoders.
                    if (turnToHeading(0)) {
                        // Drive backwards X inches
                        if (driveStraight(-currentTargetRowDistance, 0)) {
                            currentState = State.TURN_CW_TO_BALLS;
                        }
                    }
                    break;

                case TURN_CW_TO_BALLS:
                    // Step 4: Turn CW until facing ball row.
                    // Facing -90 degrees (Right).
                    if (turnToHeading(-90)) {
                        resetRelativeEncoder();
                        currentState = State.COLLECT_BALLS;
                    }
                    break;

                case COLLECT_BALLS:
                    // Step 4 Cont: Drive forward to pick up balls.
                    // Use Camera for precision, IMU/Encoders for rough direction.
                    robot.runIntake(1.0); // Turn on intake
                    
                    // Drive 24 inches into the row (or until balls collected)
                    if (driveAndCollect(24.0, -90)) {
                        robot.runIntake(0); // Turn off intake
                        resetRelativeEncoder();
                        currentState = State.REVERSE_FROM_BALLS;
                    }
                    break;

                case REVERSE_FROM_BALLS:
                    // Step 5: Reverse to original point (where we turned)
                    if (driveStraight(-24.0, -90)) {
                        currentState = State.TURN_CCW_AND_RETURN;
                    }
                    break;

                case TURN_CCW_AND_RETURN:
                    // Step 5 Cont: Turn CCW (back to 0) and return to launch zone
                    if (turnToHeading(0)) {
                        // Return to the AprilTag anchor point (approx 12 inches from tag)
                        // This handles "Lost Tag" by driving blindly if needed until tag is seen
                        if (returnToTag(12.0)) {
                            currentState = State.FACE_TAG_FINISH;
                        }
                    }
                    break;

                case FACE_TAG_FINISH:
                    // Step 6: Turn CW until facing AprilTag
                    // Align to it for shooting.
                    if (alignToTag(12.0)) {
                        // TODO: Add launcher code here
                        // robot.launchItems(1.0);
                        // sleep(1000);
                        // robot.idleLauncher(0);
                        
                        currentRow++;
                        if (currentRow >= rowDistances.length) {
                            currentState = State.DONE;
                        } else {
                            currentState = State.POSITION_FOR_ROW_START;
                        }
                    }
                    break;

                case DONE:
                    robot.moveRobot(0, 0);
                    break;
            }
            telemetry.update();
        }
    }

    // --- NAVIGATION METHODS ---

    private boolean alignToTag(double targetDistance) {
        double rangeError = 0;
        double headingError = 0;

        if (tagVisible) {
            rangeError = lastTagRange - targetDistance;
            // Turn to face tag center.
            // Note: Tag is at 45 deg on wall. If we face it directly, bearing is 0.
            headingError = lastTagBearing; 
        } else {
            // If tag lost, stop. We need to see it to align.
            robot.moveRobot(0, 0);
            return false;
        }

        double drive = Range.clip(rangeError * SPEED_GAIN, -MAX_SPEED, MAX_SPEED);
        double turn = Range.clip(headingError * TURN_GAIN, -MAX_TURN, MAX_TURN);

        robot.moveRobot(drive - turn, drive + turn);

        return (Math.abs(rangeError) < DISTANCE_THRESHOLD && Math.abs(headingError) < HEADING_THRESHOLD);
    }

    private boolean returnToTag(double targetDistance) {
        // Handles the "Lost Tag" scenario by driving blindly towards where we think it is (Forward)
        double rangeError;
        double headingError;

        if (tagVisible) {
            rangeError = lastTagRange - targetDistance;
            headingError = lastTagBearing;
        } else {
            // Dead reckoning: We assume we are facing 0 (Up) and need to drive forward
            // We set a fake range error to encourage forward movement
            rangeError = 10.0; 
            headingError = 0 - getHeading(); // Maintain 0 heading
        }

        double drive = Range.clip(rangeError * SPEED_GAIN, -MAX_SPEED, MAX_SPEED);
        double turn = Range.clip(headingError * TURN_GAIN, -MAX_TURN, MAX_TURN);

        robot.moveRobot(drive - turn, drive + turn);

        // Only return true if we actually see the tag AND are close enough
        return (tagVisible && Math.abs(lastTagRange - targetDistance) < DISTANCE_THRESHOLD);
    }

    private boolean driveStraight(double inches, double targetHeading) {
        double currentDist = getRelativeEncoderDistance();
        double distError = inches - currentDist;
        double headingError = targetHeading - getHeading();

        if (Math.abs(distError) < DISTANCE_THRESHOLD) {
            robot.moveRobot(0, 0);
            return true;
        }

        double drive = Range.clip(distError * SPEED_GAIN, -MAX_SPEED, MAX_SPEED);
        double turn = Range.clip(headingError * TURN_GAIN, -MAX_TURN, MAX_TURN);

        robot.moveRobot(drive - turn, drive + turn);
        return false;
    }

    private boolean turnToHeading(double targetHeading) {
        double headingError = targetHeading - getHeading();

        if (Math.abs(headingError) < HEADING_THRESHOLD) {
            robot.moveRobot(0, 0);
            return true;
        }

        double turn = Range.clip(headingError * TURN_GAIN, -MAX_TURN, MAX_TURN);
        robot.moveRobot(-turn, turn);
        return false;
    }

    private boolean driveAndCollect(double targetInches, double targetHeading) {
        // Use ColorBlob for precision steering
        double turnCorrection = 0;
        List<int[]> artifacts = findArtifacts();
        
        if (!artifacts.isEmpty()) {
            // Found a ball, steer towards it
            int[] closestBall = artifacts.get(0);
            // Calculate angle to ball
            double ballAngle = calculateBallAngle(closestBall[0]);
            turnCorrection = Range.clip(ballAngle * TURN_GAIN, -MAX_TURN, MAX_TURN);
            telemetry.addData("Ball Found", "Angle: %.2f", ballAngle);
        } else {
            // No ball, use IMU to maintain heading
            double headingError = targetHeading - getHeading();
            turnCorrection = Range.clip(headingError * TURN_GAIN, -MAX_TURN, MAX_TURN);
        }

        double currentDist = getRelativeEncoderDistance();
        double distError = targetInches - currentDist;

        if (Math.abs(distError) < DISTANCE_THRESHOLD) {
            robot.moveRobot(0, 0);
            return true;
        }

        double drive = Range.clip(distError * SPEED_GAIN, -MAX_SPEED, MAX_SPEED);
        robot.moveRobot(drive - turnCorrection, drive + turnCorrection);
        return false;
    }

    // --- VISION HELPERS ---

    private void initVision() {
        aprilTag = new AprilTagProcessor.Builder()
                .setLensIntrinsics(FX, FY, CX, CY)
                .build();

        colorLocatorPurple = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.ARTIFACT_PURPLE)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.asUnityCenterCoordinates(-1, 1, 1, -1))
                .setDrawContours(true)
                .setBlurSize(5)
                .build();

        colorLocatorGreen = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.ARTIFACT_GREEN)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.asUnityCenterCoordinates(-1, 1, 1, -1))
                .setDrawContours(true)
                .setBlurSize(5)
                .build();

        visionPortal = new VisionPortal.Builder()
                .addProcessor(aprilTag)
                .addProcessor(colorLocatorPurple)
                .addProcessor(colorLocatorGreen)
                .setCameraResolution(new Size(320, 240))
                .setCamera(hardwareMap.get(WebcamName.class, "cameraa"))
                .build();
    }

    private void updateLocalization() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        tagVisible = false;
        for (AprilTagDetection detection : detections) {
            if (detection.metadata != null) {
                // Use any tag or specific ID
                tagVisible = true;
                lastTagRange = detection.ftcPose.range;
                lastTagBearing = detection.ftcPose.bearing;
                break;
            }
        }
    }

    private List<int[]> findArtifacts() {
        List<ColorBlobLocatorProcessor.Blob> blobs = new ArrayList<>();
        blobs.addAll(colorLocatorPurple.getBlobs());
        blobs.addAll(colorLocatorGreen.getBlobs());

        ColorBlobLocatorProcessor.Util.filterByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, 50, 20000, blobs);
        ColorBlobLocatorProcessor.Util.sortByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, SortOrder.DESCENDING, blobs);

        List<int[]> results = new ArrayList<>();
        for (ColorBlobLocatorProcessor.Blob blob : blobs) {
            Circle c = blob.getCircle();
            results.add(new int[]{(int)c.getX(), (int)c.getY(), (int)c.getRadius()});
        }
        return results;
    }

    private double calculateBallAngle(double xPixel) {
        return Math.toDegrees(Math.atan((xPixel - CX) / FX));
    }

    // --- SENSOR HELPERS ---

    private double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    private void resetRelativeEncoder() {
        startEncoderPos = (leftEncoder.getCurrentPosition() + rightEncoder.getCurrentPosition()) / 2.0;
    }

    private double getRelativeEncoderDistance() {
        double currentPos = (leftEncoder.getCurrentPosition() + rightEncoder.getCurrentPosition()) / 2.0;
        return (currentPos - startEncoderPos) / COUNTS_PER_INCH;
    }
}