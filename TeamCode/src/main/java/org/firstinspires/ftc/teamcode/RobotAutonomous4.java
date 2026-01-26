package org.firstinspires.ftc.teamcode;

import android.util.Size;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.SortOrder;

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

@Autonomous(name = "Robot Auto 4 - Final", group = "Robot")
public class RobotAutonomous4 extends LinearOpMode {

    // --- HARDWARE ---
    private RobotHardware robot = new RobotHardware();
    private IMU imu;
    private DcMotor leftEncoder, rightEncoder;

    // --- VISION ---
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private ColorBlobLocatorProcessor colorLocatorPurple;
    private ColorBlobLocatorProcessor colorLocatorGreen;

    // --- TUNING CONSTANTS ---
    private static final double COUNTS_PER_INCH = 45.28; 
    private static final double MAX_SPEED = 0.5;
    private static final double MAX_TURN = 0.4;
    private static final double SPEED_GAIN = 0.04;
    private static final double TURN_GAIN = 0.02;
    private static final double HEADING_THRESHOLD = 2.0; // Degrees
    private static final double DISTANCE_THRESHOLD = 1.0; // Inches
    
    // --- FIELD GEOMETRY ---
    // Distance from the AprilTag (diagonal) to reach the "Lane" start point.
    private static final double LANE_ALIGNMENT_DISTANCE = 36.0; 
    
    // Y-Distances to reverse down the lane for each row.
    private double[] rowDepths = {12.0, 36.0, 60.0, 84.0}; 

    // Camera Intrinsics (Logitech C270)
    private static final double FX = 357.1;
    private static final double FY = 357.1;
    private static final double CX = 159.5;
    private static final double CY = 119.5;

    // --- STATE MACHINE ---
    private enum State {
        INIT,
        ALIGN_TO_TAG_START,    // Face the tag initially
        DRIVE_TO_LANE,         // Move away/towards tag to reach alignment lane (Hybrid)
        TURN_UP,               // Turn to 0 degrees (Up)
        DRIVE_TO_ROW,          // Reverse down the lane (Blind)
        TURN_TO_BALLS,         // Turn CW (-90) to face balls
        COLLECT_BALLS,         // Drive forward and intake
        REVERSE_FROM_BALLS,    // Reverse back to the lane
        TURN_UP_RETURN,        // Turn back to 0 degrees
        RETURN_TO_LANE_START,  // Drive forward (Up) to the start of the lane (Hybrid)
        TURN_TO_TAG,           // Turn to face the tag (Scan)
        DRIVE_TO_SCORE,        // Drive closer to tag for scoring
        SCORE,                 // Launch balls
        DONE
    }

    private State currentState = State.INIT;
    private int currentRow = 0;
    private double currentLaneDepth = 0; // Target depth for current row

    // --- LOCALIZATION MEMORY ---
    private boolean tagVisible = false;
    private double lastKnownTagRange = 0;
    private double lastKnownTagBearing = 0;
    
    // Encoders
    private double startEncoderPos = 0; // For relative moves
    private double lastKnownTagEncoderPos = 0; // Encoder value when tag was last seen

    @Override
    public void runOpMode() {
        // 1. Initialize Hardware
        robot.AutoInit(hardwareMap);

        // Access encoders matching RobotHardware direction
        leftEncoder = hardwareMap.get(DcMotor.class, "ld");
        rightEncoder = hardwareMap.get(DcMotor.class, "rd");
        leftEncoder.setDirection(DcMotor.Direction.REVERSE);
        rightEncoder.setDirection(DcMotor.Direction.FORWARD);
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
            
            // --- TELEMETRY ---
            telemetry.addData("State", currentState);
            telemetry.addData("Row", currentRow + 1);
            telemetry.addData("Tag Visible", tagVisible ? "YES" : "NO");
            telemetry.addData("Tag Range", "%.1f", lastKnownTagRange);
            telemetry.addData("Heading", "%.1f", getHeading());

            switch (currentState) {
                case ALIGN_TO_TAG_START:
                    // Step 1: Face the AprilTag (approx -45 deg for Top-Right corner)
                    // We assume start is roughly aligned, so just get the heading right first
                    if (turnToHeading(-45)) {
                        currentState = State.DRIVE_TO_LANE;
                        // Reset localization anchor for the next step
                        resetRelativeEncoder(); 
                    }
                    break;

                case DRIVE_TO_LANE:
                    // Step 2: Drive backwards/forwards to reach the "Lane"
                    // Use Hybrid method: Tag if visible, else encoders relative to last tag
                    if (driveToTagHybrid(LANE_ALIGNMENT_DISTANCE, -45)) {
                        currentState = State.TURN_UP;
                    }
                    break;

                case TURN_UP:
                    // Step 3: Turn to face Up (0 deg)
                    if (turnToHeading(0)) {
                        resetRelativeEncoder();
                        currentLaneDepth = rowDepths[currentRow];
                        currentState = State.DRIVE_TO_ROW;
                    }
                    break;

                case DRIVE_TO_ROW:
                    // Step 3 Cont: Reverse down the lane (Blind using Encoders/IMU)
                    // Move negative distance (Reverse)
                    if (driveStraight(-currentLaneDepth, 0)) {
                        currentState = State.TURN_TO_BALLS;
                    }
                    break;

                case TURN_TO_BALLS:
                    // Step 4: Turn CW to face balls.
                    if (turnToHeading(-90)) {
                        resetRelativeEncoder();
                        currentState = State.COLLECT_BALLS;
                    }
                    break;

                case COLLECT_BALLS:
                    // Step 4 Cont: Drive forward, intake on
                    robot.runIntake(1.0);
                    // Drive 24 inches or until collected
                    if (driveAndCollect(24.0, -90)) {
                        robot.runIntake(0);
                        resetRelativeEncoder();
                        currentState = State.REVERSE_FROM_BALLS;
                    }
                    break;

                case REVERSE_FROM_BALLS:
                    // Step 5: Reverse back to the lane point
                    if (driveStraight(-24.0, -90)) {
                        currentState = State.TURN_UP_RETURN;
                    }
                    break;

                case TURN_UP_RETURN:
                    // Step 5 Cont: Turn CCW back to Up (0 deg)
                    if (turnToHeading(0)) {
                        resetRelativeEncoder();
                        currentState = State.RETURN_TO_LANE_START;
                    }
                    break;

                case RETURN_TO_LANE_START:
                    // Step 5 Cont: Return to the start of the lane
                    // We moved -currentLaneDepth down, so move +currentLaneDepth up.
                    // IMPORTANT: If we see the tag, we can switch to Tag Tracking?
                    // For now, rely on strict distance to get back to the "Lane Start" 
                    // which is LANE_ALIGNMENT_DISTANCE from tag.
                    if (driveStraight(currentLaneDepth, 0)) {
                        currentState = State.TURN_TO_TAG;
                    }
                    break;

                case TURN_TO_TAG:
                    // Step 6: Turn CW until facing AprilTag
                    // If tag becomes visible, we can stop turning and lock on.
                    boolean turned = turnToHeading(-45);
                    if (tagVisible || turned) {
                         currentState = State.DRIVE_TO_SCORE;
                    }
                    break;
                    
                case DRIVE_TO_SCORE:
                    // Drive closer to tag (e.g. 12 inches) to score
                    // Again, use Hybrid to be robust
                    if (driveToTagHybrid(12.0, -45)) {
                         currentState = State.SCORE;
                    }
                    break;

                case SCORE:
                    robot.moveRobot(0,0);
                    telemetry.addData("Action", "LAUNCHING!");
                    telemetry.update();
                    
                    // --- LAUNCHER CODE PLACEHOLDER ---
                    // robot.launchItems(1.0);
                    // sleep(1000);
                    // robot.idleLauncher(0);
                    // ---------------------------------

                    currentRow++;
                    if (currentRow >= rowDepths.length) {
                        currentState = State.DONE;
                    } else {
                        currentState = State.DRIVE_TO_LANE; 
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

    private boolean driveToTagHybrid(double targetDistance, double targetHeading) {
        // Robust Navigation: Uses AprilTag if visible, Dead Reckoning if not.
        
        double rangeError;
        double headingError;

        if (tagVisible) {
            // -- VISUAL NAVIGATION --
            // Error = Current - Target. 
            // If current 12, target 36, error -24. We want to be farther.
            // Move backward. -Power. Correct.
            rangeError = lastKnownTagRange - targetDistance;
            headingError = lastKnownTagBearing;
            
            // Update "Anchor" for blind fallback
            resetRelativeEncoder(); 
            lastKnownTagEncoderPos = 0; // We just reset it
        } else {
            // -- BLIND FALLBACK --
            // We assume we were at 'lastKnownTagRange' when we lost visual.
            // Encoder Distance: Positive = Forward (decreasing range to tag), Negative = Backward (increasing range)
            // Current Est Range = LastRange - EncoderDistance
            // Example: Last 12. Back up 5 inches (Enc = -5). Est Range = 12 - (-5) = 17. Correct.
            
            double currentEncDist = getRelativeEncoderDistance();
            double estimatedRange = lastKnownTagRange - currentEncDist;
            
            rangeError = estimatedRange - targetDistance;
            
            // Blind Heading
            headingError = targetHeading - getHeading();
            
            telemetry.addData("Mode", "BLIND RECKONING");
            telemetry.addData("Est Range", "%.1f", estimatedRange);
        }

        if (Math.abs(rangeError) < DISTANCE_THRESHOLD) {
            robot.moveRobot(0, 0);
            return true;
        }

        double drive = Range.clip(rangeError * SPEED_GAIN, -MAX_SPEED, MAX_SPEED);
        double turn = Range.clip(headingError * TURN_GAIN, -MAX_TURN, MAX_TURN);

        // If facing tag, Forward drive reduces range. 
        // Logic check: rangeError = (Current - Target). 
        // If Current > Target (Too far), Error > 0. Drive > 0. Forward. Reduces Range. Correct.
        robot.moveRobot(drive - turn, drive + turn);
        
        return false;
    }

    private boolean driveStraight(double inches, double targetHeading) {
        // Blind driving using Encoders + IMU
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

        // Normalize error to -180 to 180
        while (headingError > 180) headingError -= 360;
        while (headingError <= -180) headingError += 360;

        if (Math.abs(headingError) < HEADING_THRESHOLD) {
            robot.moveRobot(0, 0);
            return true;
        }

        double turn = Range.clip(headingError * TURN_GAIN, -MAX_TURN, MAX_TURN);
        robot.moveRobot(-turn, turn);
        return false;
    }

    private boolean driveAndCollect(double targetInches, double targetHeading) {
        // Use ColorBlob for precision steering towards balls
        double turnCorrection = 0;
        List<int[]> artifacts = findArtifacts();
        
        if (!artifacts.isEmpty()) {
            // Found a ball, steer towards it
            int[] closestBall = artifacts.get(0);
            double ballAngle = calculateBallAngle(closestBall[0]);
            turnCorrection = Range.clip(ballAngle * TURN_GAIN, -MAX_TURN, MAX_TURN);
            telemetry.addData("Ball Tracking", "Angle: %.2f", ballAngle);
        } else {
            // No ball, use IMU
            double headingError = targetHeading - getHeading();
            while (headingError > 180) headingError -= 360;
            while (headingError <= -180) headingError += 360;
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
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();
    }

    private void updateLocalization() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        tagVisible = false;
        for (AprilTagDetection detection : detections) {
            if (detection.metadata != null) {
                tagVisible = true;
                lastKnownTagRange = detection.ftcPose.range;
                lastKnownTagBearing = detection.ftcPose.bearing;
                // Note: We do NOT reset encoders here. We reset them explicitly when state changes or when hybrid lock occurs.
                // Actually, hybrid method resets them when tag IS visible to keep anchor fresh.
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