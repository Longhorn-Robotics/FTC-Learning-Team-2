package org.firstinspires.ftc.teamcode;

import android.util.Size;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous(name = "Robot Auto Final", group = "Robot")
public class RobotAutonomousFinal extends LinearOpMode {

    // --- HARDWARE ---
    private RobotHardware robot = new RobotHardware();
    private IMU imu;
    private DcMotor leftEncoder, rightEncoder;

    // --- VISION ---
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    // --- CONSTANTS ---
    // Using constants derived from previous iterations
    private static final double COUNTS_PER_INCH = 45.28;
    
    // Camera Intrinsics (Logitech C270)
    private static final double FX = 357.1;
    private static final double FY = 357.1;
    private static final double CX = 159.5;
    private static final double CY = 119.5;

    // --- LOCALIZATION MEMORY ---
    private boolean tagVisible = false;
    private double lastTagRange = 0;
    private double lastTagBearing = 0;
    private double lastTagYaw = 0;
    private double lastKnownTagRange = 0;
    private double startEncoderPos = 0;

    // --- FIELD GEOMETRY ---
    // Distance from the AprilTag (diagonal) to reach the "Lane" start point.
    private static final double LANE_ALIGNMENT_DISTANCE = 36.0; 
    
    // Y-Distances to reverse down the lane for each row.
    private double[] rowDepths = {12.0, 36.0, 60.0, 84.0}; 

    // --- STATE MACHINE ---
    private enum State {
        INIT,
        ALIGN_TO_TAG_START,
        DRIVE_TO_LANE,
        TURN_UP,
        DRIVE_TO_ROW,
        TURN_TO_BALLS,
        COLLECT_BALLS,
        REVERSE_FROM_BALLS,
        TURN_UP_RETURN,
        RETURN_TO_LANE_START,
        TURN_TO_TAG,
        DRIVE_TO_SCORE,
        SCORE,
        DONE
    }
    private State currentState = State.INIT;
    private int currentRow = 0;
    private double currentLaneDepth = 0;

    @Override
    public void runOpMode() {
        // 1. Initialize Hardware
        robot.AutoInit(hardwareMap);

        // Access encoders (ld and rd as defined in RobotHardware)
        leftEncoder = hardwareMap.get(DcMotor.class, "ld");
        rightEncoder = hardwareMap.get(DcMotor.class, "rd");
        
        // Ensure encoder directions match RobotHardware directions
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

        // 2. Initialize Vision
        initVision();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            currentState = State.ALIGN_TO_TAG_START;
        }

        while (opModeIsActive()) {
            // Update sensor data
            updateLocalization();
            
            // Comprehensive Telemetry
            updateTelemetry();

            switch (currentState) {
                case INIT:
                    break;

                case ALIGN_TO_TAG_START:
                    // Step 1: Face the AprilTag (approx -45 deg for Top-Right corner)
                    if (turnToHeading(-45)) {
                        currentState = State.DRIVE_TO_LANE;
                        resetRelativeEncoder();
                    }
                    break;

                case DRIVE_TO_LANE:
                    // Step 2: Drive to "Lane" start point (Hybrid: Visual or Blind)
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
                    // Step 3 Cont: Reverse down the lane (Blind)
                    if (driveStraight(-currentLaneDepth, 0)) {
                        currentState = State.TURN_TO_BALLS;
                    }
                    break;

                case TURN_TO_BALLS:
                    // Step 4: Turn CW to face balls (-90 deg)
                    if (turnToHeading(-90)) {
                        resetRelativeEncoder();
                        currentState = State.COLLECT_BALLS;
                    }
                    break;

                case COLLECT_BALLS:
                    // Step 4 Cont: Drive forward, intake on
                    robot.runIntake(1.0);
                    // Drive 24 inches or until collected (placeholder distance)
                    // TODO: Implement sensor-based collection stop (color/limit switch)
                    if (driveStraight(24.0, -90)) { 
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
                    // Step 5 Cont: Return to the start of the lane (Tag Anchor)
                    if (driveStraight(currentLaneDepth, 0)) {
                        currentState = State.TURN_TO_TAG;
                    }
                    break;

                case TURN_TO_TAG:
                    // Step 6: Turn CW to face Tag (-45)
                    // Hybrid turn: If tag seen, lock on? For now, just turn to heading.
                    if (turnToHeading(-45)) {
                        currentState = State.DRIVE_TO_SCORE;
                    }
                    break;

                case DRIVE_TO_SCORE:
                    // Drive closer to tag (e.g. 12 inches) to score
                    if (driveToTagHybrid(12.0, -45)) {
                         currentState = State.SCORE;
                    }
                    break;

                case SCORE:
                    robot.moveRobot(0,0);
                    // Launch sequence placeholder
                    robot.launchItems(1.0);
                    if (getRuntime() > 2.0) { // Simple timer placeholder
                         // Reset runtime logic needed or use separate timer
                    }
                    
                    // Proceed to next row
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
        }
        
        // Close vision portal when done
        visionPortal.close();
    }

    // --- TUNING CONSTANTS ---
    private static final double MAX_SPEED = 0.5;
    private static final double MAX_TURN = 0.4;
    private static final double SPEED_GAIN = 0.04;
    private static final double TURN_GAIN = 0.02;
    private static final double HEADING_THRESHOLD = 2.0; // Degrees
    private static final double DISTANCE_THRESHOLD = 1.0; // Inches

    private boolean driveToTagHybrid(double targetDistance, double targetHeading) {
        double rangeError;
        double headingError;

        if (tagVisible) {
            // Visual Navigation
            rangeError = lastTagRange - targetDistance;
            headingError = lastTagBearing;
            
            // Update anchor for blind fallback
            resetRelativeEncoder();
            lastKnownTagRange = lastTagRange;
        } else {
            // Blind Fallback (Dead Reckoning)
            // Estimated Range = Last Known Range - Distance Traveled Since Loss
            double currentEncDist = getRelativeEncoderDistance();
            double estimatedRange = lastKnownTagRange - currentEncDist;
            
            rangeError = estimatedRange - targetDistance;
            headingError = targetHeading - getHeading();
        }

        if (Math.abs(rangeError) < DISTANCE_THRESHOLD) {
            robot.moveRobot(0, 0);
            return true;
        }

        double drive = com.qualcomm.robotcore.util.Range.clip(rangeError * SPEED_GAIN, -MAX_SPEED, MAX_SPEED);
        double turn = com.qualcomm.robotcore.util.Range.clip(headingError * TURN_GAIN, -MAX_TURN, MAX_TURN);

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

        double turn = com.qualcomm.robotcore.util.Range.clip(headingError * TURN_GAIN, -MAX_TURN, MAX_TURN);
        robot.moveRobot(-turn, turn);
        return false;
    }

    private boolean driveStraight(double inches, double targetHeading) {
        double currentDist = getRelativeEncoderDistance();
        double distError = inches - currentDist;
        double headingError = targetHeading - getHeading();

        // Normalize heading error
        while (headingError > 180) headingError -= 360;
        while (headingError <= -180) headingError += 360;

        if (Math.abs(distError) < DISTANCE_THRESHOLD) {
            robot.moveRobot(0, 0);
            return true;
        }

        double drive = com.qualcomm.robotcore.util.Range.clip(distError * SPEED_GAIN, -MAX_SPEED, MAX_SPEED);
        double turn = com.qualcomm.robotcore.util.Range.clip(headingError * TURN_GAIN, -MAX_TURN, MAX_TURN);

        robot.moveRobot(drive - turn, drive + turn);
        return false;
    }

    private void updateLocalization() {
        java.util.List<org.firstinspires.ftc.vision.apriltag.AprilTagDetection> detections = aprilTag.getDetections();
        tagVisible = false;
        for (org.firstinspires.ftc.vision.apriltag.AprilTagDetection detection : detections) {
            if (detection.metadata != null) {
                tagVisible = true;
                lastTagRange = detection.ftcPose.range;
                lastTagBearing = detection.ftcPose.bearing;
                lastTagYaw = detection.ftcPose.yaw;
                lastKnownTagRange = lastTagRange;
                break;
            }
        }
    }

    private void updateTelemetry() {
        telemetry.addData("--- STATE ---", currentState);
        telemetry.addData("Tag Visible", tagVisible ? "YES" : "NO");
        if (tagVisible) {
            telemetry.addData("Tag Range", "%.2f\"", lastTagRange);
            telemetry.addData("Tag Bearing", "%.2f°", lastTagBearing);
        }
        telemetry.addData("Heading", "%.2f°", getHeading());
        telemetry.addData("Encoder Pos (L/R)", "%d / %d", 
                leftEncoder.getCurrentPosition(), rightEncoder.getCurrentPosition());
        telemetry.addData("Relative Dist", "%.2f\"", getRelativeEncoderDistance());
        telemetry.update();
    }

    private double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES);
    }

    private void resetRelativeEncoder() {
        startEncoderPos = (leftEncoder.getCurrentPosition() + rightEncoder.getCurrentPosition()) / 2.0;
    }

    private double getRelativeEncoderDistance() {
        double currentPos = (leftEncoder.getCurrentPosition() + rightEncoder.getCurrentPosition()) / 2.0;
        return (currentPos - startEncoderPos) / COUNTS_PER_INCH;
    }

    private void initVision() {
        aprilTag = new AprilTagProcessor.Builder()
                .setLensIntrinsics(FX, FY, CX, CY)
                .build();

        visionPortal = new VisionPortal.Builder()
                .addProcessor(aprilTag)
                .setCameraResolution(new Size(320, 240))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();
    }
}
