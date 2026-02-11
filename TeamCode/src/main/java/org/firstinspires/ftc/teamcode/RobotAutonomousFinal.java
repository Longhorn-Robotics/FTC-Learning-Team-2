package org.firstinspires.ftc.teamcode;

import android.util.Size;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name = "Robot Auto Final - Sequential", group = "Robot")
public class RobotAutonomousFinal extends LinearOpMode {

    // --- HARDWARE ---
    private RobotHardware robot = new RobotHardware();
    private IMU imu;
    private DcMotor leftEncoder, rightEncoder;

    // --- VISION ---
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    // --- CONSTANTS ---
    private static final double COUNTS_PER_INCH = 45.28;
    
    // Camera Intrinsics (Logitech C270)
    private static final double FX = 357.1;
    private static final double FY = 357.1;
    private static final double CX = 159.5;
    private static final double CY = 119.5;

    // --- TUNING CONSTANTS ---
    private static final double MAX_SPEED = 0.4; 
    private static final double MAX_TURN = 0.25; 
    private static final double SPEED_GAIN = 0.03;
    private static final double TURN_GAIN = 0.015; 
    private static final double HEADING_THRESHOLD = 1.0; 
    private static final double DISTANCE_THRESHOLD = 1.0; 

    // --- FIELD CONFIGURATION (CW Positive) ---
    private static final int TARGET_TAG_ID = 24; 
    private static final double HEADING_TAG = 45.0;   // Facing Corner
    private static final double HEADING_UP = 0.0;     // Facing Goals
    private static final double HEADING_RIGHT = 90.0; // Facing Ball Rows

    // --- ROUTING DISTANCES ---
    private static final double DIST_TO_LANE = 36.0;  // Diagonal distance to lane start
    private static final double DIST_COLLECT = 24.0;  // Drive into row
    private static final double DIST_SCORE = 12.0;    // Final approach
    
    // Row Depths (Relative to Lane Start)
    private double[] rowDepths = {12.0, 36.0, 60.0, 84.0}; 

    // --- LOCALIZATION MEMORY ---
    private boolean tagVisible = false;
    private double lastTagRange = 0;
    private double lastTagBearing = 0;
    private double headingOffset = 0; 
    private double startEncoderPos = 0; 

    // --- STATE MACHINE ---
    private enum State {
        INIT,
        CALIBRATE_HEADING,
        ALIGN_TO_TAG,
        DRIVE_TO_LANE,
        TURN_UP,
        REVERSE_TO_ROW,
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

        leftEncoder = hardwareMap.get(DcMotor.class, "ld");
        rightEncoder = hardwareMap.get(DcMotor.class, "rd");
        
        // Force Encoders to count UP when moving forward (physically reversed motors)
        // Since we send negative power to go forward, encoders might count down.
        // Let's set direction to match physical forward motion if possible.
        // Or handle it in getRelativeEncoderDistance logic.
        // Based on test: "Heading 0, Encoder 12" -> Correct logic was achieved.
        leftEncoder.setDirection(DcMotor.Direction.REVERSE);
        rightEncoder.setDirection(DcMotor.Direction.FORWARD);
        
        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));

        initVision();

        telemetry.addData("Status", "Initialized - Sequential");
        telemetry.update();

        waitForStart();

        ElapsedTime autoTimer = new ElapsedTime();
        autoTimer.reset();

        if (opModeIsActive()) {
            currentState = State.CALIBRATE_HEADING;
        }

        while (opModeIsActive()) {
            updateLocalization();
            updateTelemetry();

            // Safety Timer (25s)
            if (autoTimer.seconds() > 25.0 && currentState != State.DONE) {
                // Emergency Exit: Just stop or drive forward?
                // Spec says "exit launch zone".
                currentState = State.DONE; 
            }

            switch (currentState) {
                case INIT:
                    break;

                case CALIBRATE_HEADING:
                    // Wait for Tag 24
                    if (tagVisible) {
                        double currentRawHeading = getRawHeading();
                        // FieldHeading = RawHeading + Offset
                        // We are facing roughly 45 deg.
                        // Precise: FieldHeading = TAG_FIELD_HEADING - Bearing
                        double fieldHeading = TAG_FIELD_HEADING - lastTagBearing;
                        headingOffset = fieldHeading - currentRawHeading;
                        currentState = State.ALIGN_TO_TAG;
                    } else if (autoTimer.seconds() > 3.0) {
                        // Fallback: Assume start is exactly 45.0
                        headingOffset = 45.0 - getRawHeading();
                        currentState = State.ALIGN_TO_TAG;
                    }
                    break;

                case ALIGN_TO_TAG:
                    // 1. Turn to face tag perfectly (45 deg)
                    if (turnTo(HEADING_TAG)) {
                        resetRelativeEncoder();
                        currentState = State.DRIVE_TO_LANE;
                    }
                    break;

                case DRIVE_TO_LANE:
                    // 2. Drive diagonal to lane start
                    // Use Hybrid visual distance if possible?
                    // For now, use relative encoder distance for robustness.
                    // If visual available, we could update target.
                    // Let's stick to pure Sequential Drive.
                    if (driveStraight(DIST_TO_LANE, HEADING_TAG)) {
                        currentState = State.TURN_UP;
                    }
                    break;

                case TURN_UP:
                    // 3. Turn to face Up (0 deg)
                    if (turnTo(HEADING_UP)) {
                        currentLaneDepth = rowDepths[currentRow];
                        resetRelativeEncoder();
                        currentState = State.REVERSE_TO_ROW;
                    }
                    break;

                case REVERSE_TO_ROW:
                    // 4. Reverse down lane
                    // Target is negative (backward) relative to current position
                    if (driveStraight(-currentLaneDepth, HEADING_UP)) {
                        currentState = State.TURN_TO_BALLS;
                    }
                    break;

                case TURN_TO_BALLS:
                    // 5. Turn to face Right (90 deg)
                    if (turnTo(HEADING_RIGHT)) {
                        resetRelativeEncoder();
                        currentState = State.COLLECT_BALLS;
                    }
                    break;

                case COLLECT_BALLS:
                    // 6. Intake Run
                    robot.runIntake(1.0);
                    if (driveStraight(DIST_COLLECT, HEADING_RIGHT)) {
                        robot.runIntake(0);
                        currentState = State.REVERSE_FROM_BALLS;
                    }
                    break;

                case REVERSE_FROM_BALLS:
                    // 7. Reverse out (Back to 0 distance relative to turn point)
                    // We drove +DIST_COLLECT, now drive back to 0 (effectively -DIST_COLLECT distance)
                    // Wait, driveStraight takes RELATIVE target from RESET point?
                    // No, `driveStraight` calculates `distError = target - current`.
                    // We didn't reset encoders after COLLECT.
                    // So current is DIST_COLLECT. Target is 0. Error is -DIST_COLLECT.
                    // Robot will drive backward. Correct.
                    if (driveStraight(0, HEADING_RIGHT)) {
                        currentState = State.TURN_UP_RETURN;
                    }
                    break;

                case TURN_UP_RETURN:
                    // 8. Turn Up
                    if (turnTo(HEADING_UP)) {
                        // We are at depth -currentLaneDepth relative to Lane Start
                        // But we reset encoders at Turn Up? No.
                        // We need to re-sync our "Lane Start" reference.
                        // Simplest: Reset encoders here. Target is +currentLaneDepth to go forward.
                        resetRelativeEncoder();
                        currentState = State.RETURN_TO_LANE_START;
                    }
                    break;

                case RETURN_TO_LANE_START:
                    // 9. Return to Lane Start
                    if (driveStraight(currentLaneDepth, HEADING_UP)) {
                        currentState = State.TURN_TO_TAG;
                    }
                    break;

                case TURN_TO_TAG:
                    // 10. Turn to Tag
                    if (turnTo(HEADING_TAG)) {
                        resetRelativeEncoder();
                        currentState = State.DRIVE_TO_SCORE;
                    }
                    break;

                case DRIVE_TO_SCORE:
                    // 11. Drive to Score
                    if (driveStraight(DIST_SCORE, HEADING_TAG)) {
                        currentState = State.SCORE;
                    }
                    break;

                case SCORE:
                    moveRobot(0,0);
                    // Launch
                    robot.launchItems(1.0);
                    sleep(1500);
                    robot.launchItems(0);
                    
                    currentRow++;
                    if (currentRow >= 2) { 
                        currentState = State.DONE;
                    } else {
                        currentState = State.ALIGN_TO_TAG; 
                    }
                    break;

                case DONE:
                    moveRobot(0, 0);
                    break;
            }
        }
        visionPortal.close();
    }

    private boolean turnTo(double targetHeading) {
        double currentHeading = getFieldHeading();
        double headingError = currentHeading - targetHeading; // Inverted for Mixer

        while (headingError > 180) headingError -= 360;
        while (headingError <= -180) headingError += 360;

        telemetry.addData("SEQ Turn", "Targ: %.1f, Err: %.1f", targetHeading, headingError);

        if (Math.abs(headingError) < HEADING_THRESHOLD) {
            moveRobot(0, 0);
            return true;
        }

        double turnPower = Range.clip(headingError * TURN_GAIN, -MAX_TURN, MAX_TURN);
        moveRobot(0, turnPower);
        return false;
    }

    private boolean driveStraight(double targetInches, double targetHeading) {
        double currentDist = getRelativeEncoderDistance();
        double distError = targetInches - currentDist;
        
        double currentHeading = getFieldHeading();
        double headingError = targetHeading - currentHeading; // Normal for Correction

        while (headingError > 180) headingError -= 360;
        while (headingError <= -180) headingError += 360;

        telemetry.addData("SEQ Drive", "DistErr: %.1f, HeadErr: %.1f", distError, headingError);

        if (Math.abs(distError) < DISTANCE_THRESHOLD) {
            moveRobot(0, 0);
            return true;
        }

        // Negative power moves forward
        double drivePower = -Range.clip(distError * SPEED_GAIN, -MAX_SPEED, MAX_SPEED);
        // Turn correction: If we veer Left (positive error), we need Right turn.
        // Steering is "Left = +Yaw". So we need -Yaw.
        // So we need -HeadingError * Gain?
        // Let's test: Heading 10, Target 0. Error -10. 
        // We are Right of target. Need Left turn (+Yaw).
        // Error -10. We need Positive result. So -1 * -10.
        // Wait, headingError = target - current. 0 - 10 = -10.
        // -10 * Gain = Negative. Negative Yaw = Right Turn.
        // But we are at 10 (Right of 0). We need Left Turn.
        // So we need Positive Yaw.
        // So we need -1 * headingError.
        double turnPower = Range.clip(-headingError * TURN_GAIN, -MAX_TURN, MAX_TURN);

        moveRobot(drivePower, turnPower);
        return false;
    }

    private void moveRobot(double x, double yaw) {
        double leftPower    = x - yaw;
        double rightPower   = x + yaw;
        double max = Math.max(Math.abs(leftPower), Math.abs(rightPower));
        if (max > 1.0) { leftPower /= max; rightPower /= max; }
        robot.moveRobot(leftPower, rightPower);
    }

    private void updateLocalization() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        tagVisible = false;
        for (AprilTagDetection detection : detections) {
            if (detection.metadata != null && detection.id == TARGET_TAG_ID) {
                tagVisible = true;
                lastTagRange = detection.ftcPose.range;
                lastTagBearing = detection.ftcPose.bearing;
                break;
            }
        }
    }

    private double getRawHeading() { return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES); }
    private double getFieldHeading() { return getRawHeading() + headingOffset; }
    private void resetRelativeEncoder() { startEncoderPos = (leftEncoder.getCurrentPosition() + rightEncoder.getCurrentPosition()) / 2.0; }
    private double getRelativeEncoderDistance() { 
        return ((leftEncoder.getCurrentPosition() + rightEncoder.getCurrentPosition()) / 2.0 - startEncoderPos) / COUNTS_PER_INCH; 
    }

    private void initVision() {
        aprilTag = new AprilTagProcessor.Builder().setLensIntrinsics(FX, FY, CX, CY).build();
        aprilTag.setDecimation(2); 
        visionPortal = new VisionPortal.Builder().addProcessor(aprilTag).setCameraResolution(new Size(320, 240)).setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")).build();
    }
}
