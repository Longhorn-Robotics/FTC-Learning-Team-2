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

@Autonomous(name = "Robot Auto Final - Sequential Core", group = "Robot")
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
    private static final double HEADING_THRESHOLD = 1.0; // Tighten for accuracy
    private static final double DISTANCE_THRESHOLD = 1.0; 

    // --- FIELD CONFIGURATION ---
    private static final int TARGET_TAG_ID = 24; 
    // Tag is at the Top-Right corner (45 degrees in Field Coordinates)
    private static final double TAG_FIELD_HEADING = 45.0;

    // --- LOCALIZATION MEMORY ---
    private boolean tagVisible = false;
    private double lastTagRange = 0;
    private double lastTagBearing = 0;
    private double headingOffset = 0; // Field Heading = Raw IMU + Offset
    private double startEncoderPos = 0; // Anchor for relative moves

    // --- STATE MACHINE ---
    private enum State {
        INIT,
        CALIBRATE_HEADING,
        TEST_SEQ_TURN,  // Phase 1 Test
        TEST_SEQ_DRIVE, // Phase 1 Test
        DONE
    }
    private State currentState = State.INIT;

    @Override
    public void runOpMode() {
        // 1. Initialize Hardware
        robot.AutoInit(hardwareMap);

        // Encoder Setup (Direction must match RobotHardware logic)
        leftEncoder = hardwareMap.get(DcMotor.class, "ld");
        rightEncoder = hardwareMap.get(DcMotor.class, "rd");
        leftEncoder.setDirection(DcMotor.Direction.REVERSE);
        rightEncoder.setDirection(DcMotor.Direction.FORWARD);
        
        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // IMU Setup
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));

        // Vision Setup
        initVision();

        telemetry.addData("Status", "Initialized - Phase 1 Core");
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

            // Safety Timer
            if (autoTimer.seconds() > 29.0) {
                currentState = State.DONE;
            }

            switch (currentState) {
                case INIT:
                    break;

                case CALIBRATE_HEADING:
                    // We assume robot starts facing roughly towards the tag (45 deg field).
                    // We wait for visibility to lock the offset.
                    if (tagVisible) {
                        // Math: RobotHeading = TagFieldHeading - TagBearing
                        // If facing tag directly (Bearing 0), RobotHeading = 45.
                        // If turned left 10 deg (Heading 55), Tag is to right (Bearing -10). 45 - (-10) = 55.
                        double currentRawHeading = getRawHeading();
                        double fieldHeading = TAG_FIELD_HEADING - lastTagBearing;
                        headingOffset = fieldHeading - currentRawHeading;
                        
                        telemetry.addData("Calibration", "LOCKED");
                        telemetry.addData("Offset", headingOffset);
                        
                        // Proceed to testing the sequential moves
                        currentState = State.TEST_SEQ_TURN;
                    } else {
                        telemetry.addData("Calibration", "WAITING FOR TAG...");
                    }
                    break;

                case TEST_SEQ_TURN:
                    // Test: Turn to 0 degrees (Up)
                    if (turnTo(0.0)) {
                        resetRelativeEncoder();
                        currentState = State.TEST_SEQ_DRIVE;
                    }
                    break;

                case TEST_SEQ_DRIVE:
                    // Test: Drive 12 inches forward at 0 degrees
                    if (driveStraight(12.0, 0.0)) {
                        currentState = State.DONE;
                    }
                    break;

                case DONE:
                    moveRobot(0, 0);
                    break;
            }
        }
        visionPortal.close();
    }

    /**
     * Phase 1: Sequential Turn
     * Rotates the robot to a specific absolute field heading using the IMU.
     */
    private boolean turnTo(double targetHeading) {
        double currentHeading = getFieldHeading();
        double headingError = targetHeading - currentHeading;

        // Normalize error to -180 to +180
        while (headingError > 180) headingError -= 360;
        while (headingError <= -180) headingError += 360;

        telemetry.addData("SEQ Turn", "Target: %.1f, Curr: %.1f, Err: %.1f", 
                targetHeading, currentHeading, headingError);

        if (Math.abs(headingError) < HEADING_THRESHOLD) {
            moveRobot(0, 0);
            return true;
        }

        // P-Controller
        // Mixer: Left = x - yaw.
        // If Error is Positive (Target > Current), we need to turn Left (CCW).
        // Positive Yaw in Mixer (-yaw) -> Left Power Decrease?
        // Wait: Left = x - yaw. If yaw is positive, Left decreases, Right increases (x + yaw).
        // Right > Left -> Turn LEFT (CCW).
        // So Positive Yaw = CCW Turn.
        // Positive Error -> Needs CCW Turn -> Positive Yaw.
        double turnPower = Range.clip(headingError * TURN_GAIN, -MAX_TURN, MAX_TURN);
        
        moveRobot(0, turnPower);
        return false;
    }

    /**
     * Phase 1: Sequential Drive
     * Drives a set distance while actively locking heading.
     * Currently purely Encoder/IMU based as requested for core logic verification.
     */
    private boolean driveStraight(double targetInches, double targetHeading) {
        double currentDist = getRelativeEncoderDistance();
        double distError = targetInches - currentDist;
        
        double currentHeading = getFieldHeading();
        double headingError = targetHeading - currentHeading;

        // Normalize heading error
        while (headingError > 180) headingError -= 360;
        while (headingError <= -180) headingError += 360;

        telemetry.addData("SEQ Drive", "DistErr: %.1f, HeadErr: %.1f", distError, headingError);

        if (Math.abs(distError) < DISTANCE_THRESHOLD) {
            moveRobot(0, 0);
            return true;
        }

        double drivePower = Range.clip(distError * SPEED_GAIN, -MAX_SPEED, MAX_SPEED);
        double turnPower = Range.clip(headingError * TURN_GAIN, -MAX_TURN, MAX_TURN);

        moveRobot(drivePower, turnPower);
        return false;
    }

    /**
     * Tank Drive Mixer (User Provided)
     * Left = x - yaw
     * Right = x + yaw
     * 
     * Analysis:
     * If Yaw > 0: Left decreases, Right increases. Robot turns LEFT (CCW).
     * If X > 0: Both increase. Robot moves FORWARD.
     */
    private void moveRobot(double x, double yaw) {
        double leftPower    = x - yaw;
        double rightPower   = x + yaw;

        double max = Math.max(Math.abs(leftPower), Math.abs(rightPower));
        if (max > 1.0) {
            leftPower /= max;
            rightPower /= max;
        }
        robot.moveRobot(leftPower, rightPower);
    }

    // --- HELPERS ---

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

    private double getRawHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    private double getFieldHeading() {
        return getRawHeading() + headingOffset;
    }

    private void updateTelemetry() {
        telemetry.addData("Tag", tagVisible ? "VISIBLE" : "LOST");
        telemetry.addData("Heading (Field)", "%.1f", getFieldHeading());
        telemetry.update();
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
        aprilTag.setDecimation(2); 
        visionPortal = new VisionPortal.Builder()
                .addProcessor(aprilTag)
                .setCameraResolution(new Size(320, 240))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();
    }
}
