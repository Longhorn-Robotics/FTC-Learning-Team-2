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

@Autonomous(name = "Robot Auto Final - Fixed", group = "Robot")
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
    
    private static final double FX = 357.1;
    private static final double FY = 357.1;
    private static final double CX = 159.5;
    private static final double CY = 119.5;

    // --- TUNING CONSTANTS ---
    private static final double MAX_SPEED = 0.4; 
    private static final double MAX_TURN = 0.25; 
    private static final double SPEED_GAIN = 0.03;
    private static final double TURN_GAIN = 0.015; 
    private static final double HEADING_THRESHOLD = 1.5; 
    private static final double DISTANCE_THRESHOLD = 1.0; 

    // Field angle of the corner AprilTag (Top-Right = -45 degrees)
    private static final double TAG_FIELD_ANGLE = -45.0;

    // --- LOCALIZATION MEMORY ---
    private boolean tagVisible = false;
    private double lastTagRange = 0;
    private double lastTagBearing = 0;
    private double lastKnownTagRange = 0;
    private double startEncoderPos = 0;
    private double headingOffset = 0; 

    // --- FIELD GEOMETRY ---
    private static final double LANE_ALIGNMENT_DISTANCE = 36.0;
    private static final double ARTIFACT_ROW_DEPTH = 15.0;
    private double[] rowDepths = {-12.0, -36.0, -60.0, -84.0};

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
        robot.AutoInit(hardwareMap);

        leftEncoder = hardwareMap.get(DcMotor.class, "ld");
        rightEncoder = hardwareMap.get(DcMotor.class, "rd");
        leftEncoder.setDirection(DcMotor.Direction.REVERSE);
        rightEncoder.setDirection(DcMotor.Direction.FORWARD);
        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));

        initVision();

        telemetry.addData("Status", "Initialized - READY");
        telemetry.update();

        waitForStart();

        ElapsedTime autoTimer = new ElapsedTime();
        autoTimer.reset();

        ElapsedTime visionTimeout = new ElapsedTime();
        while (opModeIsActive() && !tagVisible && visionTimeout.seconds() < 4.0) {
            updateLocalization();
            telemetry.addData("Status", "Waiting for initial tag lock...");
            telemetry.update();
        }

        if (tagVisible) {
            headingOffset = (TAG_FIELD_ANGLE - lastTagBearing) - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            lastKnownTagRange = lastTagRange;
            telemetry.addData("Status", "Calibrated! Offset: %.1f", headingOffset);
        } else {
            headingOffset = 0; 
            lastKnownTagRange = 24.0;
            telemetry.addData("Status", "Calibration Failed. Using raw IMU.");
        }
        telemetry.update();

        if (opModeIsActive()) {
            currentState = State.ALIGN_TO_TAG_START;
        }

        while (opModeIsActive()) {
            updateLocalization();
            
            if (autoTimer.seconds() > 25.0 && currentState != State.DONE) {
                if (performNavigationStep(60.0, 0)) currentState = State.DONE;
            } else {
                executeStateMachine();
            }
            
            updateTelemetry();
        }
        visionPortal.close();
    }

    private void executeStateMachine() {
        switch (currentState) {
            case INIT:
                break;

            case ALIGN_TO_TAG_START:
                if (performNavigationStep(lastKnownTagRange, -45)) {
                    currentState = State.DRIVE_TO_LANE;
                }
                break;

            case DRIVE_TO_LANE:
                if (performNavigationStep(LANE_ALIGNMENT_DISTANCE, -45)) {
                    currentState = State.TURN_UP;
                }
                break;

            case TURN_UP:
                if (performNavigationStep(LANE_ALIGNMENT_DISTANCE, 0)) {
                    currentLaneDepth = rowDepths[currentRow];
                    resetRelativeEncoder();
                    currentState = State.DRIVE_TO_ROW;
                }
                break;

            case DRIVE_TO_ROW:
                if (performNavigationStep(LANE_ALIGNMENT_DISTANCE + currentLaneDepth, 0)) {
                    currentState = State.TURN_TO_BALLS;
                }
                break;

            case TURN_TO_BALLS:
                if (performNavigationStep(LANE_ALIGNMENT_DISTANCE + currentLaneDepth, -90)) {
                    resetRelativeEncoder();
                    currentState = State.COLLECT_BALLS;
                }
                break;

            case COLLECT_BALLS:
                robot.runIntake(1.0);
                if (performNavigationStep(LANE_ALIGNMENT_DISTANCE + currentLaneDepth + ARTIFACT_ROW_DEPTH, -90)) {
                    robot.runIntake(0);
                    currentState = State.REVERSE_FROM_BALLS;
                }
                break;

            case REVERSE_FROM_BALLS:
                if (performNavigationStep(LANE_ALIGNMENT_DISTANCE + currentLaneDepth, -90)) {
                    currentState = State.TURN_UP_RETURN;
                }
                break;

            case TURN_UP_RETURN:
                if (performNavigationStep(LANE_ALIGNMENT_DISTANCE + currentLaneDepth, 0)) {
                    currentState = State.RETURN_TO_LANE_START;
                }
                break;

            case RETURN_TO_LANE_START:
                if (performNavigationStep(LANE_ALIGNMENT_DISTANCE, 0)) {
                    currentState = State.TURN_TO_TAG;
                }
                break;

            case TURN_TO_TAG:
                if (performNavigationStep(LANE_ALIGNMENT_DISTANCE, -45)) {
                    currentState = State.DRIVE_TO_SCORE;
                }
                break;

            case DRIVE_TO_SCORE:
                if (performNavigationStep(12.0, -45)) {
                         currentState = State.SCORE;
                }
                break;

            case SCORE:
                moveRobot(0,0);
                robot.launchItems(1.0);
                sleep(1500);
                robot.launchItems(0);
                
                currentRow++;
                if (currentRow >= 2) { 
                    currentState = State.DONE;
                } else {
                    currentState = State.DRIVE_TO_LANE; 
                }
                break;

            case DONE:
                moveRobot(0, 0);
                break;
        }
    }

    private boolean performNavigationStep(double targetRangeFromTag, double targetFieldHeading) {
        double rangeError;
        double headingError;

        if (tagVisible) {
            rangeError = lastTagRange - targetRangeFromTag;
            
            // Decoupled Heading Logic:
            // Use Vision Bearing ONLY if we are facing the tag (approx 45 deg field angle)
            // Otherwise, strictly use the IMU to avoid the "spiral" effect.
            if (Math.abs(targetFieldHeading - TAG_FIELD_ANGLE) < 10) {
                headingError = lastTagBearing;
            } else {
                headingError = targetFieldHeading - getHeading();
            }
            
            resetRelativeEncoder();
            lastKnownTagRange = lastTagRange;
        } else {
            double currentEncDist = getRelativeEncoderDistance();
            double estimatedRange = lastKnownTagRange - currentEncDist;
            rangeError = estimatedRange - targetRangeFromTag;
            headingError = targetFieldHeading - getHeading();
        }

        while (headingError > 180) headingError -= 360;
        while (headingError <= -180) headingError += 360;

        double drive = Range.clip(rangeError * SPEED_GAIN, -MAX_SPEED, MAX_SPEED);
        double turn = Range.clip(headingError * TURN_GAIN, -MAX_TURN, MAX_TURN);

        if (Math.abs(rangeError) < DISTANCE_THRESHOLD && Math.abs(headingError) < HEADING_THRESHOLD) {
            moveRobot(0, 0);
            return true;
        }

        moveRobot(drive, turn);
        return false;
    }

    private void moveRobot(double x, double yaw) {
        double leftPower    = x + yaw;
        double rightPower   = x - yaw;
        
        double max = Math.max(Math.abs(leftPower), Math.abs(rightPower));
        if (max > 1.0) {
            leftPower /= max;
            rightPower /= max;
        }
        robot.moveRobot(leftPower, rightPower);
    }

    private void updateLocalization() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        tagVisible = false;
        for (AprilTagDetection detection : detections) {
            if (detection.metadata != null) {
                tagVisible = true;
                lastTagRange = detection.ftcPose.range;
                lastTagBearing = detection.ftcPose.bearing;
                break;
            }
        }
    }

    private void updateTelemetry() {
        telemetry.addData("STATE", currentState);
        telemetry.addData("Tag", tagVisible ? "VISIBLE" : "LOST");
        telemetry.addData("Heading", "%.1f°", getHeading());
        telemetry.addData("Range", "%.1f\"", (tagVisible ? lastTagRange : (lastKnownTagRange - getRelativeEncoderDistance())));
        telemetry.update();
    }

    private double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) + headingOffset;
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
