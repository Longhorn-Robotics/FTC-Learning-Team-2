package org.firstinspires.ftc.teamcode;

import android.util.Size;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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

@TeleOp(name = "Driver Control - Robust", group = "Robot")
public class RobotTeleOp2 extends OpMode {

    // --- HARDWARE ---
    RobotHardware robot = new RobotHardware();
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
    private static final double MAX_AUTO_SPEED = 0.6; 
    private static final double MAX_AUTO_TURN = 0.3; 
    private static final double SPEED_GAIN = 0.03;
    private static final double TURN_GAIN = 0.015; 
    private static final double HEADING_THRESHOLD = 2.0; 
    private static final double DISTANCE_THRESHOLD = 2.0; 

    // Target: Launcher is ~120 inches from the Tag
    private static final double LAUNCH_TARGET_DISTANCE = 120.0;
    private static final double TAG_FIELD_ANGLE = 45.0;

    // --- LOCALIZATION MEMORY ---
    private boolean tagVisible = false;
    private double lastTagRange = 0;
    private double lastTagBearing = 0;
    private double lastKnownTagRange = 0;
    private double startEncoderPos = 0;
    private double headingOffset = 0; 
    private boolean isCalibrated = false;

    // --- DRIVER INPUTS ---
    double LStickY;
    double RStickY;
    boolean RBumper = false;
    boolean LBumper = false;

    @Override
    public void init() {
        robot.init(hardwareMap);

        // Initialize Encoders for Dead Reckoning
        leftEncoder = hardwareMap.get(DcMotor.class, "ld");
        rightEncoder = hardwareMap.get(DcMotor.class, "rd");
        // Directions match RobotAutonomousFinal
        leftEncoder.setDirection(DcMotor.Direction.REVERSE);
        rightEncoder.setDirection(DcMotor.Direction.FORWARD);
        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Initialize IMU
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));

        // Initialize Vision
        initVision();

        telemetry.addData("Status", "Initialized - Ready to Run");
        telemetry.update();
    }

    @Override
    public void loop() {
        // 1. Read Sensors & Update Localization
        updateLocalization();

        // 2. Read Inputs
        LStickY = -this.gamepad1.left_stick_y;
        RStickY = this.gamepad1.right_stick_y;
        RBumper = this.gamepad1.right_bumper;
        LBumper = this.gamepad1.left_bumper;

        // 3. Control Logic
        if (LBumper) {
            // --- AUTO-ALIGN TO LAUNCHER ---
            telemetry.addData("Mode", "AUTO-ALIGNING");
            
            // Drive to 120" from tag, facing the tag (45 deg)
            boolean reached = performNavigationStep(LAUNCH_TARGET_DISTANCE, 45.0);
            
            if (reached) {
                telemetry.addData("Status", "Aligned! Ready to Launch.");
                robot.moveRobot(0, 0);
            }
        } else {
            // --- MANUAL DRIVER CONTROL ---
            // If we are just driving, keep updating our "Anchor" whenever we see the tag
            // so that if we press the button later, we have a fresh start.
            if (tagVisible) {
                resetRelativeEncoder();
                lastKnownTagRange = lastTagRange;
                // Opportunistic Calibration: If we see the tag, ensure heading is correct
                // RobotHeading = TagFieldAngle - TagBearing
                headingOffset = (TAG_FIELD_ANGLE - lastTagBearing) - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                isCalibrated = true;
            }

            // Standard Tank Drive (Using raw stick values)
            robot.moveRobot(LStickY, RStickY);
            
            // Intake/Launcher logic from RobotTeleOp.java
            if (!RBumper) {
                robot.idleLauncher(0);
            } else {
                robot.launchItems(1);
            }
            robot.runIntake(1);
        }

        // 4. Telemetry
        telemetry.addData("Tag Visible", tagVisible);
        telemetry.addData("Field Heading", "%.1f", getHeading());
        if (isCalibrated) {
            telemetry.addData("Est. Range", "%.1f", (tagVisible ? lastTagRange : (lastKnownTagRange - getRelativeEncoderDistance())));
        } else {
            telemetry.addData("Status", "Drive to Tag to Calibrate!");
        }
        telemetry.update();
    }

    /**
     * Hybrid Navigation Logic (Same as Autonomous)
     */
    private boolean performNavigationStep(double targetRangeFromTag, double targetFieldHeading) {
        double rangeError;
        double headingError;

        if (tagVisible) {
            // VISUAL
            rangeError = lastTagRange - targetRangeFromTag;
            
            // If facing the tag (approx), use Vision Bearing
            if (Math.abs(targetFieldHeading - TAG_FIELD_ANGLE) < 15) {
                headingError = lastTagBearing;
            } else {
                headingError = targetFieldHeading - getHeading();
            }
            
            resetRelativeEncoder();
            lastKnownTagRange = lastTagRange;
        } else {
            // SENSOR FALLBACK
            if (!isCalibrated) {
                // If we've NEVER seen the tag, we can't auto-align reliably.
                telemetry.addData("Error", "No Field Calibration!");
                return false; 
            }
            
            double currentEncDist = getRelativeEncoderDistance();
            double estimatedRange = lastKnownTagRange - currentEncDist;
            rangeError = estimatedRange - targetRangeFromTag;
            headingError = targetFieldHeading - getHeading();
            telemetry.addData("Nav Mode", "SENSOR FALLBACK");
        }

        // Normalize angle
        while (headingError > 180) headingError -= 360;
        while (headingError <= -180) headingError += 360;

        double drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
        double turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);

        if (Math.abs(rangeError) < DISTANCE_THRESHOLD && Math.abs(headingError) < HEADING_THRESHOLD) {
            moveRobotAuto(0, 0);
            return true;
        }

        moveRobotAuto(drive, turn);
        return false;
    }

    /**
     * Tank Drive Mixer for Auto-Alignment
     * Uses the same corrected logic: Left = x - yaw, Right = x + yaw
     */
    private void moveRobotAuto(double x, double yaw) {
        double leftPower    = x - yaw;
        double rightPower   = x + yaw;
        
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
