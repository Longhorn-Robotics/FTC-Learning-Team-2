package org.firstinspires.ftc.teamcode;

import android.util.Size;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * TeleOp mode to test and visualize Field-Centric Localization.
 * Displays calculated X, Y, and Heading on Driver Station telemetry.
 */
@TeleOp(name = "Concept: Localization Test", group = "Concept")
public class ConceptLocalizationTest extends OpMode {

    // --- HARDWARE ---
    private RobotHardware robot = new RobotHardware();
    private IMU imu;
    private DcMotor leftEncoder, rightEncoder;

    // --- VISION ---
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    // --- CONSTANTS ---
    // Camera Intrinsics (Logitech C270)
    private static final double FX = 357.1;
    private static final double FY = 357.1;
    private static final double CX = 159.5;
    private static final double CY = 119.5;

    // --- STATE ---
    private Pose currentPose = new Pose(0, 0, 0); // Start at (0,0,0) assumption
    private double lastLoopEncoder = 0; // Encoder value from the previous loop iteration
    private boolean tagVisible = false;
    
    private double headingOffset = 0; // Calibration offset for IMU
    private static final double COUNTS_PER_INCH = 45.28; // From RobotAutonomousFinal

    @Override
    public void init() {
        robot.init(hardwareMap);

        // Encoders (for manual driving reference)
        leftEncoder = hardwareMap.get(DcMotor.class, "ld");
        rightEncoder = hardwareMap.get(DcMotor.class, "rd");
        leftEncoder.setDirection(DcMotor.Direction.REVERSE);
        rightEncoder.setDirection(DcMotor.Direction.FORWARD);
        
        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Use RUN_WITHOUT for manual teleop feel, just read position
        rightEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // IMU
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT)));
        imu.resetYaw();

        // Vision
        initVision();
        
        // Initialize encoder state
        lastLoopEncoder = getAvgEncoderDistance();

        telemetry.addData("Status", "Initialized - Drive to a Tag!");
        telemetry.update();
    }

    @Override
    public void loop() {
        // 1. Driver Control (Standard Tank)
        double leftY = -gamepad1.left_stick_y;
        double rightY = -gamepad1.right_stick_y;
        robot.moveRobot(leftY, rightY);

        // 2. Localization Update
        updateHybridPose();

        // 3. Telemetry Dashboard - ALWAYS DISPLAYED
        telemetry.addData("--- FIELD LOCALIZATION ---", "");
        telemetry.addData("Source", tagVisible ? "VISION (Absolute)" : "DEAD RECKONING (Integrated)");
        telemetry.addData("Field X", "%.1f\"", currentPose.x);
        telemetry.addData("Field Y", "%.1f\"", currentPose.y);
        telemetry.addData("Heading", "%.1f°", currentPose.heading);
        
        telemetry.addData("--- TARGETS ---", "");
        telemetry.addData("Red Tag (24)", "45.0°");
        telemetry.addData("Blue Tag (20)", "315.0°");
        
        if (!tagVisible) {
            telemetry.addData("Status", "Tag Lost - DEAD RECKONING ACTIVE");
        }
        
        telemetry.addData("--- RAW SENSORS ---", "");
        telemetry.addData("Enc Dist", "%.1f\"", getAvgEncoderDistance());
        telemetry.addData("IMU Yaw", "%.1f°", getRawHeading());
        telemetry.update();
    }

    private void updateHybridPose() {
        // 1. Calculate Encoder Delta (since last loop)
        double currentEncoder = getAvgEncoderDistance();
        double deltaDistance = currentEncoder - lastLoopEncoder;
        lastLoopEncoder = currentEncoder; // Update for next loop

        // 2. Get Vision Data
        List<AprilTagDetection> detections = aprilTag.getDetections();
        AprilTagDetection validDetection = null;

        for (AprilTagDetection detection : detections) {
            if (detection.metadata != null && 
               (detection.id == LocalizationUtils.RED_TAG_ID || detection.id == LocalizationUtils.BLUE_TAG_ID)) {
                validDetection = detection;
                break;
            }
        }

        // 3. Get Current Heading (IMU)
        double currentRawHeading = getRawHeading();
        double currentFieldHeading = currentRawHeading + headingOffset;
        
        // Manual Reset for Testing
        if (gamepad1.a) {
            currentPose = new Pose(0,0,0);
            headingOffset = -currentRawHeading; // Reset heading to 0
            currentFieldHeading = 0;
            telemetry.addData("Debug", "Pose Reset to (0,0,0)");
        }

        if (validDetection != null) {
            // --- VISION VISIBLE: ABSOLUTE CORRECTION ---
            tagVisible = true;
            
            // Calculate absolute heading offset
            double tagFieldAngle = (validDetection.id == LocalizationUtils.RED_TAG_ID) ? 45.0 : 315.0;
            headingOffset = (tagFieldAngle - validDetection.ftcPose.bearing) - currentRawHeading;
            
            // Recalculate heading with new offset
            currentFieldHeading = currentRawHeading + headingOffset;
            
            // Calculate Absolute Pose from Vision
            Pose visionPose = LocalizationUtils.calculateFieldPose(
                    validDetection.id,
                    validDetection.ftcPose.range,
                    validDetection.ftcPose.bearing,
                    currentFieldHeading
            );
            
            if (visionPose != null) {
                currentPose = visionPose;
            }
        } else {
            // --- VISION LOST: INCREMENTAL DEAD RECKONING ---
            tagVisible = false;
            
            // Integrate position based on previous pose + delta vector
            double theta = Math.toRadians(currentFieldHeading);
            double deltaX = deltaDistance * Math.sin(theta);
            double deltaY = deltaDistance * Math.cos(theta);
            
            currentPose.x += deltaX;
            currentPose.y += deltaY;
            currentPose.heading = currentFieldHeading;
            
            telemetry.addData("Debug DR", "dX: %.2f, dY: %.2f", deltaX, deltaY);
        }
        
        telemetry.addData("Debug Heading", "Raw: %.1f, Off: %.1f, Field: %.1f", 
            currentRawHeading, headingOffset, currentFieldHeading);
    }

    private double getAvgEncoderDistance() {
        return ((leftEncoder.getCurrentPosition() + rightEncoder.getCurrentPosition()) / 2.0) / COUNTS_PER_INCH;
    }
    
    private double getRawHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
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
