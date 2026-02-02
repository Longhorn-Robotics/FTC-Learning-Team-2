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
    private Pose lastAnchorPose = new Pose(0, 0, 0); // The last position confirmed by Vision
    private double lastAnchorEncoder = 0; // Encoder average at the moment of last visual lock
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
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        imu.resetYaw();

        // Vision
        initVision();

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

        // 3. Telemetry Dashboard
        telemetry.addData("--- FIELD LOCALIZATION ---", "");
        telemetry.addData("Source", tagVisible ? "VISION (Absolute)" : "DEAD RECKONING (Estimated)");
        telemetry.addData("Field X", "%.1f\"", currentPose.x);
        telemetry.addData("Field Y", "%.1f\"", currentPose.y);
        telemetry.addData("Heading", "%.1f°", currentPose.heading);
        
        telemetry.addData("--- TARGETS ---", "");
        telemetry.addData("Red Tag (24)", "45.0°");
        telemetry.addData("Blue Tag (20)", "315.0°");
        
        telemetry.addData("--- RAW SENSORS ---", "");
        telemetry.addData("Enc Dist", "%.1f\"", getAvgEncoderDistance());
        telemetry.addData("IMU Yaw", "%.1f°", getRawHeading());
        telemetry.update();
    }

    private void updateHybridPose() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        AprilTagDetection validDetection = null;

        // Find a relevant tag
        for (AprilTagDetection detection : detections) {
            if (detection.metadata != null && 
               (detection.id == LocalizationUtils.RED_TAG_ID || detection.id == LocalizationUtils.BLUE_TAG_ID)) {
                validDetection = detection;
                break;
            }
        }

        double currentRawHeading = getRawHeading();

        if (validDetection != null) {
            // --- VISION VISIBLE: RESET ANCHOR ---
            tagVisible = true;
            
            // 1. Calculate Absolute Heading based on Tag
            // If looking at Red Tag (45 deg field location), Robot Heading = 45 - Tag Bearing?
            // Spec: Tag 24 is at Top Right. If we face it, we are facing 45 deg?
            // Let's assume the user wants 45 deg to be the bearing TO the tag.
            // But 'heading' is the robot's orientation.
            // Using LocalizationUtils logic:
            // We assume robotHeading + bearing = angle_to_tag.
            
            // For calibration test: 
            // We trust the IMU's relative changes, but snap absolute value if we see a known tag?
            // Let's keep it simple: Use LocalizationUtils to get Pose based on current IMU.
            
            double calibratedHeading = currentRawHeading + headingOffset;
            
            currentPose = LocalizationUtils.calculateFieldPose(
                    validDetection.id,
                    validDetection.ftcPose.range,
                    validDetection.ftcPose.bearing,
                    calibratedHeading
            );
            
            // Update Anchors for Dead Reckoning
            if (currentPose != null) {
                lastAnchorPose = new Pose(currentPose.x, currentPose.y, currentPose.heading);
                lastAnchorEncoder = getAvgEncoderDistance();
            }
        } else {
            // --- VISION LOST: DEAD RECKONING ---
            tagVisible = false;
            
            double currentEncoder = getAvgEncoderDistance();
            double distanceDelta = currentEncoder - lastAnchorEncoder;
            double currentHeading = currentRawHeading + headingOffset;
            
            // Calculate change in position based on heading
            // Note: This is a simple linear approximation (Arc motion would be better but this is sufficient for fallback)
            // X += d * sin(theta)
            // Y += d * cos(theta)
            double theta = Math.toRadians(currentHeading);
            double deltaX = distanceDelta * Math.sin(theta);
            double deltaY = distanceDelta * Math.cos(theta);
            
            currentPose = new Pose(
                lastAnchorPose.x + deltaX,
                lastAnchorPose.y + deltaY,
                currentHeading
            );
        }
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
