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
    private Pose currentPose = new Pose(0, 0, 0);
    private boolean tagVisible = false;

    @Override
    public void init() {
        robot.init(hardwareMap);

        // Encoders (for manual driving reference)
        leftEncoder = hardwareMap.get(DcMotor.class, "ld");
        rightEncoder = hardwareMap.get(DcMotor.class, "rd");
        leftEncoder.setDirection(DcMotor.Direction.REVERSE);
        rightEncoder.setDirection(DcMotor.Direction.FORWARD);

        // IMU
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
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
        updatePoseFromVision();

        // 3. Telemetry Dashboard
        telemetry.addData("--- FIELD LOCALIZATION ---", "");
        telemetry.addData("Tag Visible", tagVisible ? "YES" : "NO");
        
        if (tagVisible) {
            telemetry.addData("Field X", "%.1f inches", currentPose.x);
            telemetry.addData("Field Y", "%.1f inches", currentPose.y);
            telemetry.addData("Heading", "%.1f degrees", currentPose.heading);
            telemetry.addData("Distance to Center", "%.1f inches", 
                Math.hypot(currentPose.x, currentPose.y)); // Approx 0,0 is center? No, 0,0 is usually corner in FTC.
                                                           // Actually, spec says Tag 24 is (55.5, 55.5). Center is (0,0)?
                                                           // Let's check LocalizationUtils constants.
                                                           // LocalizationUtils: Tag is at (55.5, 55.5).
                                                           // Center of field is typically (0,0) in standard FTC coordinates.
        } else {
            telemetry.addData("Status", "Searching for AprilTag...");
            telemetry.addData("Last Known", currentPose.toString());
        }
        
        telemetry.addData("--- RAW SENSORS ---", "");
        telemetry.addData("IMU Yaw", "%.1f", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.update();
    }

    private void updatePoseFromVision() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        tagVisible = false;
        
        for (AprilTagDetection detection : detections) {
            if (detection.metadata != null) {
                // Get current absolute heading from IMU
                // Note: We are trusting the IMU for orientation relative to the field.
                // If IMU 0 is "Up/North", and we are facing the Tag (North-East), Yaw should be -45?
                // For this test, we assume IMU 0 is aligned with Field 0 (Goal Wall).
                double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                
                // Calculate Pose
                Pose detectedPose = LocalizationUtils.calculateFieldPose(
                        detection.id,
                        detection.ftcPose.range,
                        detection.ftcPose.bearing,
                        heading
                );
                
                if (detectedPose != null) {
                    currentPose = detectedPose;
                    tagVisible = true;
                    break; // Use the first valid tag
                }
            }
        }
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
