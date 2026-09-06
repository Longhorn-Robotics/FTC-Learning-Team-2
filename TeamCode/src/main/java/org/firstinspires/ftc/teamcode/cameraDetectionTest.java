package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import android.util.Size;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;

import java.util.List;

@TeleOp(name = "IT'S AGI!!! TRUST!!!", group = "Test")
public class cameraDetectionTest extends LinearOpMode {
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private IMU imu;
    private static final double FX = 357.1;
    private static final double FY = 476.2;
    private static final double CX = 357.1;
    private static final double CY = 119.5;

    @Override
    public void runOpMode() {
        // Initialize IMU
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT)));

        // Initialize AprilTag
        aprilTag = new AprilTagProcessor.Builder()
                .setLensIntrinsics(FX, FY, CX, CY)
                .build();







        ColorBlobLocatorProcessor colorLocatorPurple = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.GREEN)   // Use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.asUnityCenterCoordinates(-1, 1, 1, -1))
                .setDrawContours(true)   // Show contours on the Stream Preview
                .setBoxFitColor(0)       // Disable the drawing of rectangles
                .setCircleFitColor(Color.rgb(255, 255, 0)) // Draw a circle
                .setBlurSize(5)          // Smooth the transitions between differ=ent colors in image


                // the following options have been added to fill in perimeter holes.
                .setDilateSize(15)       // Expand blobs to fill any divots on the edges
                .setErodeSize(15)        // Shrink blobs back to original size
                .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)


                .build();




        ColorBlobLocatorProcessor colorLocatorGreen = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.YELLOW)   // Use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.asUnityCenterCoordinates(-1, 1, 1, -1))
                .setDrawContours(true)   // Show contours on the Stream Preview
                .setBoxFitColor(0)       // Disable the drawing of rectangles
                .setCircleFitColor(Color.rgb(255, 255, 0)) // Draw a circle
                .setBlurSize(5)          // Smooth the transitions between different colors in image


                // the following options have been added to fill in perimeter holes.
                .setDilateSize(15)       // Expand blobs to fill any divots on the edges
                .setErodeSize(15)        // Shrink blobs back to original size
                .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)


                .build();



        visionPortal = new VisionPortal.Builder()
                .addProcessor(colorLocatorPurple)
                .addProcessor(colorLocatorGreen)
                .addProcessor(aprilTag)
                .setCameraResolution(new Size(320, 240))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .enableLiveView(true)
                .build();
















        telemetry.addData("Status", "Ready. Place robot at center field.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            List<AprilTagDetection> detections = aprilTag.getDetections();
            telemetry.addData("AprilTags", detections);
            telemetry.addData("Vision FPS", "%.1f", visionPortal.getFps());
            telemetry.addData("Detection Count", detections.size());
            boolean tagSeen = false;
            for (AprilTagDetection detection : detections) {
                if (detection.metadata != null) {
                    Pose fieldPose = LocalizationUtils.calculateFieldPose(
                            detection.id, detection.ftcPose.range, detection.ftcPose.bearing, heading);
                    
                    if (fieldPose != null) {
                        tagSeen = true;
                        telemetry.addData("ID", detection.id);
                        telemetry.addData("Field X", "%.2f\"", fieldPose.x); // Corrected escaping for "
                        telemetry.addData("Field Y", "%.2f\"", fieldPose.y); // Corrected escaping for "
                        telemetry.addData("Heading", "%.2f°", fieldPose.heading);
                    }
                }
            }

            if (!tagSeen) {
                //telemetry.addData("Status", "No Tag Detected");
            }
            telemetry.update();
        }
        visionPortal.close();
    }
}
