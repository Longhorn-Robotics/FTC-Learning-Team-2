package org.firstinspires.ftc.teamcode;

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
import java.util.List;

@TeleOp(name = "Localization Test", group = "Test")
public class LocalizationTest extends LinearOpMode {
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private IMU imu;

    @Override
    public void runOpMode() {
        // Initialize IMU
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));

        // Initialize AprilTag
        aprilTag = new AprilTagProcessor.Builder().build();
        visionPortal = new VisionPortal.Builder()
                .addProcessor(aprilTag)
                .setCameraResolution(new Size(320, 240))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();

        telemetry.addData("Status", "Ready. Place robot at center field.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            List<AprilTagDetection> detections = aprilTag.getDetections();
            
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
                telemetry.addData("Status", "No Tag Detected");
            }
            telemetry.update();
        }
        visionPortal.close();
    }
}
