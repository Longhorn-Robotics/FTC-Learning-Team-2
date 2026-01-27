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

    // --- STATE MACHINE ---
    private enum State {
        INIT,
        DONE
    }
    private State currentState = State.INIT;

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

        while (opModeIsActive()) {
            // Update sensor data
            updateLocalization();
            
            // Comprehensive Telemetry
            updateTelemetry();

            switch (currentState) {
                case INIT:
                    // Main logic will be added here
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
