package org.firstinspires.ftc.teamcode;

import android.util.Size;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name = "Robot Auto 3 - Focal Precision", group = "Robot")
public class RobotAutonomous3 extends LinearOpMode {

    /* --- HARDWARE & CONSTANTS --- */
    final double TICKS_PER_INCH = 45.28;
    final double TARGET_LAUNCH_DISTANCE = 24.0;

    /* --- TUNING CONTROLS --- */
    final double DRIVE_GAIN = 0.04;       // Speed reduction based on distance error
    final double TURN_GAIN  = 0.02;       // Turn sensitivity based on angle error
    final double MAX_AUTO_SPEED = 0.4;
    final double MAX_AUTO_TURN  = 0.25;

    /* --- CAMERA & FOCAL LENGTH CONSTANTS --- */
    private final int cameraWidth = 320;
    private final int cameraHeight = 240;
    private final double cameraCenterX = (cameraWidth - 1) / 2.0;
    private final double cameraCenterY = (cameraHeight - 1) / 2.0;

    // Moto E5 Play Selfie Camera intrinsics
    private final double focalLengthX = 220.9;
    private final double focalLengthY = 220.9;

    /* --- SENSOR FUSION MEMORY --- */
    private double lastKnownTagDistance = 0;
    private double lastKnownTagBearing  = 0; // Calculated using focal length
    private int    encoderAtLastSight   = 0;
    private boolean tagIsVisible        = false;

    private enum State {
        START_LAUNCH, MOVE_TO_ROW, PIVOT_TO_BALLS, PLOW_ROW,
        RETURN_TO_WALL, HEAD_TO_ZONE, ALIGN_AND_SCORE, PARK
    }

    private State currentState = State.START_LAUNCH;
    RobotHardware robot = new RobotHardware();
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private IMU imu;

    @Override
    public void runOpMode() {
        robot.AutoInit(hardwareMap);
        initIMU();
        initVision();

        while (!isStarted() && !isStopRequested()) {
            updateTagSighting();
            telemetry.addData("Status", "WAITING FOR START");
            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive()) {
            updateTagSighting();

            switch (currentState) {
                case START_LAUNCH:
                    robot.launchItems(0.8);
                    sleep(1500);
                    robot.idleLauncher(0);
                    currentState = State.MOVE_TO_ROW;
                    break;

                case MOVE_TO_ROW:
                    // Drive BACKWARDS to increase distance from tag to 48 inches
                    driveToTagDistance(48.0, 0.0);
                    currentState = State.PIVOT_TO_BALLS;
                    break;

                case PIVOT_TO_BALLS:
                    turnToAngle(90.0);
                    currentState = State.PLOW_ROW;
                    break;

                case PLOW_ROW:
                    robot.runIntake(1.0);
                    driveStraight(0.4, 24.0, 90.0);
                    robot.runIntake(0);
                    currentState = State.RETURN_TO_WALL;
                    break;

                case RETURN_TO_WALL:
                    driveStraight(-0.4, 24.0, 90.0);
                    turnToAngle(0.0);
                    currentState = State.HEAD_TO_ZONE;
                    break;

                case HEAD_TO_ZONE:
                    // Drive FORWARDS to return to striking range
                    driveToTagDistance(TARGET_LAUNCH_DISTANCE + 5.0, 0.0);
                    currentState = State.ALIGN_AND_SCORE;
                    break;

                case ALIGN_AND_SCORE:
                    if (alignWithFusion()) {
                        stopRobot();
                        robot.launchItems(0.8);
                        sleep(1500);
                        robot.idleLauncher(0);
                        currentState = State.PARK;
                    }
                    break;

                case PARK:
                    stopRobot();
                    break;
            }
            telemetry.update();
        }
    }

    /**
     * Proportional drive to a specific distance using Focal Length bearing for steering.
     */
    private void driveToTagDistance(double targetDistance, double targetHeading) {
        while (opModeIsActive()) {
            updateTagSighting();

            double currentRange;
            double headingError;

            if (tagIsVisible) {
                currentRange = lastKnownTagDistance;
                headingError = lastKnownTagBearing; // Precision calculated bearing
            } else {
                double inchesSinceSight = (getCurrentEncoderAverage() - encoderAtLastSight) / TICKS_PER_INCH;
                // Basic fusion: if speed was negative, distance increases
                currentRange = lastKnownTagDistance - (inchesSinceSight);
                headingError = targetHeading - getHeading();
            }

            double rangeError = currentRange - targetDistance;

            // Check threshold
            if (Math.abs(rangeError) < 1.0) break;

            // Proportional Speed Logic
            double drive = Range.clip(rangeError * DRIVE_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            double turn  = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);

            moveRobot(drive, turn);

            telemetry.addData("State", "Tag Drive");
            telemetry.addData("Range Error", rangeError);
            telemetry.addData("Heading Error", headingError);
            telemetry.update();
        }
        stopRobot();
    }

    private boolean alignWithFusion() {
        double currentRange;
        double currentHeading;

        if (tagIsVisible) {
            currentRange = lastKnownTagDistance;
            currentHeading = lastKnownTagBearing;
        } else {
            double inchesSinceSight = (getCurrentEncoderAverage() - encoderAtLastSight) / TICKS_PER_INCH;
            currentRange = lastKnownTagDistance - inchesSinceSight;
            currentHeading = 0.0 - getHeading();
        }

        double rangeError = currentRange - TARGET_LAUNCH_DISTANCE;

        if (Math.abs(rangeError) < 1.2 && Math.abs(currentHeading) < 2.0) return true;

        double drive = Range.clip(rangeError * DRIVE_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
        double turn  = Range.clip(currentHeading * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);

        moveRobot(drive, turn);
        return false;
    }

    private void updateTagSighting() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        if (!detections.isEmpty()) {
            AprilTagDetection tag = detections.get(0);
            tagIsVisible = true;
            lastKnownTagDistance = tag.ftcPose.range;

            // CALCULATING ACCURATE BEARING USING FOCAL LENGTH
            // Using atan((x - center) / f) for high accuracy horizontal angle
            double xPixel = tag.center.x;
            lastKnownTagBearing = Math.toDegrees(Math.atan((xPixel - cameraCenterX) / focalLengthX));

            encoderAtLastSight = getCurrentEncoderAverage();
        } else {
            tagIsVisible = false;
        }
    }

    private void driveStraight(double speed, double inches, double targetAngle) {
        int startTicks = getCurrentEncoderAverage();
        int targetTicks = (int)(inches * TICKS_PER_INCH);
        while (opModeIsActive() && Math.abs(getCurrentEncoderAverage() - startTicks) < Math.abs(targetTicks)) {
            double error = targetAngle - getHeading();
            moveRobot(speed, Range.clip(error * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN));
        }
        stopRobot();
    }

    private void turnToAngle(double targetAngle) {
        while (opModeIsActive()) {
            double error = targetAngle - getHeading();
            if (Math.abs(error) < 1.5) break;
            moveRobot(0, Range.clip(error * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN));
        }
        stopRobot();
    }

    private int getCurrentEncoderAverage() {
        com.qualcomm.robotcore.hardware.DcMotor left = hardwareMap.get(com.qualcomm.robotcore.hardware.DcMotor.class, "ld");
        com.qualcomm.robotcore.hardware.DcMotor right = hardwareMap.get(com.qualcomm.robotcore.hardware.DcMotor.class, "rd");
        return (left.getCurrentPosition() + right.getCurrentPosition()) / 2;
    }

    private void moveRobot(double drive, double turn) {
        robot.moveRobot(drive - turn, drive + turn);
    }

    private double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    private void initIMU() {
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        imu.resetYaw();
    }

    private void initVision() {
        aprilTag = new AprilTagProcessor.Builder().build();
        visionPortal = new VisionPortal.Builder()
                .addProcessor(aprilTag)
                .setCameraResolution(new Size(cameraWidth, cameraHeight))
                .setCamera(BuiltinCameraDirection.FRONT)
                .build();
    }

    private void stopRobot() { robot.moveRobot(0, 0); }
}