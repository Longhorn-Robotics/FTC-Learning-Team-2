package org.firstinspires.ftc.teamcode;

import android.util.Size;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name = "Robot Auto 3 - Fusion & Debug", group = "Robot")
public class RobotAutonomous3 extends LinearOpMode {

    /* --- HARDWARE & CONSTANTS --- */
    final double TICKS_PER_INCH = 45.28;
    final double TARGET_LAUNCH_DISTANCE = 24.0;

    /* --- TUNING CONTROLS (Adjust these to stop spinning) --- */
    final double DRIVE_GAIN = 0.04;       // How aggressively to drive to distance
    final double TURN_GAIN  = 0.02;       // How aggressively to turn to heading (Lower this if spinning)
    final double MAX_AUTO_SPEED = 0.4;    // Maximum drive power
    final double MAX_AUTO_TURN  = 0.25;   // Maximum turn power (Keep this low to avoid overshooting)

    /* --- CAMERA & ESTIMATION CONSTANTS --- */
    private double ballRadius = 2.5;
    private int cameraWidth = 320;
    private int cameraHeight = 240;
    private double cameraCenterX = (cameraWidth - 1) / 2.0;
    private double cameraCenterY = (cameraHeight - 1) / 2.0;

    // Moto E5 Play Selfie Camera constants
    private double focalLengthX = 220.9;
    private double focalLengthY = 220.9;
    private double averageFocalLength = (focalLengthX + focalLengthY) / 2.0;

    // Memory for Sensor Fusion
    private double lastKnownTagDistance = 0;
    private double lastKnownTagBearing  = 0;
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
            telemetryDebug("WAITING FOR START");
            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive()) {
            updateTagSighting();
            telemetryDebug("EXECUTING AUTO");

            switch (currentState) {
                case START_LAUNCH:
                    robot.launchItems(0.8);
                    sleep(1500);
                    robot.idleLauncher(0);
                    currentState = State.MOVE_TO_ROW;
                    break;

                case MOVE_TO_ROW:
                    // STEP 2: Drive backwards away from the tag to a specific row
                    // targetRowDist here represents the desired absolute distance from the tag
                    double targetRowDist = 48.0;
                    driveToTagDistance(targetRowDist, 0.0);
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
                    // STEP 6: Drive back towards the scoring zone
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
     * Drives to a specific absolute distance from the AprilTag.
     * Speed is calculated proportionally to the distance error.
     */
    private void driveToTagDistance(double targetDistance, double targetHeading) {
        while (opModeIsActive()) {
            updateTagSighting();

            double currentRange;
            if (tagIsVisible) {
                currentRange = lastKnownTagDistance;
            } else {
                // Fallback: estimate current range based on encoder travel since last sighting
                double inchesTraveledSinceSight = (getCurrentEncoderAverage() - encoderAtLastSight) / TICKS_PER_INCH;
                // If we were moving forward, distance decreases; if backward, it increases.
                // This is a simplified estimation.
                currentRange = lastKnownTagDistance - inchesTraveledSinceSight;
            }

            double rangeError = currentRange - targetDistance;

            // Check if we are close enough to the target distance
            if (Math.abs(rangeError) < 1.0) break;

            // Calculate proportional speed (Drive Gain)
            double speed = Range.clip(rangeError * DRIVE_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);

            // Calculate heading error
            double error;
            if (tagIsVisible) {
                error = lastKnownTagBearing;
            } else {
                error = targetHeading - getHeading();
            }

            double turn = Range.clip(error * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);

            moveRobot(speed, turn);

            telemetryDebug("DRIVING TO TAG DISTANCE");
            telemetry.addData("Target Dist", "%.1f", targetDistance);
            telemetry.addData("Current Est Dist", "%.1f", currentRange);
            telemetry.addData("Speed", "%.2f", speed);
            telemetry.update();
        }
        stopRobot();
    }

    private void driveWithTagAnchor(double speed, double travelInches, double targetHeading) {
        int startTicks = getCurrentEncoderAverage();
        int targetTicks = (int)(Math.abs(travelInches) * TICKS_PER_INCH);
        double startTagDist = tagIsVisible ? lastKnownTagDistance : 0;

        while (opModeIsActive()) {
            updateTagSighting();
            int currentTicks = Math.abs(getCurrentEncoderAverage() - startTicks);
            double encoderInchesMoved = currentTicks / TICKS_PER_INCH;

            boolean reachedDestination = false;
            if (tagIsVisible && startTagDist != 0) {
                double visualDistMoved = Math.abs(lastKnownTagDistance - startTagDist);
                if (visualDistMoved >= Math.abs(travelInches)) reachedDestination = true;
            } else {
                if (encoderInchesMoved >= Math.abs(travelInches)) reachedDestination = true;
            }

            if (reachedDestination) break;

            double error;
            if (tagIsVisible && encoderInchesMoved > 5.0) {
                error = lastKnownTagBearing;
            } else {
                error = targetHeading - getHeading();
            }

            // Using the new TURN_GAIN and MAX_AUTO_TURN
            double turn = Range.clip(error * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
            moveRobot(speed, turn);

            telemetryDebug("DRIVING WITH TAG ANCHOR");
            telemetry.update();
        }
        stopRobot();
    }

    private boolean alignWithFusion() {
        double currentRange;
        double currentBearing;

        if (tagIsVisible) {
            currentRange = lastKnownTagDistance;
            currentBearing = lastKnownTagBearing;
        } else {
            double inchesTraveledSinceSight = (getCurrentEncoderAverage() - encoderAtLastSight) / TICKS_PER_INCH;
            currentRange = lastKnownTagDistance - inchesTraveledSinceSight;
            currentBearing = 0.0 - getHeading();
        }

        double rangeError = currentRange - TARGET_LAUNCH_DISTANCE;
        if (Math.abs(rangeError) < 1.2 && Math.abs(currentBearing) < 2.0) return true;

        // Using the new GAINS and MAX limits
        double drive = Range.clip(rangeError * DRIVE_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
        double turn  = Range.clip(currentBearing * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);

        moveRobot(drive, turn);
        return false;
    }

    private void updateTagSighting() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        if (!detections.isEmpty()) {
            AprilTagDetection tag = detections.get(0);
            tagIsVisible = true;
            lastKnownTagDistance = tag.ftcPose.range;
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
            updateTagSighting();
            double error = targetAngle - getHeading();
            double turn = Range.clip(error * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
            moveRobot(speed, turn);
            telemetryDebug("DRIVING STRAIGHT");
            telemetry.update();
        }
        stopRobot();
    }

    private void turnToAngle(double targetAngle) {
        while (opModeIsActive()) {
            double error = targetAngle - getHeading();
            if (Math.abs(error) < 1.5) break;
            moveRobot(0, Range.clip(error * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN));
            telemetryDebug("TURNING");
            telemetry.update();
        }
        stopRobot();
    }

    private int getCurrentEncoderAverage() {
        com.qualcomm.robotcore.hardware.DcMotor left = hardwareMap.get(com.qualcomm.robotcore.hardware.DcMotor.class, "ld");
        com.qualcomm.robotcore.hardware.DcMotor right = hardwareMap.get(com.qualcomm.robotcore.hardware.DcMotor.class, "rd");
        return (left.getCurrentPosition() + right.getCurrentPosition()) / 2;
    }

    private void telemetryDebug(String status) {
        telemetry.addLine("--- ROBOT LOGS ---");
        telemetry.addData("Status", status);
        telemetry.addData("State", currentState);
        telemetry.addData("Heading", "%.1f deg", getHeading());

        telemetry.addLine("\n--- VISION & FUSION ---");
        telemetry.addData("Tag Visible", tagIsVisible ? "YES" : "NO (USING BACKUP)");
        telemetry.addData("Distance", "%.2f in", lastKnownTagDistance);
        telemetry.addData("Angle (hAngle)", "%.2f deg", lastKnownTagBearing);

        if (!tagIsVisible) {
            double estTravel = (getCurrentEncoderAverage() - encoderAtLastSight) / TICKS_PER_INCH;
            telemetry.addData("Est. Travel Since Sight", "%.1f in", estTravel);
        }
    }

    private void moveRobot(double x, double yaw) {
        robot.moveRobot(x - yaw, x + yaw);
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
        aprilTag = new AprilTagProcessor.Builder()
                .setLensIntrinsics(focalLengthX, focalLengthY, cameraCenterX, cameraCenterY)
                .build();

        visionPortal = new VisionPortal.Builder()
                .addProcessor(aprilTag)
                .setCameraResolution(new Size(cameraWidth, cameraHeight))
                .setCamera(BuiltinCameraDirection.FRONT)
                .build();
    }

    private void stopRobot() { robot.moveRobot(0, 0); }
}