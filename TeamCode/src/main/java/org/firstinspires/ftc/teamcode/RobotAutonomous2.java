
//import libraries
package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.SortOrder;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.opencv.Circle;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import android.util.Size;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "Auto V8 - Correct Artifact Detection", group = "Robot")
public class RobotAutonomous2 extends LinearOpMode {

    //==============================================================================================
    // =========   TUNING VARIABLES   =========
    //==============================================================================================

    // === LENS INTRINSICS & CAMERA ===
    final double focalLengthX = 220.9;
    final double focalLengthY = 220.9;
    final int cameraWidth = 320;
    final int cameraHeight = 240;
    final double cameraCenterX = (cameraWidth - 1) / 2.0;
    final double cameraCenterY = (cameraHeight - 1) / 2.0;
    final double averageFocalLength = (focalLengthX + focalLengthY) / 2.0;
    final double ballRadius = 2.5; // inches

    // === APRIL TAG ===
    private static final int HOOP_APRILTAG_ID = -1;
    final double DISTANCE_TO_SCORE = 12.0;
    final double TARGET_YAW_DEGREES = -45.0;

    // === AUTONOMOUS PATHING ===
    final double[] ROW_BACKUP_DISTANCES = { 36.0, 48.0, 60.0, 72.0 };
    final double COLLECTION_DRIVE_DISTANCE = 24.0;

    // === GAINS & SPEED ===
    final double SPEED_GAIN = 0.02;
    final double TURN_GAIN = 0.01;
    final double YAW_GAIN = 0.015;
    final double ARTIFACT_STEERING_GAIN = 0.01;
    final double MAX_AUTO_SPEED = 0.5;
    final double MAX_AUTO_TURN = 0.25;

    //==============================================================================================
    // =========   STATE MACHINE   =========
    //==============================================================================================

    private enum AutoState {
        START, DRIVE_TO_HOOP, PERFORM_LAUNCH, DRIVE_BACKWARD_FROM_HOOP,
        TURN_FOR_COLLECTION, DRIVE_AND_COLLECT, TURN_FOR_RETURN, END_AUTONOMOUS
    }

    private AutoState currentState = AutoState.START;
    private int currentRow = 0;

    private RobotHardware2 robot = new RobotHardware2();
    private AprilTagProcessor aprilTag;
    private ColorBlobLocatorProcessor colorLocatorPurple;
    private ColorBlobLocatorProcessor colorLocatorGreen;
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        initVision();

        telemetry.addData("Status", "Initialization Complete");
        telemetry.addData(">", "Press Play to start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive() && currentState != AutoState.END_AUTONOMOUS) {
            telemetry.addData("Current State", currentState.toString());
            telemetry.addData("Targeting Row", currentRow + 1);

            switch (currentState) {
                case START:
                    setVisionProcessors(true, false, false); // Enable AprilTag, disable artifacts
                    currentState = AutoState.DRIVE_TO_HOOP;
                    break;

                case DRIVE_TO_HOOP:
                    if (driveToAprilTag(DISTANCE_TO_SCORE)) {
                        currentState = AutoState.PERFORM_LAUNCH;
                    }
                    break;

                case PERFORM_LAUNCH:
                    telemetry.addLine("Placeholder for LAUNCH code.");
                    sleep(1000); // Placeholder

                    if (currentRow < ROW_BACKUP_DISTANCES.length) {
                        currentState = AutoState.DRIVE_BACKWARD_FROM_HOOP;
                    } else {
                        currentState = AutoState.END_AUTONOMOUS;
                    }
                    break;

                case DRIVE_BACKWARD_FROM_HOOP:
                    if (driveBackwardsFromAprilTag(ROW_BACKUP_DISTANCES[currentRow])) {
                        currentState = AutoState.TURN_FOR_COLLECTION;
                    }
                    break;

                case TURN_FOR_COLLECTION:
                    encoderTurn(90, 0.3);
                    currentState = AutoState.DRIVE_AND_COLLECT;
                    break;

                case DRIVE_AND_COLLECT:
                    robot.runIntake(1.0);
                    driveAndCollectHybrid(COLLECTION_DRIVE_DISTANCE, 0.5);
                    robot.runIntake(0.0);
                    currentState = AutoState.TURN_FOR_RETURN;
                    break;

                case TURN_FOR_RETURN:
                    encoderTurn(-90, 0.3);
                    currentRow++;
                    currentState = AutoState.DRIVE_TO_HOOP;
                    break;

                case END_AUTONOMOUS:
                    robot.moveRobot(0, 0);
                    break;
            }
            telemetry.update();
        }
        robot.moveRobot(0, 0);
    }

    private boolean driveToAprilTag(double desiredDistance) {
        AprilTagDetection desiredTag = getDesiredAprilTag();
        if (desiredTag == null) {
            moveRobot(0, -MAX_AUTO_TURN);
            return false;
        }

        double rangeError = (desiredTag.ftcPose.range - desiredDistance);
        double headingError = desiredTag.ftcPose.bearing;
        telemetry.addData("Range Error: %.2f", rangeError);
        telemetry.addData("Heading Error: %.2f", headingError);
        telemetry.update();
        double drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
        double turn  = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);

        moveRobot(drive, turn);
        return (Math.abs(rangeError) < 10 && Math.abs(headingError) < 7);
    }

    private boolean driveBackwardsFromAprilTag(double backupDistance) {
        AprilTagDetection desiredTag = getDesiredAprilTag();
        if (desiredTag == null) {
            robot.moveRobot(0, 0);
            return false;
        }
        double rangeError = (desiredTag.ftcPose.range - backupDistance);
        double yawError = (desiredTag.ftcPose.yaw - TARGET_YAW_DEGREES);

        double drive = Range.clip(-rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
        double turn = Range.clip(yawError * YAW_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);

        moveRobot(drive, turn);
        return (Math.abs(rangeError) < 1.0 && Math.abs(yawError) < 3.0);
    }

    private void driveAndCollectHybrid(double inches, double speed) {
        setVisionProcessors(false, true, true); // Disable AprilTag, enable artifacts

        int leftTarget = robot.LDriveMotor.getCurrentPosition() + (int)(inches * RobotHardware2.COUNTS_PER_INCH);
        int rightTarget = robot.RDriveMotor.getCurrentPosition() + (int)(inches * RobotHardware2.COUNTS_PER_INCH);
        robot.LDriveMotor.setTargetPosition(leftTarget);
        robot.RDriveMotor.setTargetPosition(rightTarget);
        robot.LDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.RDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.moveRobot(Math.abs(speed), Math.abs(speed));

        while (opModeIsActive() && (robot.LDriveMotor.isBusy() || robot.RDriveMotor.isBusy())) {
            double turn = 0;
            List<int[]> artifacts = findArtifacts();
            if (!artifacts.isEmpty()) {
                int[] closestArtifact = artifacts.get(0);
                double[] ballPosition = getBallPosition(closestArtifact[0], closestArtifact[1], closestArtifact[2]);
                double headingError = ballPosition[1];
                turn = Range.clip(headingError * ARTIFACT_STEERING_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
            }
            moveRobot(speed, turn);
            telemetry.addData("Path", "Collecting... Angle Correction: %.2f", turn);
            telemetry.update();
        }

        robot.moveRobot(0, 0);
        robot.LDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.RDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        setVisionProcessors(true, false, false); // Re-enable AprilTag for return trip
    }

    private void moveRobot(double x, double yaw) {
        double leftPower    = x - yaw;
        double rightPower   = x + yaw;


        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftPower), Math.abs(rightPower));

//        if (x<0) {
//            max *= -1;
//        }
        if (max >1.0) {
            leftPower /= max;
            rightPower /= max;
        }
        // Send powers to the wheels.
        robot.moveRobot(leftPower, rightPower);
    }

    public void encoderTurn(double degrees, double speed) {
        int turnTicks = (int) (degrees * RobotHardware2.COUNTS_PER_DEGREE_TURN);
        int leftTarget = robot.LDriveMotor.getCurrentPosition() + turnTicks;
        int rightTarget = robot.RDriveMotor.getCurrentPosition() - turnTicks;

        robot.LDriveMotor.setTargetPosition(leftTarget);
        robot.RDriveMotor.setTargetPosition(rightTarget);
        robot.LDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.RDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.moveRobot(Math.abs(speed), Math.abs(speed));

        while (opModeIsActive() && (robot.LDriveMotor.isBusy() || robot.RDriveMotor.isBusy())) {
            telemetry.addData("Path", "Turning %f degrees", degrees);
            telemetry.update();
        }
        robot.moveRobot(0, 0);
        robot.LDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.RDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private AprilTagDetection getDesiredAprilTag() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null && (detection.id == HOOP_APRILTAG_ID || HOOP_APRILTAG_ID < 0)) {
                return detection;
            }
        }
        return null;
    }

    public List<int[]> findArtifacts() {
        List<ColorBlobLocatorProcessor.Blob> blobsPurple = colorLocatorPurple.getBlobs();
        List<ColorBlobLocatorProcessor.Blob> blobsGreen = colorLocatorGreen.getBlobs();
        List<ColorBlobLocatorProcessor.Blob> blobs = new ArrayList<>();
        blobs.addAll(blobsPurple);
        blobs.addAll(blobsGreen);

        ColorBlobLocatorProcessor.Util.filterByCriteria(ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, 50, 20000, blobs);
        ColorBlobLocatorProcessor.Util.filterByCriteria(ColorBlobLocatorProcessor.BlobCriteria.BY_CIRCULARITY, 0.6, 1, blobs);
        ColorBlobLocatorProcessor.Util.sortByCriteria(ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, SortOrder.DESCENDING, blobs);

        List<int[]> circlesList = new ArrayList<>();
        for (ColorBlobLocatorProcessor.Blob b : blobs) {
            Circle circleFit = b.getCircle();
            circlesList.add(new int[]{(int) circleFit.getX(), (int) circleFit.getY(), (int) circleFit.getRadius()});
        }
        return circlesList;
    }

    private double[] getBallPosition(int xPixel, int yPixel, int rPixel) {
        double distance = (ballRadius * averageFocalLength) / rPixel;
        double hAngle = Math.toDegrees(Math.atan((xPixel - cameraCenterX) / focalLengthX));
        return new double[]{distance, hAngle};
    }

    private void initVision() {
        aprilTag = new AprilTagProcessor.Builder()
                .setLensIntrinsics(focalLengthX, focalLengthY, cameraCenterX, cameraCenterY)
                .build();

        colorLocatorPurple = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.ARTIFACT_PURPLE)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.asUnityCenterCoordinates(-1, 1, 1, -1))
                .setDrawContours(true)
                .setBoxFitColor(0)
                .setCircleFitColor(Color.rgb(255, 255, 0))
                .setBlurSize(5)
                .setDilateSize(15)
                .setErodeSize(15)
                .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)
                .build();

        colorLocatorGreen = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.ARTIFACT_GREEN)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.asUnityCenterCoordinates(-1, 1, 1, -1))
                .setDrawContours(true)
                .setBoxFitColor(0)
                .setCircleFitColor(Color.rgb(255, 255, 0))
                .setBlurSize(5)
                .setDilateSize(15)
                .setErodeSize(15)
                .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(BuiltinCameraDirection.FRONT)
                .setCameraResolution(new Size(cameraWidth, cameraHeight))
                .addProcessors(aprilTag, colorLocatorPurple, colorLocatorGreen)
                .build();
    }

    private void setVisionProcessors(boolean enableAprilTag, boolean enablePurple, boolean enableGreen) {
        visionPortal.setProcessorEnabled(aprilTag, enableAprilTag);
        visionPortal.setProcessorEnabled(colorLocatorPurple, enablePurple);
        visionPortal.setProcessorEnabled(colorLocatorGreen, enableGreen);
    }
}
