
//import libraries
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import android.util.Size;

import java.util.List;

@Autonomous(name = "Auto V3 - State Machine - Corrected", group = "Robot")
public class RobotAutonomous2 extends LinearOpMode {

    //==============================================================================================
    // =========   TUNING VARIABLES   =========
    //==============================================================================================

    // === CAMERA & APRIL TAG ===
    //private static final int HOOP_APRILTAG_ID = 5;
    private static final int HOOP_APRILTAG_ID = -1;
    final double DISTANCE_TO_SCORE = 12.0; // Inches
    final double TARGET_YAW_DEGREES = -45.0;

    // === AUTONOMOUS PATHING ===
    final double[] ROW_BACKUP_DISTANCES = { 36.0, 48.0, 60.0, 72.0 }; // Inches
    final double COLLECTION_DRIVE_DISTANCE = 24.0; // Inches

    // === GAINS & SPEED ===
    final double SPEED_GAIN = 0.02;
    final double TURN_GAIN = 0.015;
    final double YAW_GAIN = 0.02;
    final double MAX_AUTO_SPEED = 0.5;
    final double MAX_AUTO_TURN = 0.3;

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
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        initAprilTag();

        telemetry.addData("Status", "Initialization Complete");
        telemetry.addData(">", "Press Play to start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive() && currentState != AutoState.END_AUTONOMOUS) {
            telemetry.addData("Current State", currentState.toString());
            telemetry.addData("Targeting Row", currentRow + 1);

            switch (currentState) {
                case START:
                    currentState = AutoState.DRIVE_TO_HOOP;
                    break;

                case DRIVE_TO_HOOP:
                    if (driveToAprilTag(DISTANCE_TO_SCORE)) {
                        currentState = AutoState.PERFORM_LAUNCH;
                    }
                    break;

                case PERFORM_LAUNCH:
                    // *** YOUR LAUNCHER CODE GOES HERE ***
                    telemetry.addLine("Placeholder for LAUNCH code.");
                    sleep(1000); // Placeholder for launch time

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
                    encoderTurn(90, 0.3); // Turn RIGHT
                    currentState = AutoState.DRIVE_AND_COLLECT;
                    break;

                case DRIVE_AND_COLLECT:
                    robot.runIntake(1.0);
                    encoderDrive(COLLECTION_DRIVE_DISTANCE, 0.5);
                    robot.runIntake(0.0);
                    currentState = AutoState.TURN_FOR_RETURN;
                    break;

                case TURN_FOR_RETURN:
                    encoderTurn(-90, 0.3); // Turn LEFT
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
            robot.moveRobot(0, MAX_AUTO_TURN * 0.5); // Turn right to find tag
            telemetry.addLine("No AprilTag detected.");
            telemetry.update();
            return false;
        }

        double rangeError = (desiredTag.ftcPose.range - desiredDistance);
        double headingError = desiredTag.ftcPose.bearing;

        double drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
        double turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);

        robot.moveRobot(drive + turn, drive - turn);

        return (Math.abs(rangeError) < 0.5 && Math.abs(headingError) < 2.0);
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

        robot.moveRobot(drive + turn, drive - turn);

        return (Math.abs(rangeError) < 1.0 && Math.abs(yawError) < 3.0);
    }

    public void encoderDrive(double inches, double speed) {
        int leftTarget = robot.LDriveMotor.getCurrentPosition() + (int)(inches * RobotHardware2.COUNTS_PER_INCH);
        int rightTarget = robot.RDriveMotor.getCurrentPosition() + (int)(inches * RobotHardware2.COUNTS_PER_INCH);

        robot.LDriveMotor.setTargetPosition(leftTarget);
        robot.RDriveMotor.setTargetPosition(rightTarget);

        robot.LDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.RDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.moveRobot(Math.abs(speed), Math.abs(speed));

        while (opModeIsActive() && robot.LDriveMotor.isBusy() && robot.RDriveMotor.isBusy()) {
            telemetry.addData("Path", "Driving %f inches", inches);
            telemetry.update();
        }

        robot.moveRobot(0, 0);
        robot.LDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.RDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void encoderTurn(double degrees, double speed) {
        // Positive degrees = CW/Right turn. Negative degrees = CCW/Left turn.
        int turnTicks = (int) (degrees * RobotHardware2.COUNTS_PER_DEGREE_TURN);

        int leftTarget = robot.LDriveMotor.getCurrentPosition() + turnTicks;
        int rightTarget = robot.RDriveMotor.getCurrentPosition() - turnTicks;

        robot.LDriveMotor.setTargetPosition(leftTarget);
        robot.RDriveMotor.setTargetPosition(rightTarget);

        robot.LDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.RDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.moveRobot(Math.abs(speed), Math.abs(speed));

        while (opModeIsActive() && robot.LDriveMotor.isBusy() && robot.RDriveMotor.isBusy()) {
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
            if (detection.metadata != null && detection.id == HOOP_APRILTAG_ID || HOOP_APRILTAG_ID < 0) {
                return detection;
            }
        }
        return null;
    }

    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder().build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(BuiltinCameraDirection.FRONT)
                .setCameraResolution(new Size(640, 480))
                .addProcessor(aprilTag)
                .enableLiveView(true)
                .build();
    }
}
