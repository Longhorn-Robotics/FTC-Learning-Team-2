//import libraries
package org.firstinspires.ftc.teamcode;
import android.media.MediaPlayer;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import android.media.MediaPlayer;

import java.util.List;


//Declare teleop
@TeleOp(name = "Driver Control", group  = "Robot")



public class RobotTeleOp extends OpMode{
    //Get functions to move the robot
    RobotHardware robot = new RobotHardware();

    private MediaPlayer maxverstappan;

    //set up our camera (for driving automatically driving to the AprilTag)
    private static final int DESIRED_TAG_ID = -1;    // Choose the tag you want to approach or set to -1 for ANY tag.
    private int cameraWidth = 320;
    private int cameraHeight = 240;
    private double cameraCenterX = (cameraWidth - 1) / 2;
    private double cameraCenterY = (cameraHeight - 1) / 2;

    //moto e5 play selfie cmaerea (MAYBE)
    private double focalLengthX = 220.9;
    private double focalLengthY = 220.9;
    private AprilTagProcessor aprilTag = new AprilTagProcessor.Builder()
            .setLensIntrinsics(focalLengthX, focalLengthY, cameraCenterX, cameraCenterY)
            // ... these parameters are fx, fy, cx, cy.
            .build();
    private VisionPortal visionPortal = new VisionPortal.Builder()
            .addProcessor(aprilTag)
            .setCameraResolution(new Size(cameraWidth, cameraHeight))
            .enableLiveView(true)
            .setCamera(BuiltinCameraDirection.FRONT)
            //.setCamera(hardwareMap.get(WebcamName.class, "cameraa"))
            .build();

    //init our hardware
    @Override
    public void init() {

        maxverstappan = MediaPlayer.create(hardwareMap.appContext, R.raw.maxverstappan);
        maxverstappan.setLooping(false);
        robot.init(hardwareMap);
        maxverstappan.start();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }


    //Define variables for reading the game controller
    double LStickY = 0;
    double RStickY = 0;
    boolean RBumper = false;
    boolean LBumper = false;


    @Override
    //run our linear op mode
    public void loop() {
        //read controller data
        LStickY = -this.gamepad1.left_stick_y;
        RStickY = -this.gamepad1.right_stick_y;
        RBumper = this.gamepad1.right_bumper;
        LBumper = this.gamepad1.left_bumper;

        if (LBumper) {
            driveToAprilTag(12);
        }

        //run commands on on the robot
        if (RBumper) {
            robot.idleLauncher();
        }
        else {
            robot.launchItems(1);
        }
        robot.moveRobot(LStickY, RStickY);
        robot.runIntake();


        //record telemetry data
        telemetry.addData("Left Stick Y", LStickY);
        telemetry.addData("Right Stick Y", RStickY);
        telemetry.addData("Launching", RBumper);
        telemetry.addData("Status", "Running");
        System.out.println("Running");
        //.runDetection();
        telemetry.update();
    }

    @Override
    public void stop () {

    }





    //Drive to the apil tag for proper allignment while shooting
    private void driveToAprilTag (double DESIRED_DISTANCE){
        boolean targetFound = false;
        AprilTagDetection desiredTag  = null;
        double rangeError;
        double headingError;
        String progress = "Driving";
        double  drive = 0;        // Desired forward power/speed (-1 to +1) +ve is forward
        double  turn = 0;        // Desired turning power/speed (-1 to +1) +ve is CounterClockwise
        final double SPEED_GAIN =   0.02 ;   //  Speed Control "Gain". e.g. Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
        final double TURN_GAIN  =   0.01 ;   //  Turn Control "Gain".  e.g. Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
        final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
        final double MAX_AUTO_TURN  = 0.25;  //  Clip the turn speed to this max value (adjust for your robot)

        // Step through the list of detected tags and look for a matching tag
        while (progress.equals("Driving")){
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                if (detection.metadata != null) {
                    //  Check to see if we want to track towards this tag.
                    if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                        // Yes, we want to use this tag.
                        targetFound = true;
                        desiredTag = detection;
                        break;  // don't look any further.
                    } else {
                        // This tag is in the library, but we do not want to track it right now.
                    }
                } else {
                    // This tag is NOT in the library, so we don't have enough information to track to it.
                }
            }

            telemetry.update();
            if (targetFound) {
                // Determine heading and range error so we can use them to control the robot automatically.
                rangeError   = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                //When the phone is sideways
//                headingError = desiredTag.ftcPose.bearing;
                //When the phone is the right way around
                headingError = desiredTag.ftcPose.elevation;
                telemetry.addData("Distance", rangeError);
                telemetry.addData("heading", headingError);
                telemetry.update();
                // Use the speed and turn "gains" to calculate how we want the robot to move.  Clip it to the maximum
                drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                turn  = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
                telemetry.addData("progress", "calculated");
                telemetry.update();

//                telemetry.addData("Auto","Drive %5.2f, Turn %5.2f", drive, turn);
//                telemetry.update();
            } else {
                progress = "No AprilTag!";
                return;
            }
//            telemetry.update();
            telemetry.addData("progress", "updated");
            telemetry.update();

            // Apply desired axes motions to the drivetrain.
            if (rangeError < 3 && rangeError > -3){
                progress = "Done";
                moveRobot(0,0);
                return;
            }
            else {
                telemetry.addData("progress", "Driving");
                telemetry.update();
                moveRobot(drive, turn);
//                sleep(300);
//                moveRobot(0,0);
                progress = "Driving";
                //return "Driving";
            }
        }
    }

    private void moveRobot(double x, double yaw) {
        // Calculate left and right wheel powers.
        double leftPower    = x - yaw;
        double rightPower   = x + yaw;


        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftPower), Math.abs(rightPower));

        if (max >1.0) {
            leftPower /= max;
            rightPower /= max;
        }
        // Send powers to the wheels.
        robot.moveRobot(leftPower, rightPower);


    }

}
