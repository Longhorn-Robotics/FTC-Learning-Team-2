//import libraries
package org.firstinspires.ftc.teamcode;
import android.graphics.Color;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.SortOrder;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.opencv.Circle;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;
import java.lang.Math;


//Declare teleop
@Autonomous(name = "Auto Control", group  = "Robot")


//init and run our teleop
public class RobotAutonomous extends LinearOpMode {




    //  Set the GAIN constants to controlxx the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN =   0.02 ;   //  Speed Control "Gain". e.g. Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double TURN_GAIN  =   0.01 ;   //  Turn Control "Gain".  e.g. Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 1;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 1;  //  Clip the turn speed to this max value (adjust for your robot)

//    private DcMotor leftDrive   = null;  //  Used to control the left drive wheel
//    private DcMotor rightDrive  = null;  //  Used to control the right drive wheel

    private static final boolean USE_WEBCAM = false;  // Set true to use a webcam, or false for a phone camera
    private static final int DESIRED_TAG_ID = -1;    // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;



    //IN INCHES
    private int cameraWidth = 320;
    private int cameraHeight = 240;
    private double ballRadius = 2;
    //The x and y focal lengths of the camera
    private double xf = 251.75;
    private double yf = 251.75;
    //the average of the focal lengths
    private double ef = (xf + yf) / 2;
    private double cx = (cameraWidth - 1) / 2;
    private double cy = (cameraHeight - 1) / 2;



    boolean targetFound     = false;    // Set to true when an AprilTag target is detected
    double  drive           = 0;        // Desired forward power/speed (-1 to +1) +ve is forward
    double  turn            = 0;        // Desired turning power/speed (-1 to +1) +ve is CounterClockwise
    String progress;

    // Initialize the ball Detection process
    private ColorBlobLocatorProcessor colorLocatorPurple = new ColorBlobLocatorProcessor.Builder()
            .setTargetColorRange(ColorRange.ARTIFACT_PURPLE)   // Use a predefined color match
            .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
            .setRoi(ImageRegion.asUnityCenterCoordinates(-0.75, 0.75, 0.75, -0.75))
            .setDrawContours(true)   // Show contours on the Stream Preview
            .setBoxFitColor(0)       // Disable the drawing of rectangles
            .setCircleFitColor(Color.rgb(255, 255, 0)) // Draw a circle
            .setBlurSize(5)          // Smooth the transitions between different colors in image

            // the following options have been added to fill in perimeter holes.
            .setDilateSize(15)       // Expand blobs to fill any divots on the edges
            .setErodeSize(15)        // Shrink blobs back to original size
            .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)

            .build();


    private ColorBlobLocatorProcessor colorLocatorGreen = new ColorBlobLocatorProcessor.Builder()
            .setTargetColorRange(ColorRange.ARTIFACT_GREEN)   // Use a predefined color match
            .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
            .setRoi(ImageRegion.asUnityCenterCoordinates(-0.75, 0.75, 0.75, -0.75))
            .setDrawContours(true)   // Show contours on the Stream Preview
            .setBoxFitColor(0)       // Disable the drawing of rectangles
            .setCircleFitColor(Color.rgb(255, 255, 0)) // Draw a circle
            .setBlurSize(5)          // Smooth the transitions between different colors in image

            // the following options have been added to fill in perimeter holes.
            .setDilateSize(15)       // Expand blobs to fill any divots on the edges
            .setErodeSize(15)        // Shrink blobs back to original size
            .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)

            .build();
    /*
     * Build a vision portal to run the Color Locator process.
     *
     *  - Add the colorLocator process created above.
     *  - Set the desired video resolution.
     *      Since a high resolution will not improve this process, choose a lower resolution
     *      that is supported by your camera.  This will improve overall performance and reduce
     *      latency.
     *  - Choose your video source.  This may be
     *      .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))  .....   for a webcam
     *  or
     *      .setCamera(BuiltinCameraDirection.BACK)    ... for a Phone Camera
     */
    VisionPortal portal = new VisionPortal.Builder()
            .addProcessor(colorLocatorPurple)
            .addProcessor(colorLocatorGreen)
            .setCameraResolution(new Size(cameraWidth, cameraHeight))
            //.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
            .setCamera(BuiltinCameraDirection.BACK)
            .build();



    //Access our robot hardware
    RobotHardware robot = new RobotHardware();

    @Override
    public void runOpMode() {
        //init hardware
        robot.init(hardwareMap);
        initAprilTag();
        //getPos.initAprilTag();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        sleep(10000);

        String state = "goToLaunchZone1";

        waitForStart();

        while (opModeIsActive()) {


            switch (state) {
                case "goToLaunchZone1":
                    //double[] position = getPos.runDetection();
                    //driveApril.runOpMode(hardwareMap);
                    progress = driveToAprilTag(12);
                    if (progress == "Done") {
                        state = "GoToLaunchZone1";
                    }
                    break;
                case "goToBallRow1":
                    driveToBallRow(1);
                    break;
                default:
                    break;
            }



        }
    }

    private void driveToBallRow(int rowNumber) {
        driveToAprilTag(46);
        while (true) {
            robot.moveRobot(0.5, -0.5);
        }

    }

    private String driveToAprilTag (double DESIRED_DISTANCE){
        targetFound = false;
        desiredTag  = null;

        // Step through the list of detected tags and look for a matching tag
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

        if (targetFound) {
            // Determine heading and range error so we can use them to control the robot automatically.
            double  rangeError   = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
            double  headingError = desiredTag.ftcPose.bearing;
            // Use the speed and turn "gains" to calculate how we want the robot to move.  Clip it to the maximum
            drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            turn  = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;

            telemetry.addData("Auto","Drive %5.2f, Turn %5.2f", drive, turn);
        } else {
            return "No AprilTag!";
        }
        telemetry.update();

        // Apply desired axes motions to the drivetrain.
        if (drive < 3){
            return "Done";
        }
        else {
            moveRobot(drive, turn);
            sleep(10);
            return "Driving";
        }
    }




    public void moveRobot(double x, double yaw) {
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





    private void initAprilTag() {
        if (USE_WEBCAM)
            setManualExposure(6, 250);
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // e.g. Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2);

        // Create the vision portal by using a builder.
        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }

    //Manually set the camera gain and exposure. This can only be called AFTER calling initAprilTag(), and only works for Webcams;
    private void    setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls
        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }
    }






























    public List<int[]> findArtifacts() {
        List<ColorBlobLocatorProcessor.Blob> blobsPurple = colorLocatorPurple.getBlobs();
        List<ColorBlobLocatorProcessor.Blob> blobsGreen = colorLocatorGreen.getBlobs();
        List<ColorBlobLocatorProcessor.Blob> blobs = new ArrayList<>();
        blobs.addAll(blobsPurple);
        blobs.addAll(blobsGreen);

            ColorBlobLocatorProcessor.Util.filterByCriteria(
                    ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA,
                    50, 20000, blobs);  // filter out very small blobs.
            ColorBlobLocatorProcessor.Util.filterByCriteria(
                    ColorBlobLocatorProcessor.BlobCriteria.BY_CIRCULARITY,
                    0.6, 1, blobs);
            ColorBlobLocatorProcessor.Util.sortByCriteria(
                    ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, SortOrder.DESCENDING, blobs);


            telemetry.addLine("Circularity Radius Center");
            List<int[]> circlesList = new ArrayList<>();
            for (ColorBlobLocatorProcessor.Blob b : blobs) {
                Circle circleFit = b.getCircle();
                int[] circleData = {(int) circleFit.getX(), (int)circleFit.getY(), (int)circleFit.getRadius()};
                circlesList.add(circleData);
            }
        return circlesList;
    }







    private String driveToClosestBall (double DESIRED_DISTANCE){
        targetFound = false;
        desiredTag  = null;

        // Step through the list of detected tags and look for a matching tag
        List<int[]> currentDetections = findArtifacts();
        if (currentDetections.size() < 1) {
            return "No Balls!";
        }

        // Determine heading and range error so we can use them to control the robot automatically.
        int [] closestBall = currentDetections.get(0);
        double[] ballLocation = getBallPosition(closestBall[0], closestBall[1], closestBall[2]);
        double  rangeError   = (ballLocation[0] - DESIRED_DISTANCE);
        double  headingError = ballLocation[1];
        // Use the speed and turn "gains" to calculate how we want the robot to move.  Clip it to the maximum
        drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
        turn  = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;

        // Apply desired axes motions to the drivetrain.
        if (drive < 3){
            return "Done";
        }
        else {
            moveRobot(drive, turn);
            sleep(10);
            return "Driving";
        }
    }


    private double[] getBallPosition(int xPixel, int yPixel, int rPixel) {

        double distance = (ballRadius * ef) / rPixel;

        //Horizontal angle
        double hAngle = Math.toDegrees(Math.atan(xPixel - cx) / xf);
        //double vAngle = Math.toDegrees(Math.atan(yPixel - cy) / yf);
        double[] returnedData = {distance, hAngle};
        return returnedData;


    }
}





















