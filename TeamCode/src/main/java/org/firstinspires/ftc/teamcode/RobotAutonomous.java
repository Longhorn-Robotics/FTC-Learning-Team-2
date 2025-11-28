//import libraries
package org.firstinspires.ftc.teamcode;
import android.graphics.Color;
import android.util.Size;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
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
import java.math.BigDecimal;
import java.math.RoundingMode;

//MIGHT NEED TO REMOVE FOR COMP!!!!!
import android.media.MediaPlayer;
//Declare teleop
@Autonomous(name = "Auto Control", group  = "Robot")




//init and run our teleop
public class RobotAutonomous extends LinearOpMode {

    //MIGHT NEED TO REMOVE FOR COMP!!!
    //MediaPlayer mediaPlayer;




//    aprilTag = new AprilTagProcessor.Builder().build();
//
//    // Adjust Image Decimation to trade-off detection-range for detection-rate.
//    // e.g. Some typical detection data using a Logitech C920 WebCam
//    // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
//    // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
//    // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
//    // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
//    // Note: Decimation can be changed on-the-fly to adapt during a match.
//        aprilTag.setDecimation(2);



    // Adjust Image Decimation to trade-off detection-range for detection-rate.
    // e.g. Some typical detection data using a Logitech C920 WebCam
    // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
    // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
    // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
    // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
    // Note: Decimation can be changed on-the-fly to adapt during a match.
    //aprilTag.setDecimation(2);




    //  Set the GAIN constants to controlxx the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    //speed gain was 0.02
    final double SPEED_GAIN =   0.02 ;   //  Speed Control "Gain". e.g. Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    //was 0.01
    final double TURN_GAIN  =   0.01 ;   //  Turn Control "Gain".  e.g. Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)


    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.25;  //  Clip the turn speed to this max value (adjust for your robot)



    //DEFAULT VALUES

//    final double SPEED_GAIN =   0.02 ;   //  Speed Control "Gain". e.g. Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
//    final double TURN_GAIN  =   0.01 ;   //  Turn Control "Gain".  e.g. Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
//
//    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
//    final double MAX_AUTO_TURN  = 0.25;  //  Clip the turn speed to this max value (adjust for your robot)





//    private DcMotor leftDrive   = null;  //  Used to control the left drive wheel
//    private DcMotor rightDrive  = null;  //  Used to control the right drive wheel


    private static final boolean USE_WEBCAM = false;  // Set true to use a webcam, or false for a phone camera
    private static final int DESIRED_TAG_ID = -1;    // Choose the tag you want to approach or set to -1 for ANY tag.
    // Used to manage the video source.
    //private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;




    //GET THE VALUES FOR OUR BALL TRACKING
    //IN INCHES
    private double ballRadius = 2.5;
    private int cameraWidth = 320;
    private int cameraHeight = 240;


    //Calcualte xf and xy (for the z flip 4) (the operations in the definitons are to covert the units to meters)
    //The FULL dimensions of the camera sensor
    private int fullCameraWidth = 3216;
    private int fullCameraHeight = 2208;
    //in micrometers (the size of the pixel)
    private BigDecimal pixelPitchX = new BigDecimal(1.12 * 0.000001);
    private BigDecimal pixelPitchY = new BigDecimal(1.12 * 0.000001);
    //in mm (the focal length)
    private BigDecimal focalLength = new BigDecimal(3.2 * 0.001);


    //perform the calculation
    //round to SCALE decimal places
    int roundingScale = 1; // For example, 10 decimal places
    RoundingMode roundingMode = RoundingMode.HALF_UP;
    private double fullFocalX = focalLength.divide(pixelPitchX, roundingScale, roundingMode).doubleValue();
    private double fullFocalY = focalLength.divide(pixelPitchY, roundingScale, roundingMode).doubleValue();


    //Scale down to our deesried reolution


//    private double focalLengthX = fullFocalX * ((double)cameraWidth / fullCameraWidth);
//    private double focalLengthY = fullFocalY * ((double)cameraHeight / fullCameraHeight);










    private double cameraCenterX = (cameraWidth - 1) / 2;
    private double cameraCEnterY = (cameraHeight - 1) / 2;


    //OR JUST MANUALLY SET IT


    //logitehc c270
//    private double focalLengthX = 357.1;
////    private double focalLengthY = 476.2;
/// //moto e5 play rear (main) camera
//    private double focalLengthX = 224.1;
//    private double focalLengthY = 224.1;
        //moto e5 play selfie cmaerea (MAYBE)
    private double focalLengthX = 282.2;
    private double focalLengthY = 282.2;



private double averageFocalLengh = (focalLengthX + focalLengthY) / 2;


    //The x and y focal lengths of the camera
    //MOTO G4 Play
//    private double xf = 251.75;
//    private double yf = 251.75;
    //Samsung Galaxy Z flip 4 Ultrawide
//    private double xf = 284.3;
//    private double yf = 310.7;
    //the average of the focal lengths
//    private double ef = (xf + yf) / 2;








    boolean targetFound     = false;    // Set to true when an AprilTag target is detected
    double  drive           = 0;        // Desired forward power/speed (-1 to +1) +ve is forward
    double  turn            = 0;        // Desired turning power/speed (-1 to +1) +ve is CounterClockwise
    //String progress;


    private AprilTagProcessor aprilTag = new AprilTagProcessor.Builder()
            .setLensIntrinsics(focalLengthX, focalLengthY, cameraCenterX, cameraCEnterY)
            // ... these parameters are fx, fy, cx, cy.
            .build();

    // Initialize the ball Detection process
    private ColorBlobLocatorProcessor colorLocatorPurple = new ColorBlobLocatorProcessor.Builder()
            .setTargetColorRange(ColorRange.ARTIFACT_PURPLE)   // Use a predefined color match
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




    private ColorBlobLocatorProcessor colorLocatorGreen = new ColorBlobLocatorProcessor.Builder()
            .setTargetColorRange(ColorRange.ARTIFACT_GREEN)   // Use a predefined color match
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


        private VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(colorLocatorPurple)
                .addProcessor(colorLocatorGreen)
                .addProcessor(aprilTag)
                .setCameraResolution(new Size(cameraWidth, cameraHeight))
                .enableLiveView(true)
                .setCamera(BuiltinCameraDirection.FRONT)
                //.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))


                .build();



    //.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))  .....   for a webcam
//    VisionPortal portal = new VisionPortal.Builder()
//            .addProcessor(colorLocatorPurple)
//            .addProcessor(colorLocatorGreen)
//            .addProcessor(aprilTag)
//            .setCameraResolution(new Size(cameraWidth, cameraHeight))
//            //.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
//
//            .enableLiveView(true)
//            .setCamera(BuiltinCameraDirection.BACK)
//
//
//            .build();






    //Access our robot hardware
    RobotHardware robot = new RobotHardware();


    @Override
    public void runOpMode() {
        //init hardware
//        visionPortal.setProcessorEnabled(colorLocatorPurple, false);
//        visionPortal.setProcessorEnabled(colorLocatorGreen, false);
        //visionPortal.setProcessorEnabled(aprilTag, false);
        robot.init(hardwareMap);
        initAprilTag();



        //MIGHT NEED TO REMOVE FOR COMP!!!
        MediaPlayer foundBall = MediaPlayer.create(hardwareMap.appContext, R.raw.targetacquired);
        foundBall.setLooping(false);

        MediaPlayer gotBall = MediaPlayer.create(hardwareMap.appContext, R.raw.heheboy);
        gotBall.setLooping(false);


        String musicState = "FindingBall";
        //visionPortal.setProcessorEnabled(aprilTag, true);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        robot.runIntake();


        driveToClosestBall(0);

        //driveToAprilTag(36);
//        while (opModeIsActive()) {
//            //String state = driveToClosestBall(5);
//
//
//
//
//
//
//
////            if (state.equals("Driving") && musicState.equals("FindingBall")){
////                foundBall.start();
////                musicState = "FoundBall";
////            }
////            else if (state.equals("No Balls!") && musicState.equals("FoundBall")) {
////                gotBall.start();
////                musicState = "FindingBall";
////            }
//
//
//        }

////        visionPortal.setProcessorEnabled(aprilTag, true);
//        driveToAprilTag(12);
////        visionPortal.setProcessorEnabled(colorLocatorPurple, true);
////        visionPortal.setProcessorEnabled(colorLocatorGreen, true);
//        navigateAndCollectBallRow(1);
////        visionPortal.setProcessorEnabled(colorLocatorPurple, false);
////        visionPortal.setProcessorEnabled(colorLocatorGreen, false);
//        //driveToAprilTag(12);
    }


    private void navigateAndCollectBallRow(int rowNumber) {
        if (rowNumber == 1){
            driveToAprilTag(46);
        }


        collectBallRow();
        robot.moveRobot(-0.3, -0.3);

        if (rowNumber == 1) {
            sleep(1000);
        }
        robot.moveRobot(0, 0);
        return;
    }



    private void collectBallRow () {


        boolean foundBall = false;
        while (! foundBall) {
            robot.moveRobot(0.25, -0.25);
            List<int[]> currentDetections = findArtifacts();
            if (currentDetections.size() > 0) {
                int[] closestBall = currentDetections.get(0);
                double[] ballLocation = getBallPosition(closestBall[0], closestBall[1], closestBall[2]);
                if (ballLocation[1] < 20 && ballLocation[1] > -20) {
                    robot.moveRobot(0,0);
                    foundBall = true;
                }
            }
        }
        String state;
        ElapsedTime startTime = new ElapsedTime();
        while (startTime.seconds() < 3.5) {
            state = driveToClosestBall(0);
        }
        robot.moveRobot(0,0);
//        if (state.equals("No Ball!")) {
//            robot.moveRobot(0,0);
//            return;
//        }
    }


    private void driveToAprilTag (double DESIRED_DISTANCE){
        targetFound = false;
        desiredTag  = null;
        double rangeError;
        double headingError;
        String progress = "Driving";

//        telemetry.addData("Status", "Started");
//        telemetry.update();
        // Step through the list of detected tags and look for a matching tag
        while (progress.equals("Driving")){
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
//            telemetry.addData("Tags", currentDetections);
//            telemetry.update();
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

//            telemetry.addData("Status", "Scanned");
//            telemetry.addData("Tags", currentDetections.size());
//            telemetry.addData("Found", targetFound);
            telemetry.update();
            if (targetFound) {
                // Determine heading and range error so we can use them to control the robot automatically.
                rangeError   = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                headingError = desiredTag.ftcPose.bearing;
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
                return;
            }
            else {
                telemetry.addData("progress", "Driving");
                telemetry.update();
                moveRobot(drive, turn);
                sleep(300);
                moveRobot(0,0);
                progress = "Driving";
                //return "Driving";
            }
        }
    }








    public void moveRobot(double x, double yaw) {
        // Calculate left and right wheel powers.
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










    private void initAprilTag() {
        if (USE_WEBCAM)
            setManualExposure(6, 250);
        // Create the AprilTag processor by using a builder.

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // e.g. Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2);

//        VisionPortal portal = new VisionPortal.Builder()
//                .addProcessor(colorLocatorPurple)
//                .addProcessor(colorLocatorGreen)
//                .addProcessor(aprilTag)
//                .setCameraResolution(new Size(cameraWidth, cameraHeight))
//                //.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
//
//                .enableLiveView(true)
//                .setCamera(BuiltinCameraDirection.BACK)
//
//
//                .build();

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
//        ColorBlobLocatorProcessor.Util.filterByCriteria(
//            ColorBlobLocatorProcessor.BlobCriteria.BY_ASPECT_RATIO,
//                0.7, 1, blobs);





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
        ElapsedTime startTime = new ElapsedTime();
        while (true) {
            // Step through the list of detected tags and look for a matching tag
            List<int[]> currentDetections = findArtifacts();
            if (currentDetections.size() < 1) {
                telemetry.addData("Status", "Nothing detected");
                telemetry.update();
                sleep(1000000000);
                return "No Balls!";
            }

                // Determine heading and range error so we can use them to control the robot automatically.
                int[] closestBall = currentDetections.get(0);
                double[] ballLocation = getBallPosition(closestBall[0], closestBall[1], closestBall[2]);
                double rangeError = (ballLocation[0] - DESIRED_DISTANCE);
                double headingError = ballLocation[1];
                // Use the speed and turn "gains" to calculate how we want the robot to move.  Clip it to the maximum
                drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);


                // Apply desired axes motions to the drivetrain.
                moveRobot(drive, turn);
                sleep(10);
                //return "Driving";
            }
    }




    private double[] getBallPosition(int xPixel, int yPixel, int rPixel) {


        double distance = (ballRadius * averageFocalLengh) / rPixel;


        //Horizontal angle
        double hAngle = Math.toDegrees(Math.atan((xPixel - cameraCenterX) / focalLengthX));
        //double vAngle =  Math.toDegrees(Math.atan((yPixel - cameraCenterY) / focalLengthY));
        telemetry.addData("Distance", distance);
        telemetry.addData("Angle", hAngle);
        telemetry.update();
        double[] returnedData = {distance, hAngle};
        return returnedData;




    }

//
//
//
//
//
//
//
//
//
//
//
//
//
//
//    //This function will try to drive to a spot on the field, and use th eapril tags to determine where it is on the map (might need to use motor encoders)
//    private void driveToSpot (double DESIRED_DISTANCE){
//        targetFound = false;
//        desiredTag  = null;
//        double rangeError;
//        double headingError;
//        String progress = "Driving";
//
////        telemetry.addData("Status", "Started");
////        telemetry.update();
//        // Step through the list of detected tags and look for a matching tag
//        while (progress.equals("Driving")){
//            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
////            telemetry.addData("Tags", currentDetections);
////            telemetry.update();
//            for (AprilTagDetection detection : currentDetections) {
//                // Look to see if we have size info on this tag.
//                if (detection.metadata != null) {
//                    //  Check to see if we want to track towards this tag.
//                    if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
//                        // Yes, we want to use this tag.
//                        targetFound = true;
//                        desiredTag = detection;
//                        break;  // don't look any further.
//                    } else {
//                        // This tag is in the library, but we do not want to track it right now.
//                    }
//                } else {
//                    // This tag is NOT in the library, so we don't have enough information to track to it.
//                }
//            }
//
////            telemetry.addData("Status", "Scanned");
////            telemetry.addData("Tags", currentDetections.size());
////            telemetry.addData("Found", targetFound);
//            telemetry.update();
//            if (targetFound) {
//                // Determine heading and range error so we can use them to control the robot automatically.
//                rangeError   = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
//                headingError = desiredTag.ftcPose.bearing;
//                telemetry.addData("Distance", rangeError);
//                telemetry.addData("heading", headingError);
//                telemetry.update();
//                // Use the speed and turn "gains" to calculate how we want the robot to move.  Clip it to the maximum
//                drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
//                turn  = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
//
//
////                telemetry.addData("Auto","Drive %5.2f, Turn %5.2f", drive, turn);
////                telemetry.update();
//            } else {
//                progress = "No AprilTag!";
//                return;
//            }
////            telemetry.update();
//
//
//            // Apply desired axes motions to the drivetrain.
//            if (rangeError < 3 && rangeError > -3){
//                progress = "Done";
//                return;
//            }
//            else {
//                moveRobot(drive, turn);
//                sleep(10);
//                progress = "Driving";
//                //return "Driving";
//            }
//        }
//    }
}

