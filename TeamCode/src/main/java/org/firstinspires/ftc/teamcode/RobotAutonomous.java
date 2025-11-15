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
            .setCameraResolution(new Size(320, 240))
            .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
            .build();



    //Access our robot hardware
    RobotHardware robot = new RobotHardware();
    //trackArtifacts tracker = new trackArtifacts(this);

    //AprilTagLocalization getPos = new AprilTagLocalization(this);
    //RobotAutoDriveToAprilTagTank driveApril = new RobotAutoDriveToAprilTagTank (this);
    //run our linear op mode
    @Override
    public void runOpMode() {
        //init hardware
        robot.init(hardwareMap);
        initAprilTag();
        //getPos.initAprilTag();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        sleep(10000);
        //tracker.runDetection();

        String state = "goToLaunchZone1";
        //#wait fo the user to press the Play button
        waitForStart();

        while (opModeIsActive()) {
            //read controller data



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
                    telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                }
            } else {
                // This tag is NOT in the library, so we don't have enough information to track to it.
                telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
            }
        }

        // Tell the driver what we see, and what to do.
        if (targetFound) {
            telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
            telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
            telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
        } else {
            telemetry.addData("\n>","Drive using joysticks to find valid target\n");
        }

        // If Left Bumper is being pressed, AND we have found the desired target, Drive to target Automatically .
        if (targetFound) {

            // Determine heading and range error so we can use them to control the robot automatically.
            double  rangeError   = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
            double  headingError = desiredTag.ftcPose.bearing;

            // Use the speed and turn "gains" to calculate how we want the robot to move.  Clip it to the maximum
            drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            turn  = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;

            telemetry.addData("Auto","Drive %5.2f, Turn %5.2f", drive, turn);
        } else {

            // drive using manual POV Joystick mode.
            //IF THE ROBOT DOES NOT SEE AN APRIL TAG
//            drive = -gamepad1.left_stick_y  / 2.0;  // Reduce drive rate to 50%.
//            turn  = -gamepad1.right_stick_x / 4.0;  // Reduce turn rate to 25%.
//            telemetry.addData("Manual","Drive %5.2f, Turn %5.2f", drive, turn);
            //the code is designed such that if the speed is greated than 1 (which is over our max), it knows it doesn't see the tag
            drive = 2;
        }
        telemetry.update();

        // Apply desired axes motions to the drivetrain.
        if (drive < 0.1){
            return "Done";
        }
        else if (drive <= 1) {
            moveRobot(drive, turn);
            sleep(10);
            return "Driving";
        }
        else {
            return "No AprilTag!";
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

    /**
     * Initialize the AprilTag processor.
     */
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

    /*
     Manually set the camera gain and exposure.
     This can only be called AFTER calling initAprilTag(), and only works for Webcams;
    */
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
}


























//
//
//
///*
// * Copyright (c) 2024 Phil Malone
// *
// * Permission is hereby granted, free of charge, to any person obtaining a copy
// * of this software and associated documentation files (the "Software"), to deal
// * in the Software without restriction, including without limitation the rights
// * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// * copies of the Software, and to permit persons to whom the Software is
// * furnished to do so, subject to the following conditions:
// *
// * The above copyright notice and this permission notice shall be included in all
// * copies or substantial portions of the Software.
// * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// * SOFTWARE.
// */
//
//package org.firstinspires.ftc.robotcontroller.external.samples;
//
//import android.graphics.Color;
//import android.util.Size;
//
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.opencv.Circle;
//import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
//import org.firstinspires.ftc.vision.opencv.ColorRange;
//import org.firstinspires.ftc.vision.opencv.ImageRegion;
//
//import java.util.List;
//
///*
// * This OpMode illustrates how to use a video source (camera) to locate specifically colored regions.
// * This sample is targeted towards circular blobs. To see rectangles, look at ConceptVisionColorLocator_Rectangle
// *
// * Unlike a "color sensor" which determines the color of nearby object, this "color locator"
// * will search the Region Of Interest (ROI) in a camera image, and find any "blobs" of color that
// * match the requested color range.  These blobs can be further filtered and sorted to find the one
// * most likely to be the item the user is looking for.
// *
// * To perform this function, a VisionPortal runs a ColorBlobLocatorProcessor process.
// *   The ColorBlobLocatorProcessor (CBLP) process is created first, and then the VisionPortal is built.
// *   The (CBLP) analyses the ROI and locates pixels that match the ColorRange to form a "mask".
// *   The matching pixels are then collected into contiguous "blobs" of pixels.
// *   The outer boundaries of these blobs are called its "contour".  For each blob, the process then
// *   creates the smallest possible circle that will fully encase the contour. The user can then call
// *   getBlobs() to retrieve the list of Blobs, where each contains the contour and the circle.
// *   Note: The default sort order for Blobs is ContourArea, in descending order, so the biggest
// *   contours are listed first.
// *
// * A colored enclosing circle is drawn on the camera preview to show the location of each Blob
// * The original Blob contour can also be added to the preview.
// * This is helpful when configuring the ColorBlobLocatorProcessor parameters.
// *
// * Tip:  Connect an HDMI monitor to the Control Hub to view the Color Location process in real-time.
// *       Or use a screen copy utility like ScrCpy.exe to view the video remotely.
// *
// * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
// * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
// */
//
//@Disabled
//@TeleOp(name = "Concept: Vision Color-Locator (Circle)", group = "Concept")
//public class ConceptVisionColorLocator_Circle extends LinearOpMode {
//    @Override
//    public void runOpMode() {
//        /* Build a "Color Locator" vision processor based on the ColorBlobLocatorProcessor class.
//         * - Specify the color range you are looking for. Use a predefined color, or create your own
//         *
//         *   .setTargetColorRange(ColorRange.BLUE)     // use a predefined color match
//         *     Available colors are: RED, BLUE, YELLOW, GREEN, ARTIFACT_GREEN, ARTIFACT_PURPLE
//         *   .setTargetColorRange(new ColorRange(ColorSpace.YCrCb,  // or define your own color match
//         *                                       new Scalar( 32, 176,  0),
//         *                                       new Scalar(255, 255, 132)))
//         *
//         * - Focus the color locator by defining a RegionOfInterest (ROI) which you want to search.
//         *     This can be the entire frame, or a sub-region defined using:
//         *     1) standard image coordinates or 2) a normalized +/- 1.0 coordinate system.
//         *     Use one form of the ImageRegion class to define the ROI.
//         *       ImageRegion.entireFrame()
//         *       ImageRegion.asImageCoordinates(50, 50,  150, 150)  100x100 pixels at upper left corner
//         *       ImageRegion.asUnityCenterCoordinates(-0.5, 0.5, 0.5, -0.5)  50% width/height in center
//         *
//         * - Define which contours are included.
//         *   You can get ALL the contours, ignore contours that are completely inside another contour.
//         *     .setContourMode(ColorBlobLocatorProcessor.ContourMode.ALL_FLATTENED_HIERARCHY)
//         *     .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
//         *     EXTERNAL_ONLY helps to avoid bright reflection spots from breaking up solid colors.
//         *
//         * - Turn the displays of contours ON or OFF.
//         *     Turning these on helps debugging but takes up valuable CPU time.
//         *        .setDrawContours(true)                Draws an outline of each contour.
//         *        .setEnclosingCircleColor(int color)   Draws a circle around each contour. 0 to disable.
//         *        .setBoxFitColor(int color)            Draws a rectangle around each contour. 0 to disable. ON by default.
//         *
//         *
//         * - include any pre-processing of the image or mask before looking for Blobs.
//         *     There are some extra processing you can include to improve the formation of blobs.
//         *     Using these features requires an understanding of how they may effect the final
//         *     blobs.  The "pixels" argument sets the NxN kernel size.
//         *        .setBlurSize(int pixels)
//         *        Blurring an image helps to provide a smooth color transition between objects,
//         *        and smoother contours.  The higher the number, the more blurred the image becomes.
//         *        Note:  Even "pixels" values will be incremented to satisfy the "odd number" requirement.
//         *        Blurring too much may hide smaller features.  A size of 5 is good for a 320x240 image.
//         *
//         *     .setErodeSize(int pixels)
//         *        Erosion removes floating pixels and thin lines so that only substantive objects remain.
//         *        Erosion can grow holes inside regions, and also shrink objects.
//         *        "pixels" in the range of 2-4 are suitable for low res images.
//         *
//         *     .setDilateSize(int pixels)
//         *        Dilation makes objects and lines more visible by filling in small holes, and making
//         *        filled shapes appear larger. Dilation is useful for joining broken parts of an
//         *        object, such as when removing noise from an image.
//         *        "pixels" in the range of 2-4 are suitable for low res images.
//         *
//         *        .setMorphOperationType(MorphOperationType morphOperationType)
//         *        This defines the order in which the Erode/Dilate actions are performed.
//         *        OPENING:    Will Erode and then Dilate which will make small noise blobs go away
//         *        CLOSING:    Will Dilate and then Erode which will tend to fill in any small holes in blob edges.
//         */
//        ColorBlobLocatorProcessor colorLocator = new ColorBlobLocatorProcessor.Builder()
//                .setTargetColorRange(ColorRange.ARTIFACT_PURPLE)   // Use a predefined color match
//                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
//                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.75, 0.75, 0.75, -0.75))
//                .setDrawContours(true)   // Show contours on the Stream Preview
//                .setBoxFitColor(0)       // Disable the drawing of rectangles
//                .setCircleFitColor(Color.rgb(255, 255, 0)) // Draw a circle
//                .setBlurSize(5)          // Smooth the transitions between different colors in image
//
//                // the following options have been added to fill in perimeter holes.
//                .setDilateSize(15)       // Expand blobs to fill any divots on the edges
//                .setErodeSize(15)        // Shrink blobs back to original size
//                .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)
//
//                .build();
//        /*
//         * Build a vision portal to run the Color Locator process.
//         *
//         *  - Add the colorLocator process created above.
//         *  - Set the desired video resolution.
//         *      Since a high resolution will not improve this process, choose a lower resolution
//         *      that is supported by your camera.  This will improve overall performance and reduce
//         *      latency.
//         *  - Choose your video source.  This may be
//         *      .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))  .....   for a webcam
//         *  or
//         *      .setCamera(BuiltinCameraDirection.BACK)    ... for a Phone Camera
//         */
//        VisionPortal portal = new VisionPortal.Builder()
//                .addProcessor(colorLocator)
//                .setCameraResolution(new Size(320, 240))
//                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
//                .build();
//
//        telemetry.setMsTransmissionInterval(100);   // Speed up telemetry updates for debugging.
//        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
//
//        // WARNING:  To view the stream preview on the Driver Station, this code runs in INIT mode.
//        while (opModeIsActive() || opModeInInit()) {
//            telemetry.addData("preview on/off", "... Camera Stream\n");
//
//            // Read the current list
//            List<ColorBlobLocatorProcessor.Blob> blobs = colorLocator.getBlobs();
//
//            /*
//             * The list of Blobs can be filtered to remove unwanted Blobs.
//             *   Note:  All contours will be still displayed on the Stream Preview, but only those
//             *          that satisfy the filter conditions will remain in the current list of
//             *          "blobs".  Multiple filters may be used.
//             *
//             * To perform a filter
//             *   ColorBlobLocatorProcessor.Util.filterByCriteria(criteria, minValue, maxValue, blobs);
//             *
//             * The following criteria are currently supported.
//             *
//             * ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA
//             *   A Blob's area is the number of pixels contained within the Contour.  Filter out any
//             *   that are too big or small. Start with a large range and then refine the range based
//             *   on the likely size of the desired object in the viewfinder.
//             *
//             * ColorBlobLocatorProcessor.BlobCriteria.BY_DENSITY
//             *   A blob's density is an indication of how "full" the contour is.
//             *   If you put a rubber band around the contour you would get the "Convex Hull" of the
//             *   contour. The density is the ratio of Contour-area to Convex Hull-area.
//             *
//             * ColorBlobLocatorProcessor.BlobCriteria.BY_ASPECT_RATIO
//             *   A blob's Aspect ratio is the ratio of boxFit long side to short side.
//             *   A perfect Square has an aspect ratio of 1.  All others are > 1
//             *
//             * ColorBlobLocatorProcessor.BlobCriteria.BY_ARC_LENGTH
//             *   A blob's arc length is the perimeter of the blob.
//             *   This can be used in conjunction with an area filter to detect oddly shaped blobs.
//             *
//             * ColorBlobLocatorProcessor.BlobCriteria.BY_CIRCULARITY
//             *   A blob's circularity is how circular it is based on the known area and arc length.
//             *   A perfect circle has a circularity of 1.  All others are < 1
//             */
//            ColorBlobLocatorProcessor.Util.filterByCriteria(
//                    ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA,
//                    50, 20000, blobs);  // filter out very small blobs.
//
//            ColorBlobLocatorProcessor.Util.filterByCriteria(
//                    ColorBlobLocatorProcessor.BlobCriteria.BY_CIRCULARITY,
//                    0.6, 1, blobs);     /* filter out non-circular blobs.
//             * NOTE: You may want to adjust the minimum value depending on your use case.
//             * Circularity values will be affected by shadows, and will therefore vary based
//             * on the location of the camera on your robot and venue lighting. It is strongly
//             * encouraged to test your vision on the competition field if your event allows
//             * sensor calibration time.
//             */
//
//            /*
//             * The list of Blobs can be sorted using the same Blob attributes as listed above.
//             * No more than one sort call should be made.  Sorting can use ascending or descending order.
//             * Here is an example.:
//             *   ColorBlobLocatorProcessor.Util.sortByCriteria(
//             *      ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, SortOrder.DESCENDING, blobs);
//             */
//
//            telemetry.addLine("Circularity Radius Center");
//
//            // Display the Blob's circularity, and the size (radius) and center location of its circleFit.
//            for (ColorBlobLocatorProcessor.Blob b : blobs) {
//
//                Circle circleFit = b.getCircle();
//                telemetry.addLine(String.format("%5.3f      %3d     (%3d,%3d)",
//                        b.getCircularity(), (int) circleFit.getRadius(), (int) circleFit.getX(), (int) circleFit.getY()));
//            }
//
//            telemetry.update();
//            sleep(100); // Match the telemetry update interval.
//        }
//    }
//}
