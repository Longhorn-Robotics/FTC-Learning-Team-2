//import libraries
package org.firstinspires.ftc.teamcode;


import android.media.MediaPlayer;
import android.util.Size;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;




//Declare teleop
@TeleOp(name = "PikaBot", group  = "Robot")






public class TeleOpPika extends LinearOpMode {
    private int stall;
    //Get functions to move the robot
    RobotHardware robot = new RobotHardware();


    private MediaPlayer girl;
    private MediaPlayer happy;
    private MediaPlayer thunderbolt;
    private MediaPlayer scream;


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
    public void runOpMode() {


        girl = MediaPlayer.create(hardwareMap.appContext, R.raw.pg);
        girl.setLooping(false);
        happy = MediaPlayer.create(hardwareMap.appContext, R.raw.ph);
        happy.setLooping(false);
        thunderbolt = MediaPlayer.create(hardwareMap.appContext, R.raw.pt);
        thunderbolt.setLooping(false);
        scream = MediaPlayer.create(hardwareMap.appContext, R.raw.ps);
        scream.setLooping(false);


        robot.init(hardwareMap);
        //maxverstappan.start();
        telemetry.addData("Status", "Initialized");
        telemetry.update();




        //Define variables for reading the game controller
        double LStickY;
        double RStickY;
        boolean s = false; //thunderbolt
        boolean t = false; // scream
        boolean o = false; //girl
        boolean x = false; //happy


        waitForStart();


        while (opModeIsActive()) {
            //read controller data
            LStickY = -this.gamepad1.left_stick_y;
            RStickY = this.gamepad1.right_stick_y;
            x = this.gamepad1.x;
            s = this.gamepad1.y;
            t = this.gamepad1.a;
            o = this.gamepad1.b;




            //run commands on on the robot
            if (s) {
                thunderbolt.start();
                robot.moveRobot(0.5, 0.5);
                sleep(800);
                robot.moveRobot(-1, 1);
                sleep(2400);
                robot.moveRobot(0, 0);
            } else if (t) {
                scream.start();
                robot.moveRobot(1,-1);
                sleep(1800);
                robot.moveRobot(0, 0);
//            robot.runIntake(0);
//            ElapsedTime intakeWaitTime = new ElapsedTime();
//            while (intakeWaitTime.seconds() < 2.5) {
//                stall = 0;
//            }
//            robot.runIntake(1);
//            intakeWaitTime.reset();
//            while (intakeWaitTime.seconds() < 2.5) {
//                stall = 0;
//            }
            }
            else if (x) {
                happy.start();
                robot.moveRobot(1,-1);
                sleep(400);
                robot.moveRobot(1,-1);
                sleep(400);
                robot.moveRobot(1,-1);
                sleep(400);
                robot.moveRobot(1,-1);
                sleep(400);
                robot.moveRobot(0,0);
            }
            else if (o) {
                girl.start();
                robot.moveRobot(1,1);
                sleep(1200);
                robot.moveRobot(0,0);


            }
            robot.moveRobot(LStickY, RStickY);
            robot.runIntake(1);




            //record telemetry data
            telemetry.addData("Left Stick Y", LStickY);
            telemetry.addData("Right Stick Y", RStickY);
            telemetry.addData("Status", "Running");
            System.out.println("Running");
            //.runDetection();
            telemetry.update();
        }
    }




}

