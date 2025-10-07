////import libraries
//package org.firstinspires.ftc.teamcode;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//
////Declare teleop
//@TeleOp(name = "Driver Control", group  = "Robot")
//
//
//
//public class RobotTeleOp2 extends OpMode{
//    //get functions and data from other files
//    RobotHardware robot = new RobotHardware();
//    trackArtifacts tracker = new trackArtifacts();
//
//
//    //init our hardware
//    @Override
//    public void init() {
//        robot.init();
//        telemetry.addData("Status", "Initialized");
//        telemetry.update();
//    }
//
//
//    //Access our robot hardware
//    double LStickY = 0;
//    double RStickY = 0;
//    boolean RBumper = false;
//
//
//    @Override
//    //run our linear op mode
//    public void loop() {
//        //read controller data
//        LStickY = -this.gamepad1.left_stick_y;
//        RStickY = -this.gamepad1.right_stick_y;
//        RBumper = this.gamepad1.right_bumper;
//
//
//
//        //run commands on on the robot
//        if (RBumper) {
//            robot.idleLauncher();
//        }
//        else {
//            robot.launchItems(1);
//        }
//        robot.moveRobot(LStickY, RStickY);
//        robot.runIntake();
//
//
//        //record telemetry data
//        telemetry.addData("Left Stick Y", LStickY);
//        telemetry.addData("Right Stick Y", RStickY);
//        telemetry.addData("Launching", RBumper);
//        telemetry.addData("Status", "Running");
//        System.out.println("Running");
//        tracker.runDetection();
//        telemetry.update();
//    }
//
//    @Override
//    public void stop () {
//
//    }
//
//}
