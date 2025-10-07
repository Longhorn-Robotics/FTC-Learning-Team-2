//import libraries
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


//Declare teleop
@TeleOp(name = "Driver Control", group  = "Robot")


//init and run our teleop
public class RobotTeleOp extends LinearOpMode{

    //Access our robot hardware
    RobotHardware robot = new RobotHardware(this);
    trackArtifacts tracker = new trackArtifacts(this);
    //run our linear op mode
    public void runOpMode() {
        //init hardware
        robot.init();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        sleep(10000);
        tracker.runDetection();


        //#wait fo the user to press the Play button
        double LStickY = 0;
        double RStickY = 0;
        boolean RBumper = false;
        waitForStart();

        while (opModeIsActive()) {
            //read controller data
            LStickY = -this.gamepad1.left_stick_y;
            RStickY = -this.gamepad1.right_stick_y;
            RBumper = this.gamepad1.right_bumper;



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
            tracker.runDetection();
            telemetry.update();
            sleep(50);
        }


    }

}
