//import libraries
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


//Declare teleop
@Autonomous(name = "Auto Control", group  = "Robot")


//init and run our teleop
public class RobotAutonomous extends LinearOpMode{

    //Access our robot hardware
    RobotHardware robot = new RobotHardware();
    trackArtifacts tracker = new trackArtifacts(this);
    //run our linear op mode
    @Override
    public void runOpMode() {
        //init hardware
        robot.init();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        sleep(10000);
        tracker.runDetection();


        //#wait fo the user to press the Play button
        waitForStart();

        while (opModeIsActive()) {
            //read controller data


            //Insert the code to figure out the best path for robot to take to score
            //Then, convert that into a format that we can input below

            //We have two methods that work, create a path, and follow it OR
            // create a path, move on it slightly, regenerate path to make sure we are on right track, and that nothing changed, repeat



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
