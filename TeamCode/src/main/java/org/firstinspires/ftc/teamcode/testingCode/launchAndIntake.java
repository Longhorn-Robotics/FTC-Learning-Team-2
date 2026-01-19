//import libraries
package org.firstinspires.ftc.teamcode.testingCode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


//Declare teleop
@TeleOp(name = "Launch N Intake", group  = "Robot")



public class launchAndIntake extends LinearOpMode{
    //get functions and data from other files
    motorTestHardware robot = new motorTestHardware();



    //init our hardware
    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        robot.moveRobot(1,1,0);
        sleep(8000);
        robot.moveRobot(1,0,0);
        sleep(4000);
        robot.moveRobot(1,0,-1);
        sleep(5000);
        robot.moveRobot(1,1,-1);
        sleep(4000);
        robot.moveRobot(1,0,0);
    }


    //Access our robot hardware


}
