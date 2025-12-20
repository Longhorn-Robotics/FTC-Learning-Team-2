//import libraries
package org.firstinspires.ftc.teamcode.testingCode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


//Declare teleop
@TeleOp(name = "Servo Test", group  = "Robot")



public class servoTest extends OpMode{
    //get functions and data from other files
    servoTestHardware robot = new servoTestHardware();
    double LStickY;


    //init our hardware
    @Override
    public void init() {
        robot.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }


    //Access our robot hardware


    @Override
    //run our linear op mode
    public void loop() {
        LStickY = -this.gamepad1.left_stick_y;
        //read controller data




        //run commands on on the robot
//        robot.moveRobot( 100000000, -1000000);
        robot.setAngle(LStickY);


        //record telemetry data
        telemetry.addData("Status", "Running");
        System.out.println("Running");
        //.runDetection();
        telemetry.update();
    }

    @Override
    public void stop () {

    }

}
