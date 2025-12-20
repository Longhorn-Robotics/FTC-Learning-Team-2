//https://ftc-docs.firstinspires.org/en/latest/programming_resources/tutorial_specific/android_studio/creating_op_modes/Creating-and-Running-an-Op-Mode-%28Android-Studio%29.html#teamcode-module


package org.firstinspires.ftc.teamcode.testingCode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;


//@TeleOp

public class servoTestHardware {
    //Create our robot parts
    //private LinearOpMode localOpMode = null;
    private Servo servoa;




    //public RobotHardware2(OpMode opmode) {
    //localOpMode = opmode;
    //}
    public servoTestHardware(){};

    HardwareMap localHardwareMap;
    //init our hardware
    public void init(HardwareMap localHardwareMap) {
        servoa = localHardwareMap.get(Servo.class, "se");



    }
    //move the robot
    public void setAngle(double angle){
        servoa.setPosition(angle);
    }
    //Run the flywheel intake

}


