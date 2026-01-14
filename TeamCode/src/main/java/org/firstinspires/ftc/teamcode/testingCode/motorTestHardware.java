//https://ftc-docs.firstinspires.org/en/latest/programming_resources/tutorial_specific/android_studio/creating_op_modes/Creating-and-Running-an-Op-Mode-%28Android-Studio%29.html#teamcode-module


package org.firstinspires.ftc.teamcode.testingCode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


//@TeleOp

public class motorTestHardware {
    //Create our robot parts
    //private LinearOpMode localOpMode = null;
    private DcMotor motora;
    private DcMotor motorb;
    private DcMotor motorc;




    //public RobotHardware2(OpMode opmode) {
    //localOpMode = opmode;
    //}
    public motorTestHardware(){};

    HardwareMap localHardwareMap;
    //init our hardware
    public void init(HardwareMap localHardwareMap) {
        motora = localHardwareMap.get(DcMotor.class, "motora");
        motorb = localHardwareMap.get(DcMotor.class, "motorb");
        motorc = localHardwareMap.get(DcMotor.class, "motorc");


    }
    //move the robot
    public void moveRobot(double LStickY, double RStickY, double a){
        motora.setPower(LStickY);
        motorb.setPower(RStickY);
        motorc.setPower(a);
    }
    //Run the flywheel intake

}


