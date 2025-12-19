//https://ftc-docs.firstinspires.org/en/latest/programming_resources/tutorial_specific/android_studio/creating_op_modes/Creating-and-Running-an-Op-Mode-%28Android-Studio%29.html#teamcode-module


package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


//@TeleOp

public class RobotHardware {
    //Create our robot parts
    private LinearOpMode localOpMode = null;
    private DcMotor LDriveMotor;
    private DcMotor RDriveMotor;
    private DcMotor LaunchMotor;
    private DcMotor IntakeMotor;
    private Servo ReleaseFlap;



    //public RobotHardware2(OpMode opmode) {
        //localOpMode = opmode;
    //}
    public RobotHardware(){};

    HardwareMap localHardwareMap;
    //init our hardware
    public void init(HardwareMap localHardwareMap) {
        LDriveMotor = localHardwareMap.get(DcMotor.class, "ld");
        RDriveMotor = localHardwareMap.get(DcMotor.class, "rd");
        LaunchMotor = localHardwareMap.get(DcMotor.class, "ou");
        IntakeMotor = localHardwareMap.get(DcMotor.class, "in");
        //ReleaseFlap = localHardwareMap.get(Servo.class, "ReleaseFlap");

    }

    public void AutoInit (HardwareMap localHardwareMap) {
        LDriveMotor = localHardwareMap.get(DcMotor.class, "ld");
        RDriveMotor = localHardwareMap.get(DcMotor.class, "rd");
        IntakeMotor = localHardwareMap.get(DcMotor.class, "in");
        LaunchMotor = localHardwareMap.get(DcMotor.class, "ou");
        LDriveMotor.setDirection(DcMotor.Direction.REVERSE);
        RDriveMotor.setDirection(DcMotor.Direction.FORWARD);
        IntakeMotor.setDirection(DcMotor.Direction.REVERSE);
        LaunchMotor.setDirection(DcMotor.Direction.FORWARD);
    }
    //move the robot
    public void moveRobot(double LPower, double RPower){
        LDriveMotor.setPower(LPower);
        RDriveMotor.setPower(RPower);
    }
    //Run the intake
    public void runIntake(double speed) {
        IntakeMotor.setPower(speed);
    }
    //set the launcher to a slower speed while we aren't using it, but we don't stop it so it can get back up to speed quickly
    //close the flap to stop balls from rolling into the launcher
    public void idleLauncher(double speed) {
        //eleaseFlap.setPosition(0);
        LaunchMotor.setPower(speed * -1);
    }
    //Start the launcher at the given speed
    //Open the flap to allow balls to enter the launcher
    //add angle later by changing the speed of the two wheels to change to angle the ball is launched
    public void launchItems(double speed) {
        LaunchMotor.setPower(speed * -1);
        //ReleaseFlap.setPosition(90);
    }
}


