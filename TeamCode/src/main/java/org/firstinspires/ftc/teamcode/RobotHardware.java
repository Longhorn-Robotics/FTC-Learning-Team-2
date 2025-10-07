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
    private DcMotor LLaunchMotor;
    private DcMotor RLaunchMotor;
    private DcMotor LIntakeMotor;
    private DcMotor RIntakeMotor;
    private Servo ReleaseFlap;



    //public RobotHardware2(OpMode opmode) {
        //localOpMode = opmode;
    //}
    public RobotHardware(){};

    HardwareMap localHardwareMap;
    //init our hardware
    public void init() {
        LDriveMotor = localHardwareMap.get(DcMotor.class, "LDriveMotor");
        RDriveMotor = localHardwareMap.get(DcMotor.class, "RDriveMotor");
        LLaunchMotor = localHardwareMap.get(DcMotor.class, "LLaunchMotor");
        RLaunchMotor = localHardwareMap.get(DcMotor.class, "RLaunchMotor");
        LIntakeMotor = localHardwareMap.get(DcMotor.class, "LIntakeMotor");
        RIntakeMotor = localHardwareMap.get(DcMotor.class, "RIntakeMotor");
        ReleaseFlap = localHardwareMap.get(Servo.class, "ReleaseFlap");

    }
    //move the robot
    public void moveRobot(double LStickY, double RStickY){
        LDriveMotor.setPower(LStickY);
        RDriveMotor.setPower(RStickY);
    }
    //Run the flywheel intake
    public void runIntake() {
        LIntakeMotor.setPower(1);
        RIntakeMotor.setPower(1);
    }
    //set the launcher to a slower speed while we aren't using it, but we don't stop it so it can get back up to speed quickly
    //close the flap to stop balls from rolling into the launcher
    public void idleLauncher() {
        ReleaseFlap.setPosition(0);
        LLaunchMotor.setPower(0.5);
        RLaunchMotor.setPower(0.5);
    }
    //Start the launcher at the given speed
    //Open the flap to allow balls to enter the launcher
    //add angle later by changing the speed of the two wheels to change to angle the ball is launched
    public void launchItems(double speed) {
        LLaunchMotor.setPower(speed);
        RLaunchMotor.setPower(speed);
        ReleaseFlap.setPosition(90);
    }
}


