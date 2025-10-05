//https://ftc-docs.firstinspires.org/en/latest/programming_resources/tutorial_specific/android_studio/creating_op_modes/Creating-and-Running-an-Op-Mode-%28Android-Studio%29.html#teamcode-module


package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;



//@TeleOp

public class RobotHardware{
    private LinearOpMode initOpMode = null;
    private DcMotor LDriveMotor;
    private DcMotor RDriveMotor;
    private DcMotor LLaunchMotor;
    private DcMotor RLaunchMotor;
    private DcMotor LIntakeMotor;
    private DcMotor RIntakeMotor;
    private Servo ReleaseFlap;


    public double LStickY = 0;
    public double RStickY = 0;


    public RobotHardware(LinearOpMode opmode) {
        initOpMode = opmode;
    }


    //@Override
    public void init() {
        LDriveMotor = initOpMode.hardwareMap.get(DcMotor.class, "LDriveMotor");
        RDriveMotor = initOpMode.hardwareMap.get(DcMotor.class, "RDriveMotor");
        LLaunchMotor = initOpMode.hardwareMap.get(DcMotor.class, "LLaunchMotor");
        RLaunchMotor = initOpMode.hardwareMap.get(DcMotor.class, "RLaunchMotor");
        LIntakeMotor = initOpMode.hardwareMap.get(DcMotor.class, "LIntakeMotor");
        RIntakeMotor = initOpMode.hardwareMap.get(DcMotor.class, "RIntakeMotor");
        ReleaseFlap = initOpMode.hardwareMap.get(Servo.class, "ReleaseFlap");

    }

    public void moveRobot(double LStickY, double RStickY){
        LDriveMotor.setPower(LStickY);
        RDriveMotor.setPower(RStickY);
    }
    public void startIntake() {
        LIntakeMotor.setPower(1);
        RIntakeMotor.setPower(1);
    }
    public void idleLauncher() {
        ReleaseFlap.setPosition(0);
        LLaunchMotor.setPower(0.5);
        RLaunchMotor.setPower(0.5);
    }
    //add angle later by changing the speed of the two wheels to change to angle the ball is launched
    public void launchItems(speed) {
        LLaunchMotor.setPower(speed);
        RLaunchMotor.setPower(speed);
        ReleaseFlap.setPosition(90);
    }
}


