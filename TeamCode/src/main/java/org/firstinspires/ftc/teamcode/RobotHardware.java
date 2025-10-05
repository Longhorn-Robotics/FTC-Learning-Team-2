//https://ftc-docs.firstinspires.org/en/latest/programming_resources/tutorial_specific/android_studio/creating_op_modes/Creating-and-Running-an-Op-Mode-%28Android-Studio%29.html#teamcode-module


package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp

public class RobotHardware{
    private LinearOpMode initOpMode = null;
    private DcMotor LDriveMotor;
    private DcMotor RDriveMotor;
    private DcMotor LLaunchMotor;
    private DcMotor RLaunchMotor;
    private DcMotor LIntakeMotor;
    private DcMotor RIntakeMotor;


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

        initOpMode.telemetry.addData("Status", "Initialized");
        initOpMode.telemetry.update();
    }

    public void moveRobot(double LStickY, double RStickY){
        LDriveMotor.setPower(LStickY);
        RDriveMotor.setPower(RStickY);
    }
}


