//https://ftc-docs.firstinspires.org/en/latest/programming_resources/tutorial_specific/android_studio/creating_op_modes/Creating-and-Running-an-Op-Mode-%28Android-Studio%29.html#teamcode-module


package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp

public class MainController_HardwareInit extends LinearOpMode{
    private DcMotor LDriveMotor;
    private DcMotor RDriveMotor;
    private DcMotor LLaunchMotor;
    private DcMotor RLaunchMotor;
    private DcMotor LIntakeMotor;
    private DcMotor RIntakeMotor;


    @Overide
    public void runOpMode() {
        LDriveMotor = hardwareMap.get(DcMotor.class, "LDriveMotor");
        RDriveMotor = hardwareMap.get(DcMotor.class, "RDriveMotor");
        LLaunchMotor = hardwareMap.get(DcMotor.class, "LLaunchMotor");
        RLaunchMotor = hardwareMap.get(DcMotor.class, "RLaunchMotor");
        LIntakeMotor = hardwareMap.get(DcMotor.class, "LIntakeMotor");
        RIntakeMotor = hardwareMap.get(DcMotor.class, "RIntakeMotor");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //#wait fo the user to press the Play button
        waitForStart();
        double tgtPower = 0;
        while (opModeIsActive()) {
            tgtPower = -this.gamepad1.left_stick_y;
            motorTest.setPower(tgtPower);
            telemetry.addData("Target Power", tgtPower);
            telemetry.addData("LDriveMotor Power", LDriveMotor.getPower());
            telemetry.addData("RDriveMotor Power", RDriveMotor.getPower());
            telemetry.addData("Status", "Running");
            telemetry.update();
        }

    }

}
