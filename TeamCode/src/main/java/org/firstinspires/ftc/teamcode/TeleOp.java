package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Concept: Robot Hardware Class", group="Robot")

public class TeleOp extends LinearOpMode{
    RobotHardware   robot       = new RobotHardware(this);


    public void runOpMode() {
//        LDriveMotor = hardwareMap.get(DcMotor.class, "LDriveMotor");
//        RDriveMotor = hardwareMap.get(DcMotor.class, "RDriveMotor");
//        LLaunchMotor = hardwareMap.get(DcMotor.class, "LLaunchMotor");
//        RLaunchMotor = hardwareMap.get(DcMotor.class, "RLaunchMotor");
//        LIntakeMotor = hardwareMap.get(DcMotor.class, "LIntakeMotor");
//        RIntakeMotor = hardwareMap.get(DcMotor.class, "RIntakeMotor");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //#wait fo the user to press the Play button
        waitForStart();
        double LStickY = 0;
        double RStickY = 0;
        while (opModeIsActive()) {
            LStickY = -this.gamepad1.left_stick_y;
            RStickY = -this.gamepad1.right_stick_y;
            robot.moveRobot(LStickY, RStickY);
            telemetry.addData("Left Stick Y", LStickY);
            telemetry.addData("Right Stick Y", RStickY);
//            telemetry.addData("LDriveMotor Power", robot.LDriveMotor.getPower());
//            telemetry.addData("RDriveMotor Power", robot.RDriveMotor.getPower());
            telemetry.addData("Status", "Running");
            telemetry.update();
        }
        sleep(50);

    }

}
