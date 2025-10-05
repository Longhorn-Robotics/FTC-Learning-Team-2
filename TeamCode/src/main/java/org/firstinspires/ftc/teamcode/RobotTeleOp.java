package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


//@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Driver Control", group="Robot")
@TeleOp(name = "Driver Control", group  = "Robot")
public class RobotTeleOp extends LinearOpMode{
    RobotHardware   robot       = new RobotHardware(this);


    public void runOpMode() {
        robot.init();

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
            telemetry.addData("Status", "Running");
            telemetry.update();
            sleep(50);
        }


    }

}
