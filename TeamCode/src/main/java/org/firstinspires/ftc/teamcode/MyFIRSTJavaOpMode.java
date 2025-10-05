package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp

public class MyFIRSTJavaOpMode extends LinearOpMode{
    private DcMotor motorTest;

    @Overide
    public void runOpMode() {
        motorTest = hardwareMap.get(DcMotor.class, "motorTest");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //#wait fo the user to press the Play button
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.update();
        }

    }

}
