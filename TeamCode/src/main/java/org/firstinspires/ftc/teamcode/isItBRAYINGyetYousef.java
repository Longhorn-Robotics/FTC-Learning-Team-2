package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.concurrent.TimeUnit;
//test
@TeleOp(name = "TeleopLite", group = "Pushbot")

public class isItBRAYINGyetYousef extends OpMode {
    public static int bumper_sens;
    public static double gyro_pos;
    public static double intake_pos;

    public static double left_power;
    public static double right_power;
    public static double arm_power;



    RobotHardwareTest robot = new RobotHardwareTest();
    // Code to run ONCE when the driver hits INIT
    @Override
    public void init() {
        System.out.println("Bismillah");
        bumper_sens = 10;
        gyro_pos = 0;
        intake_pos = 0;
        robot.init(hardwareMap);
    }

    // Code to run REPEATED LY after the driver hits INIT, but before they hit PLAY
    @Override
    public void init_loop() {

    }

    // Code to run ONCE when the driver hits PLAY
    @Override
    public void start() {

    }

    // Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    @Override
    public void loop() {
        robot.arm.setPower(0.2f);
        try {
            robot.arm.setPower(0.2f);
            TimeUnit.SECONDS.sleep(1);
            robot.arm.setPower(0);
            TimeUnit.SECONDS.sleep(5);
            robot.arm.setPower(-0.1);
            TimeUnit.SECONDS.sleep(1);
            robot.arm.setPower(0);
            TimeUnit.SECONDS.sleep(1);
            robot.arm.setPower(0.1);
            TimeUnit.SECONDS.sleep(3);
            robot.arm.setPower(-0.2);

        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }


        intake_pos += (gamepad1.right_trigger - gamepad1.left_trigger) * 2.0f;

        if(gamepad1.right_bumper) {
            System.out.println("Gyrating right Insha'allah");
            gyro_pos += bumper_sens;
        } else if(gamepad1.left_bumper) {
            System.out.println("Gyrating left Insha'allah");
            gyro_pos -= bumper_sens;
        }

        //Arm and wheels
        robot.arm.setPower(arm_power);
        robot.left.setPower(left_power);
        robot.right.setPower(-right_power);

        //Gyro Servo
        robot.intake.setPosition(intake_pos);
        robot.gyro.setPosition(intake_pos);
    }

    // Code to run ONCE after the driver hits STOP
    @Override
    public void stop() {
        System.out.println("Alhamdulillah");
    }
}