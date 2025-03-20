 package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleopLite", group = "Pushbot")

public class TeleopTest extends OpMode {
    private static float bumper_sens;
    private static double intake_pos;
    private static double gyro_pos;
    private static double armLowerLimit = 0;
    private static double armUpperLimit = 1;
    public static int targetArmPos;

    RobotHardwareTest robot = new RobotHardwareTest();
    // Code to run ONCE when the driver hits INIT
    @Override
    public void init() {
        System.out.println("Bismillah");
        bumper_sens = 0.005f;
        gyro_pos = 0;
        intake_pos = 0;
        robot.init(hardwareMap);
    }

    // Code to run REPEATED LY after the driver hits INIT, but before they hit PLAY
    @Override
    public void init_loop() {

    }

    // Code to run ONCE when the driver h its PLAY
    @Override
    public void start() {

    }

    // Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    @Override
    public void loop() {
        double arm_power = (gamepad1.left_stick_y) * 0.01f;
        double left_power = (-gamepad1.right_stick_y + gamepad1.right_stick_x) * 0.1f;
        double right_power = -(-gamepad1.right_stick_y - gamepad1.right_stick_x) * 0.1f;
        intake_pos += (gamepad1.right_trigger - gamepad1.left_trigger) * 0.005f;
        //System.out.println(intake_pos);

        Log.d("myTag", "This is my message");

        if(gamepad1.right_bumper) {
            gyro_pos = 0.5;
            System.out.println("right bumper");
        } else if(gamepad1.left_bumper) {
            gyro_pos = 0;
            Log.d("myTag", "This is my message");
        }

        targetArmPos -= arm_power;
        targetArmPos = (int)Math.min(armUpperLimit, targetArmPos);
        targetArmPos = (int)Math.max(armLowerLimit, targetArmPos);
        System.out.println(robot.arm.getCurrentPosition());

        //Arm and wheels
        robot.arm.setTargetPosition(targetArmPos);
        robot.left.setPower(left_power);
        robot.right.setPower(right_power);

        //Gyro Servo
        robot.intake.setPosition(intake_pos);
        robot.gyro.setPosition(gyro_pos);
    }

    // Code to run ONCE after the driver hits STOP
    @Override
    public void stop() {
    }
}