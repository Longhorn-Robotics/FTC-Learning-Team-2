 package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleopLite", group = "Pushbot")

public class TeleopTest extends OpMode {
    private static float bumper_sens;
    private static double intake_pos;
    private static double gyro_pos = 0.5f;
    private static double armLowerLimit = -4000;
    private static double armUpperLimit = 39500;
    public static int targetArmPos;

    RobotHardwareTest robot = new RobotHardwareTest();
    // Code to run ONCE when the driver hits INIT
    @Override
    public void init() {
        telemetry.addLine("Bismillah");
        bumper_sens = 0.005f;
        gyro_pos = 0;
        intake_pos = 0;
        robot.init(hardwareMap);

        gyro_pos = 0.3333333333;

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
        double arm_power = (gamepad1.left_stick_y);
        double left_power = (-gamepad1.right_stick_y - gamepad1.right_stick_x) * 0.5f;
        double right_power = -(-gamepad1.right_stick_y + gamepad1.right_stick_x) * 0.5f;
        gyro_pos -= (-gamepad1.right_trigger + gamepad1.left_trigger) * 0.0025;
        gyro_pos = Math.min(1, gyro_pos);
        gyro_pos = Math.max(0, gyro_pos);
        telemetry.addLine(String.valueOf(gyro_pos)); //not 300 is 270 haha
        //telemetry.addLine(String.valueOf(arm_power));


        if(gamepad1.left_bumper) {
            intake_pos = 1;
        }else if(gamepad1.right_bumper) {
            intake_pos = 0;
            telemetry.addLine("WORKED");
        }
        else {
            intake_pos = 0.5;
        }

        targetArmPos -= 7.5 * arm_power;
        targetArmPos = (int)Math.min(armUpperLimit, targetArmPos);
        targetArmPos = (int)Math.max(armLowerLimit, targetArmPos);
        //telemetry.addLine(String.valueOf(targetArmPos));

        //Arm and wheels
        robot.arm.setTargetPosition(targetArmPos);
        robot.arm.setPower(10);
        //robot.arm.setPower(arm_power);
        //robot.arm.
        //telemetry.addLine(String.valueOf(robot.arm.getCurrentPosition()));

        robot.left.setPower(-left_power);
        robot.right.setPower(-right_power);

        //Gyro Servo
        robot.intake.setPosition(intake_pos);
        robot.gyro.setPosition(gyro_pos);

        telemetry.update();
    }

    // Code to run ONCE after the driver hits STOP
    @Override
    public void stop() {

    }
}