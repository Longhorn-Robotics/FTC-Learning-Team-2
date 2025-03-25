package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class RobotHardwareTest {
    HardwareMap hwMap;
    public DcMotor arm;
    public DcMotor left;
    public DcMotor right;
    public Servo intake;
    public Servo gyro;

    private ElapsedTime period = new ElapsedTime();

    public RobotHardwareTest() {}

    public void init(HardwareMap ahwMap) {
        // Save reference to hardware map
        hwMap = ahwMap;

        // Initialize drive motors
        arm = hwMap.get(DcMotor.class, "armMotor");
        left = hwMap.get(DcMotor.class, "lMotor");
        right = hwMap.get(DcMotor.class, "rMotor");
        intake = hwMap.get(Servo.class, "iServo");
        gyro = hwMap.get(Servo.class, "gServo");

        arm.setDirection(DcMotor.Direction.FORWARD);
        arm.setPower(0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        left.setDirection(DcMotor.Direction.FORWARD);
        left.setPower(0);
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        right.setDirection(DcMotor.Direction.FORWARD);
        right.setPower(0);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}
