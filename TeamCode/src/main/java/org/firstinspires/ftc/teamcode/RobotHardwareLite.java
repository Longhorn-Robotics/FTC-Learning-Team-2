package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class RobotHardwareLite {
    HardwareMap hwMap;
    public DcMotor lbDrive;
    public DcMotor lfDrive;
    public DcMotor rbDrive;
    public DcMotor rfDrive;
    public DcMotor rail;

    private ElapsedTime period = new ElapsedTime();

    public RobotHardwareLite() {}

    public void init(HardwareMap ahwMap) {
        // Save reference to hardware map
        hwMap = ahwMap;

        // Initialize drive motors
        lfDrive = hwMap.get(DcMotor.class, "motorFL");
        lbDrive = hwMap.get(DcMotor.class, "motorBL");
        rfDrive = hwMap.get(DcMotor.class, "motorFR");
        rbDrive = hwMap.get(DcMotor.class, "motorBR");
        lfDrive.setDirection(DcMotor.Direction.FORWARD);
        lbDrive.setDirection(DcMotor.Direction.FORWARD);
        rfDrive.setDirection(DcMotor.Direction.FORWARD);
        rbDrive.setDirection(DcMotor.Direction.FORWARD);
        lfDrive.setPower(0);
        lbDrive.setPower(0);
        rfDrive.setPower(0);
        rbDrive.setPower(0);
        lfDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lbDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rfDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rbDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lfDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lbDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rfDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rbDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize linear rail motor to run with encoder
        rail = hwMap.get(DcMotor.class, "railRAIL");
        rail.setDirection(DcMotor.Direction.REVERSE);
        rail.setTargetPosition(0);
        rail.setPower(0.5);
        rail.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rail.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rail.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rail.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}
