
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotHardware2 {

    // Drive Motors
    public DcMotor LDriveMotor;
    public DcMotor RDriveMotor;

    // Other Motors/Servos
    public DcMotor LaunchMotor;
    public DcMotor IntakeMotor;
    private Servo ReleaseFlap;

    // Constants for encoder calculations
    static final double COUNTS_PER_MOTOR_REV = 537.6; // goBILDA 5203 Series (19.2:1 Ratio)
    static final double WHEEL_DIAMETER_INCHES = 3.78;   // For 96mm Rhino Wheels
    static final double ROBOT_TRACK_WIDTH_INCHES = 6.89; // For 17.5cm track width
    public static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV) / (WHEEL_DIAMETER_INCHES * 3.1415);
    public static final double COUNTS_PER_DEGREE_TURN = (COUNTS_PER_MOTOR_REV * ROBOT_TRACK_WIDTH_INCHES * 3.1415) / (360 * WHEEL_DIAMETER_INCHES * 3.1415);

    public RobotHardware2() {};

    public void init(HardwareMap localHardwareMap) {
        LDriveMotor = localHardwareMap.get(DcMotor.class, "ld");
        RDriveMotor = localHardwareMap.get(DcMotor.class, "rd");
        LaunchMotor = localHardwareMap.get(DcMotor.class, "ou");
        IntakeMotor = localHardwareMap.get(DcMotor.class, "in");

        LDriveMotor.setDirection(DcMotor.Direction.REVERSE);
        RDriveMotor.setDirection(DcMotor.Direction.FORWARD);
        IntakeMotor.setDirection(DcMotor.Direction.REVERSE);
        LaunchMotor.setDirection(DcMotor.Direction.FORWARD);

        LDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        LDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void moveRobot(double LPower, double RPower) {
        LDriveMotor.setPower(LPower);
        RDriveMotor.setPower(RPower);
    }

    public void runIntake(double speed) {
        IntakeMotor.setPower(speed);
    }

    public void launchItems(double speed) {
        LaunchMotor.setPower(speed * -1);
    }
}
