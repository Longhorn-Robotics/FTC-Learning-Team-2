package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.function.BooleanSupplier;

/*
 _____ _____ _____ ___  ___  ___  
/  ___|_   _|  __ \|  \/  | / _ \ 
\ `--.  | | | |  \/| .  . |/ /_\ \
 `--. \ | | | | __ | |\/| ||  _  |
/\__/ /_| |_| |_\ \| |  | || | | |
\____/ \___/ \____/\_|  |_/\_| |_/
  u      n      e      a      l
  p      t      n      n      g
  e      e      e      a      o
  r      g      r      g      r
         r      a      e      i
         a      l      m      t
         t             e      h
         e             n      m
         d             t
*/
@TeleOp(name = "TeleopSIGMA", group = "Pushbot")
public class TeleopSIGMA extends OpMode {

    static final double RAIL_MIN = 10.0f;
    static final double RAIL_MAX = 3000.0f;
    static final double CLAW_OPEN = 0.0f;
    static final double CLAW_CLOSED = 0.75f;
    static final double ARM_DOWN = 5.0f;
    static final double ARM_UP = 673.0f;

    final double hangingMotorSpeed = 2.0f;

    /* Declare OpMode members. */
    RobotHardwareSIGMA robot = new RobotHardwareSIGMA();
    double railPosition = 0.0f;
    double armPosition = 0.0f;
    boolean slowmode = false;
    boolean clawOpen = true;
    double clawPosition = CLAW_OPEN;

    // Debounce Stuff - by Teo
    // It would be a good idea to make this a separate class or something
    // especially given the entire point of this is because it's supposed
    // to be better programming practices. But that's boring.
    private final static int maxButtons = 20;
    private int buttons = 0;
    private final BooleanSupplier[] buttonConditions = new BooleanSupplier[maxButtons];
    private final Boolean[] wasPressed = new Boolean[maxButtons];
    private final Runnable[] buttonActions = new Runnable[maxButtons];

    private void addButton(BooleanSupplier buttonCondition, Runnable action) {
        if (buttons >= maxButtons) return;
        buttonConditions[buttons] = buttonCondition;
        buttonActions[buttons] = action;
        wasPressed[buttons] = false;
        buttons++;
    }

    private void doButtonPresses() {
        for (int i = 0; i < buttons; i++) {
            boolean was = wasPressed[i];
            boolean is = buttonConditions[i].getAsBoolean();
            if (!was && is) {
                buttonActions[i].run();
            }
            wasPressed[i] = is;
        }
    }

    // Code to run ONCE when the driver hits INIT
    @Override
    public void init() {
        robot.init(hardwareMap);
        // Send telemetry message to signify robot waiting
        telemetry.addData("Say", "Hello thomas");

        robot.rail.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rail.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rail.setTargetPosition((int) RAIL_MIN);

        addButton(() -> gamepad1.right_bumper, () -> slowmode = !slowmode);
        addButton(() -> gamepad2.left_bumper, () -> {
            if (railPosition == RAIL_MIN) railPosition = RAIL_MAX;
            else railPosition = RAIL_MIN;
        });
        addButton(() -> gamepad2.triangle,
                () -> {
                    if (armPosition == ARM_UP) armPosition = ARM_DOWN;
                    else armPosition = ARM_UP;
        });
        addButton(() -> gamepad2.circle, () -> {
            clawOpen = !clawOpen;
            robot.claw.setPosition(clawOpen ? CLAW_OPEN : CLAW_CLOSED);
        });
    }

    // Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
    @Override
    public void init_loop() {
    }

    // Code to run ONCE when the driver hits PLAY
    @Override
    public void start() {
    }

    public void wheels() {

        double final_throttle = 0.0f;
        double final_strafe = 0.0f;
        double final_yaw = 0.0f;
        double joystickMultiplier = !slowmode ? 1.0f : 0.25f;


        final_throttle += (gamepad1.left_stick_y * joystickMultiplier);
        final_strafe += (gamepad1.left_stick_x * joystickMultiplier);
        final_yaw += (gamepad1.right_stick_x * joystickMultiplier);

        robot.lfDrive.setPower(final_throttle - final_strafe - final_yaw);
        robot.lbDrive.setPower(final_throttle + final_strafe - final_yaw);
        robot.rfDrive.setPower(-final_throttle - final_strafe - final_yaw);
        robot.rbDrive.setPower(-final_throttle + final_strafe - final_yaw);
    }

    public void hanging() {

        double input_power = (gamepad2.right_trigger - gamepad2.left_trigger) * 20.0f;

        robot.hangDrive.setPower(input_power);
    }

    @SuppressLint("DefaultLocale")
    public void rail() {
        railPosition += (gamepad2.right_trigger - gamepad2.left_trigger) * 20.0f;
        // Clamps rail position based on max and min values
        railPosition = Math.min(Math.max(railPosition, RAIL_MIN), RAIL_MAX);

        telemetry.addLine(String.format("Target RAIL Position: %d", (int) railPosition));
        if (robot.rail.getCurrentPosition() > railPosition) robot.rail.setPower(0.4);
        else robot.rail.setPower(0.8);

        robot.rail.setTargetPosition((int) railPosition);
    }

    @SuppressLint("DefaultLocale")
    public void arm() {

        if (gamepad2.dpad_up) armPosition -= 2.5;
        if (gamepad2.dpad_down) armPosition += 2.5;

        armPosition = Math.min(Math.max(armPosition, ARM_DOWN), ARM_UP);

        telemetry.addLine(String.format("Current ARM Position: %d", robot.arm.getCurrentPosition()));
        telemetry.addLine(String.format("Target ARM Position: %d", (int) armPosition));
        robot.arm.setTargetPosition((int) armPosition);
    }

    @SuppressLint("DefaultLocale")
    public void claw() {

        telemetry.addLine(String.format("Target CLAW Position: %f", clawPosition));
//        robot.claw.setPosition(clawPosition);
        telemetry.addLine(String.format("Current CLAW Position: %f", robot.claw.getPosition()));

    }

    // Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    @Override
    public void loop() {
        // Final Robot Instructions
        wheels();
        arm();
        rail();
        claw();
        hanging();

        doButtonPresses();

        telemetry.update();
    }

    // Code to run ONCE after the driver hits STOP
    @Override
    public void stop() {
    }
}
