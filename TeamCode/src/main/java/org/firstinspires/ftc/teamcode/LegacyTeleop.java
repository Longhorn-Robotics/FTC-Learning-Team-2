///* Copyright (c) 2017 FIRST. All rights reserved.
// *
// * Redistribution and use in source and binary forms, with or without modification,
// * are permitted (subject to the limitations in the disclaimer below) provided that
// * the following conditions are met:
// *
// * Redistributions of source code must retain the above copyright notice, this list
// * of conditions and the following disclaimer.
// *
// * Redistributions in binary form must reproduce the above copyright notice, this
// * list of condition    s and the following disclaimer in the documentation and/or
// * other materials provided with the distribution.
// *
// * Neither the name of FIRST nor the names of its contributors may be used to endorse or
// * promote products derived from this software without specific prior written permission.
// *
// * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
// * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
// * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
// * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// */
//
//package org.firstinspires.ftc.teamcode;
//
//        import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//        import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//
//        import java.util.ArrayList;
//        import java.util.List;
//
///**
// * This file provides basic Telop driving for a Pushbot robot.
// * The code is structured as an Iterative OpMode
// *
// * This OpMode uses the common Pushbot hardware class to define the devices on
// * the robot.
// * All device access is managed through the HardwarePushbot class.
// *
// * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
// * It raises and lowers the claw using the Gampad Y and A buttons respectively.
// * It also opens and closes the claws slowly using the left and right Bumper
// * buttons.
// *
// * Use Android Studios to Copy this Class, and Paste it into your team's code
// * folder with a new name.
// * Remove or comment out the @Disabled line to add this opmode to the Driver
// * Station OpMode list
// */
//
//@TeleOp(name = "LegacyTeleop", group = "Pushbot")
//public class LegacyTeleop extends OpMode {
//    // final float DEADZONE = 1.1f;
//    /* Declare OpMode members. */
//    RobotHardware robot = new RobotHardware(); // use the class created to define a Pushbot's hardware
//
//    /*
//     * Code to run ONCE when the driver hits INIT
//     */
//    @Override
//    public void init() {
//        /*
//         * Initialize the hardware variables.
//         * The init() method of the hardware class does all the work here
//         */
//        robot.init(hardwareMap);
//        // Send telemetry message to signify robot waiting
//        telemetry.addData("Say", "Hello thomas");
//    }
//
//    /*
//     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
//     */
//    @Override
//    public void init_loop() {
//    }
//
//    /*
//     * Code to run ONCE when the driver hits PLAY
//     */
//    @Override
//    public void start() {
//    }
//
//    // Variables to make trackpad do trackpad things
//    double trackpadDifferenceX = 0;
//    double trackpadDifferenceY = 0;
//    double prevTrackpadX = 0;
//    double prevTrackpadY = 0;
//    boolean wasTouchpad = false;
//
//    double clawOffset = 0.25;
//    double currentClaw = 0;
//
//    boolean circleWasPressed = false;
//
//    /*
//     * This is Jonathan's code for fine movement
//     * If the robot isn't working try deleting this
//     * Update: this code is perfect and deleting it is a sin
//     */
//    double movemult = 1.0;
//
//    /*
//     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
//     */
//    @Override
//    public void loop() {
//        // Gamepad Inputs
//        float gp1_throttle = gamepad1.left_stick_y;
//        float gp1_strafe = gamepad1.left_stick_x;
//        float gp1_yaw = gamepad1.right_stick_x;
//        // Final Robot Instructions
//        double final_throttle = -gp1_throttle;
//        double final_strafe = gp1_strafe;
//        float final_yaw = gp1_yaw;
//
//        // double value = robot.distanceR.getUltrasonicLevel();
//
//        // telemetry.addData("distance_R: ", value);
//
//        if (Math.abs(trackpadDifferenceX) < 0.0001)
//            trackpadDifferenceX = 0;
//        if (Math.abs(trackpadDifferenceY) < 0.0001)
//            trackpadDifferenceY = 0;
//        // Trackpad magic
//        trackpadDifferenceX = trackpadDifferenceX * 0.8;
//        trackpadDifferenceY = trackpadDifferenceY * 0.8;
//        if (gamepad2.touchpad_finger_1 || trackpadDifferenceX != 0 || trackpadDifferenceY != 0) {
//            telemetry.addLine("finger detected");
//
//            if (wasTouchpad) {
//                trackpadDifferenceX += gamepad2.touchpad_finger_1_x - prevTrackpadX;
//                trackpadDifferenceY += gamepad2.touchpad_finger_1_y - prevTrackpadY;
//            }
//            final_throttle += trackpadDifferenceY * 2.5;
//            final_strafe += trackpadDifferenceX * 2.5;
//
//            telemetry.addLine(String.format("touchpad_difference_x: %s", trackpadDifferenceX));
//            telemetry.addLine(String.format("touchpad_difference_y: %s", trackpadDifferenceY));
//            prevTrackpadX = gamepad2.touchpad_finger_1_x;
//            prevTrackpadY = gamepad2.touchpad_finger_1_y;
//        }
//        wasTouchpad = gamepad2.touchpad_finger_1;
//
//        if (gamepad1.right_trigger > 0.5) {// Also Jonathan's code
//            movemult = 0.3;
//        } else {
//            movemult = 1.0;
//        }
//
//        if (gamepad2.circle) {
//            if (!circleWasPressed) {
//                circleWasPressed = true;
//                telemetry.addLine("detected toggle");
//                if (currentClaw != 0) {
//                    currentClaw = 0;
//                } else {
//                    currentClaw = -0.16;
//                }
//            }
//        } else {
//            circleWasPressed = false;
//        }
//        if (gamepad2.dpad_up)
//            currentClaw += 0.0007;
//        if (gamepad2.dpad_down)
//            currentClaw -= 0.0007;
//
//        // if (currentClaw < -0.5) currentClaw = -0.5;
//
//        // Claw Code Stuff
//        robot.clawL.setPosition(clawOffset + currentClaw + 0.21);
//        robot.clawR.setPosition(clawOffset - currentClaw + 0.39);
//
//        telemetry.addLine(String.format("claw position: %s", currentClaw));
//
//        // More Gamepad Inputs
//        double trigger_left = gamepad2.left_trigger;
//        double trigger_right = gamepad2.right_trigger;
//        // boolean bumper_left = gamepad1.left_bumper;
//        // boolean bumper_right = gamepad1.right_bumper;
//        telemetry.addLine(String.format("left_trigger: %s", trigger_left));
//        telemetry.addLine(String.format("right_trigger: %s", trigger_right));
//
//        // // robot.elevateL.setPower((trigger_right - trigger_left) * 1);
//        // // robot.elevateR.setPower((trigger_right - trigger_left) * 1);
//        // if ((trigger_right - trigger_left) > 0) {
//        // //robot.elevateDown.setPower((trigger_right - trigger_left) * -0.75);
//        // } else {
//        // //robot.elevateDown.setPower((trigger_right - trigger_left) * -1.18);
//        // }
//        robot.elevate.setPower((trigger_right - trigger_left) * 1);
//
//        // movemult is Jonathan's code, 0.7 isn't
//        robot.lfDrive.setPower((final_throttle - final_strafe - final_yaw) * 0.7 * movemult);
//        robot.lbDrive.setPower((final_throttle + final_strafe - final_yaw) * 0.7 * movemult);
//        robot.rfDrive.setPower((-final_throttle - final_strafe - final_yaw) * 0.7 * movemult);
//        robot.rbDrive.setPower((-final_throttle + final_strafe - final_yaw) * 0.7 * movemult);
//
//        telemetry.update();
//    }
//
//    /*
//     * Code to run ONCE after the driver hits STOP
//     */
//    @Override
//    public void stop() {
//    }
//}