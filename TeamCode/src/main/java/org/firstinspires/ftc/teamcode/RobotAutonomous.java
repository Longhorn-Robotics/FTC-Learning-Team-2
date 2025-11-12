//import libraries
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;

import java.util.List;


//Declare teleop
@Autonomous(name = "Auto Control", group  = "Robot")


//init and run our teleop
public class RobotAutonomous extends LinearOpMode {

    //Access our robot hardware
    RobotHardware robot = new RobotHardware();
    //trackArtifacts tracker = new trackArtifacts(this);

    AprilTagLocalization getPos = new AprilTagLocalization(this);
    RobotAutoDriveToAprilTagTank driveApril = new RobotAutoDriveToAprilTagTank (this);
    //run our linear op mode
    @Override
    public void runOpMode() {
        //init hardware
        robot.init(hardwareMap);
        getPos.initAprilTag();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        sleep(10000);
        //tracker.runDetection();

        String state = "Launch";
        //#wait fo the user to press the Play button
        waitForStart();

        while (opModeIsActive()) {
            //read controller data



            switch (state) {
                case "Launch":
                    double[] position = getPos.runDetection();
                    driveApril.runOpMode(hardwareMap);
                    break;
                default:
                    break;
            }






        }

    }
}