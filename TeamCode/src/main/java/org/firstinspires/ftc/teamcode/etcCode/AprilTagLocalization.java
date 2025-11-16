///* Copyright (c) 2024 Dryw Wade. All rights reserved.
// *
// * Redistribution and use in source and binary forms, with or without modification,
// * are permitted (subject to the limitations in the disclaimer below) provided that
// * the following conditions are met:
// *
// * Redistributions of source code must retain the above copyright notice, this list
// * of conditions and the following disclaimer.
// *
// * Redistributions in binary form must reproduce the above copyright notice, this
// * list of conditions and the following disclaimer in the documentation and/or
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
//package org.firstinspires.ftc.teamcode.etcCode;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.Position;
//import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
//import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
//import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
//
//import java.util.List;
//
//
//
////DETAILS AND COMMENTS IN       >>>>>>>   ConceptAprilTagLocalization    <<<<<<<<<<<<<<
//public class AprilTagLocalization {
//
//    private static final boolean USE_WEBCAM = false;  // true for webcam, false for phone camera
//
//    private LinearOpMode localOpMode = null;
//    public AprilTagLocalization(LinearOpMode opmode) {
//        localOpMode = opmode;
//    }
//
//
//
//
//    private Position cameraPosition = new Position(DistanceUnit.INCH,
//            0, 0, 0, 0);
//    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
//            0, -90, 0, 0);
//    private AprilTagProcessor aprilTag;
//    private VisionPortal visionPortal;
//
//
//    public void initAprilTag() {
//
//        aprilTag = new AprilTagProcessor.Builder()
//                .setCameraPose(cameraPosition, cameraOrientation)
//                .build();
//
//        VisionPortal.Builder builder = new VisionPortal.Builder();
//        builder.setCamera(BuiltinCameraDirection.BACK);
//        builder.addProcessor(aprilTag);
//        // Build the Vision Portal, using the above settings.
//        visionPortal = builder.build();
//
//    }
//
//    public double[] runDetection() {
//
//        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
//        localOpMode.telemetry.addData("# AprilTags Detected", currentDetections.size());
//
//        // Step through the list of detections and display info for each one.
//        for (AprilTagDetection detection : currentDetections) {
//            if (detection.metadata != null) {
//                localOpMode.telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
//                // Only use tags that don't have Obelisk in them
//                if (!detection.metadata.name.contains("Obelisk")) {
////                    localOpMode.telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)",
////                            detection.robotPose.getPosition().x,
////                            detection.robotPose.getPosition().y,
////                            detection.robotPose.getPosition().z));
////                    localOpMode.telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)",
////                            detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES),
////                            detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES),
////                            detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)));
//
//
//                    return new double[] {detection.robotPose.getPosition().x, detection.robotPose.getPosition().y, detection.robotPose.getPosition().z, detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)};
//                }
//            } else {
////                localOpMode.telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
////                localOpMode.telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
//                return new double [] {0, 0, 0, 0};
//            }
//        }
//
//        // Add "key" information to telemetry
////        localOpMode.telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
////        localOpMode.telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
//        return new double [] {0, 0, 0, 0};
//
//
//    }
//
//}
