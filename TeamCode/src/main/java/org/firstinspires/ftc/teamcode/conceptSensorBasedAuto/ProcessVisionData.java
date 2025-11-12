//package org.firstinspires.ftc.teamcode;
//
//
////this file will take the data of the location of the circles, along with the april tag, and
////create a route/ commands for the robot to go in. For now, we will ust make it go to the nearest ball
////and do the proper route planning and april tag tracker later
//
//public class ProcessVisionData {
//    public ProcessVisionData() {
//    }
//
//    ;
//
//
//    public float[][] createMap(float[][] objects) {
//        //temp map data and data structure until actually programmed
//        float[][] map = new float[1][2];
//        return map;
//    }
//
//    public float[][] generateRoute(float[][] circles) {
//        //assuming circles is an array in the format
//        //{{{x,y}}{radius}, ...}
//
//        //this will be in the following format
//        //{{distance to travel}{angle to travel in), ...}
//        //where each array in the array is a ball the robot needs to travel to
//
//        //float[][] path = new float[3][2];
//        float[][] path = new float[1][2];
//
//
//        return path;
//    }
//
//
//    public float[] updateMapMotor(float leftMotorMovement, float rightMotorMovement, float[] map) {
//        return map;
//    }
//
//    public float[] updateMapMotor(float AprilTagData, float[] map) {
//        return map;
//    }
//}