package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;

//this file will take the data of the location of the circles, along with the april tag, and
//create a route/ commands for the robot to go in. For now, we will ust make it go to the nearest ball
//and do the proper route planning and april tag tracker later

public class createRoute {
    public createRoute(){};
    public float generateRoute (float[][] circles) {
        //assuming circles is an array in the format
        //{{{x,y}}{radius}, ...}

        //this will be in the follwoing format
        //{{distance to travel}{angle to travel in), ...}
        //where each array in the array is a ball the robot needs to travel to

        //float[][] path = new float[3][2];
        float[][] path = new float[1][2];

        int largest_circle = 0;
        //find the closest ball in the array
        for (int i = 0; i<arr.length; i++){
            if (circles[i][1] > circles[largest_circle][1]){
                largest_circle = i;
            }
        }








        return path;
    }
}
