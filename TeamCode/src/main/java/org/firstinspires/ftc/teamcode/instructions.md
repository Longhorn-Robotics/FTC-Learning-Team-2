Court Layout:
Ok, essentially, there is a hoop/target at the top right corneer of the court. On that is a april tag (since it is in a corner, it is facing 45 degrees). Then, there are 4 rows of balls, each with 3 balls in it lined up such that each row is perpendicular to the right wall. Our bot can only hold 3 balls at a time. In 30 seconds, we need to pick up as many balls as possible and launch it into the hoop. there are two scoring zones, which are areas we are allowed to launch from. Our scroing zone is a right triange, with one base on the top of the court, and the hypotnuse extneding outward straight from the center of the hoop (same direction as april tag), and anohte rbase on the ohter side of this hypotnuse connecting back to the top. Us this to your advantge when writing the scripot, and ont worry about the actua lturn on/off itnake and luancher code for now, jsut leave a blank space where I can add it later. Let me know if you need any dimesniosn, but hopefully you won't need it since it is camera based (so just leave variables I can fill in numbers for if needed). By the way, when I refrence top, bottom, left and right sides of the court, that was form a top down view. Let me know if you havea ny questiosn, and your revisions to your plan.  

Code rules:
You may NOT edit the RobotHardware.java file, and you MUST use it.
You may NOT edit any files OTHER THAN RobotAutonomous4.java
The code must always have a lot of telemtry for debugging the code
This is using the FTC SDK with an Adnroid as a Control Hub
Please use the RobotAutonomous.java as a refrence for how to program difernet thigns, but don't restrict yourself to it. You may also use RobotAutonomous3 as a refrence, but keep in mind it doesn't work for some unknow reasons. 


Robots Actions Plan (This is what the robot mus tdo in each step):
Note: When I say it goes left, rught, up, or down, that means from a top down perspective of the court assiming the goals are in the top left and right corners (we ar ecurrnetly coding for the right corner)
1. Using the camera, the robot must make sure it is properly aligned with the AprilTag
2. The robot must go backwards/forwards until it is at a point such that if it went down the court for some X distance, it would be right next to the first ball row
3. The robot must turn so it is facing upwards and reverse until it reaches the first ball row. You must use the AprilTag as an anchor to make sure you are going the right way, but keep in mind that the AprilTag is at a 45 degree angle, so the code must be able take this into account (as it is going paralel to the wall)
4. Once the robot is next to the ball row, the robot will turn CW until it is facing the ball row. We will use use the IMU/Encoders to make sur eit is facing the right direction, and we can also use the Camera to detect the balls for percision, bu tkeep in mind there were lilely be people snd object outside of the court with simialr colors, so we don't want it to drive there, which is why the IMU/Encoders can make sure it is in roughly the right direction while the camera is only for percsison. The robot wil lgo forward for some X Distacne to pick up the balls, as we have an active intake.
5. Once the robot collects the ball row, it wil reverse until it is at the pont where it originally tuned CW to pick up the balls. The robot will turn CCW and use the AprilTag as an ahcor to return to the launch zone.
6. The robot will turn CW until it is facing the AprilTag

IMPORTANT:
For Steps 3 and 5, and 6, it is very likely the robot will lose sigt of the AprilTag. When this happens, the robot must remember it's lcation/heading/distacne or whatever so it can drive purely on the IMU and encoders UNTIL it sees the AprilTag again.