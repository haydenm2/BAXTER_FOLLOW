#!/bin/bash

cd ~/baxter_ws   #moves you to the correct folder
./baxter.sh           #sets up environment variables correct for baxter to work
sleep 1s
rosrun baxter_tools enable_robot.py -r   #this resets the robot state if you pushed the e-stop
sleep 1s
rosrun baxter_tools enable_robot.py -e   #this re-enables the robot, and it may make a noise
sleep 1s
rosrun baxter_tools tuck_arms.py -u        #this will untuck his arms
