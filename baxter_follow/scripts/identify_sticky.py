import sys, os
from collections import deque
import numpy as np
import scipy.io as sio
#!/usr/local/bin/python

from copy import deepcopy
from threading import RLock, Timer
import time
from math import pi
from baxter_interface.limb import Limb
from rad_baxter_limb import RadBaxterLimb
from baxter_pykdl import baxter_kinematics as b_kin
import rospy
import tf

from sensor_msgs.msg import Image
import baxter_interface
import cv
import cv2
import cvbridge

# right camera call back function
def right_camera_callback(self, data):
    self.camera_callback(data, "Right Hand Camera")

def main():
	rospy.init_node('identify_sticky') 	#start identify_sticky node
	limb = RadBaxterLimb('right')	#load limb

	# create subscriber to the required camera
	callback = self.right_camera_callback
	camera_str = "/cameras/right_hand_camera/image"
	camera_sub = rospy.Subscriber(camera_str, Image, callback)	#init subscriber
    cam_pub = rospy.Publisher('cam_data', String, queue_size=1)

    while not rospy.is_shutdown():
        if count < 15
            hello_str = "Count is %s and no sticky note" % count
        else
            hello_str = "Count is %s and no sticky note" % count

        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        count = count + 1
        rate.sleep()
	#cam = baxter_interface.camera.CameraController("right_hand_camera")
	#cam.open()

	#SUDO CODE:

	#start identify_sticky node
	#init subscriber
	#load limb
	#start camera
	#get image from right hand camera
	#convert RGB image to HSV image
	#convert HSV image to binary using OpenCV inRange() function
	#publish msg to topic sticky_sight topic if threshold exists


    # #modify this as you want
    # joint_command = [0, 0, 0, 0, 0, 0, 0]
    # while not rospy.is_shutdown():
    #     control_rate = rospy.Rate(500)
    #     limb.set_joint_positions_mod(joint_command)
    #     control_rate.sleep()

if __name__ == '__main__':
	main()
