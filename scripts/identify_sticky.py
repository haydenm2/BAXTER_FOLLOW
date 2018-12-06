import sys, os
from collections import deque
import numpy as np
#!/usr/local/bin/python

#from copy import deepcopy
#from threading import RLock, Timer
#import time
from math import pi
#from baxter_interface.limb import Limb
#from rad_baxter_limb import RadBaxterLimb
#from baxter_pykdl import baxter_kinematics as b_kin
import rospy
import tf

from sensor_msgs.msg import Image
from std_msgs.msg import String
#import baxter_interface
import cv2
#import cvbridge

# right camera call back function
#def right_camera_callback(self, data):
#    self.camera_callback(data, "Right Hand Camera")

def main():
    rospy.init_node('identify_sticky') 	#start identify_sticky node
    #limb = RadBaxterLimb('right')	#load limb

    # create subscriber to the required camera
    #callback = self.right_camera_callback
    camera_str = "/cameras/right_hand_camera/image"
    #camera_sub = rospy.Subscriber(camera_str, Image, callback)	#init subscriber
    cam_pub = rospy.Publisher('cam_data', String, queue_size=1)
    count = 1
    while not rospy.is_shutdown():
        # read image through command line
        img = cv2.imread("/home/haydenm2/dev_ws/src/baxter_follow/other/sticky.jpg")

        # convert the image to grayscale
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        lower_yel = np.array([25,100,150])
        upper_yel = np.array([35,255,255])

        mask = cv2.inRange(hsv, lower_yel, upper_yel)
        res = cv2.bitwise_and(img,img, mask= mask)
        img = cv2.imread('/Documents/2016.jpg')
        if cv2.countNonZero(mask) <= 10:
#            print "No Sticky Note Detected"
            str = "No Sticky Note Detected"
        else:
#            print "STICKY NOTE DETECTED!"
            str = "STICKY NOTE DETECTED!"

#        cv2.imshow('frame',img)  #show original image
#        cv2.imshow('mask',mask)  #show mask to be applied
#        cv2.imshow('res',res)    #show resulting image
#        cv2.waitKey(0)           #wait until key press
#        cv2.destroyAllWindows()  #kill all open image windows

        #rospy.loginfo(str)
        cam_pub.publish(str)
#        count = count + 1
        rate = rospy.Rate(10) # 10hz
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
