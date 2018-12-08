import sys, os
from collections import deque
import numpy as np
#!/usr/local/bin/python

#from copy import deepcopy #for copying numpy arrays as seperate variables
#from threading import RLock, Timer
#import time
from math import pi
#from baxter_interface.limb import Limb
#from rad_baxter_limb import RadBaxterLimb
#from baxter_pykdl import baxter_kinematics as b_kin
import rospy
import tf

from sensor_msgs.msg import Image
from sensor_msgs.msg import Range
from std_msgs.msg import String
from baxter_follow.msg import CamArray
#import baxter_interface
import cv2
import cv_bridge

	#SUDO CODE:

	#start identify_sticky node
	#init subscriber
	#load limb
	#start camera
	#get image from right hand camera
	#convert RGB image to HSV image
	#convert HSV image to binary using OpenCV inRange() function
	#publish msg to topic sticky_sight topic if threshold exists

global img
global ir

def ir_callback(message):
    global ir
    # Convert image from a ROS image message to a CV image
    ir = message.range()
    # Desirable range is probably around 0.2 meters

def camera_callback(message):
    global img
    # Convert image from a ROS image message to a CV image
    bridge = CvBridge()
    img = bridge.imgmsg_to_cv2(message, "bgr8")

def main():
    rospy.init_node('identify_sticky') 	#start identify_sticky node
#    limb = RadBaxterLimb('right')	#load limb

    # create subscriber to the required camera
    camera_str = "/cameras/right_hand_camera/image"
    ir_str = "/robot/range/right_hand_range/state"
#    camsub = rospy.Subscriber(camera_str, Image, camera_callback)	#init camera subscriber
#    camsub = rospy.Subscriber(ir_str, Range, ir_callback)	#init ir rangefinder subscriber
    cam_pub = rospy.Publisher('cam_data', CamArray, queue_size=1)
    global img
    global ir

    while not rospy.is_shutdown():
        img = cv2.imread("/home/haydenm2/dev_ws/src/baxter_follow/other/stickyhand.jpeg")  #load sample image for debugging
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)  #convert RGB to HSV image

        lower_yel = np.array([23,150,50]) #yellow lower bound definition
        upper_yel = np.array([35,255,255]) #yellow upper bound definition

        mask = cv2.inRange(hsv, lower_yel, upper_yel)
        res = cv2.bitwise_and(img,img, mask= mask)

        if cv2.countNonZero(mask) <= 10:
#            print "No Sticky Note Detected"    #for debugging
            sticky = 0
            height, width = img.shape[:2]
            offx = 0
            offy = 0
            ir = 0.2
            rospy.loginfo("NO STICY DETECTED")
        # display the image

        else:
#            print "STICKY NOTE DETECTED!"      #for debugging
            sticky = 1
            ir = 0.3

###-------blob detection algorithm from
###-------https://www.learnopencv.com/find-center-of-blob-centroid-using-opencv-cpp-python/

            # convert the image to grayscale
            gray_image = cv2.cvtColor(res, cv2.COLOR_RGB2GRAY)

            # convert the grayscale image to binary image
            ret,thresh = cv2.threshold(gray_image,127,255,0)

            # find contours in the binary image
            im2, contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
            for c in contours:
               # calculate moments for each contour
               M = cv2.moments(c)

               # calculate x,y coordinate of center
               cX = int(M["m10"] / M["m00"])
               cY = int(M["m01"] / M["m00"])
               cv2.circle(img, (cX, cY), 5, (0, 0, 255), -1)
               height, width = img.shape[:2]
               offx = int(width/2-cX)
               offy = int(height/2-cY)
               rospy.loginfo("STICKY DETECTED!!!")
#               rospy.loginfo("x= %s y= %s width= %s height= %s", cX, cY, width, height)
#               rospy.loginfo("x-offset= %s  y-offset= %s", offx, offy)
           # display the image
#           cv2.imshow("Image", img)
#           cv2.waitKey(0)
        arr = CamArray()
        arr.visible = sticky
        arr.xoffset = float(offx)/float(width/2)
        arr.yoffset = float(offy)/float(height/2)
        arr.img_width = width
        arr.img_height = height
        arr.distance = ir
#        cv2.imshow('frame',img)  #show original image
#        cv2.imshow('mask',mask)  #show mask to be applied
#        cv2.imshow('res',res)    #show resulting image
#        cv2.waitKey(0)           #wait until key press
#        cv2.destroyAllWindows()  #kill all open image windows

        #rospy.loginfo(str)
        cam_pub.publish(arr)
        rate = rospy.Rate(10) # 10hz
        rate.sleep()
	#cam = baxter_interface.camera.CameraController("right_hand_camera")
	#cam.open()



if __name__ == '__main__':
	main()
