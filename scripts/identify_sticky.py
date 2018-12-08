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
from cv_bridge import CvBridge, CvBridgeError

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
    ir = message.range

    # Desirable range is probably around 0.2 meters
#    rospy.loginfo("receiving ir %s", ir)

def camera_callback(message):
    global img
    # Convert image from a ROS image message to a CV image
    try:
        bridge = CvBridge()
        img = bridge.imgmsg_to_cv2(message, "bgr8")
        #rospy.sleep(0.001)
#        rospy.loginfo("receiving images")
    except CvBridgeError, e:
        print(e)
#        rospy.loginfo("receiving images error")
    

def main():
    rospy.init_node('identify_sticky') 	#start identify_sticky node
#    limb = RadBaxterLimb('right')	#load limb

    # create subscriber to the required camera
    camera_str = "/cameras/right_hand_camera/image"
    ir_str = "/robot/range/right_hand_range/state"
    camsub = rospy.Subscriber(camera_str, Image, camera_callback, queue_size=1)	#init camera subscriber
    irsub = rospy.Subscriber(ir_str, Range, ir_callback, queue_size=1)	#init ir rangefinder subscriber
    cam_pub = rospy.Publisher('cam_data', CamArray, queue_size=1)
    global img
    global ir
    ir = 1
    rospy.sleep(0.1)
    while not rospy.is_shutdown():
        rospy.loginfo("Identify Sticky Running")

#        img = cv2.imread("/home/radlab/baxter_hayden/src/baxter_follow/other/sticky.png")  #load sample image for debugging
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)  #convert RGB to HSV image
        lower_yel = np.array([23,50,50]) #yellow lower bound definition
        upper_yel = np.array([50,255,255]) #yellow upper bound definition

        lower = np.array([0,0,0]) #yellow lower bound definition
        upper = np.array([255,1,255]) #yellow upper bound definition

        mask = cv2.inRange(hsv, lower_yel, upper_yel)
        res = cv2.bitwise_and(img,img, mask= mask)
        gray_image = cv2.cvtColor(res, cv2.COLOR_RGB2GRAY)

        
        if cv2.countNonZero(mask) <= 20:
#            print "No Sticky Note Detected"    #for debugging
            sticky = 0
            height, width = img.shape[:2]
            offx = 0
            offy = 0
#            rospy.loginfo("NO STICY DETECTED")
            if ir > 60:
                ir = 0.2
        # display the image

        else:
            #rospy.loginfo("------------DEBUG SIGNAL-------------------")
#            print "STICKY NOTE DETECTED!"      #for debugging
            sticky = 1

###-------blob detection algorithm from
###-------https://www.learnopencv.com/find-center-of-blob-centroid-using-opencv-cpp-python/

            # convert the image to grayscale
            gray_image = cv2.cvtColor(res, cv2.COLOR_RGB2GRAY)
            #cv2.imshow("Image", gray_image)
            #cv2.waitKey(0.1)
            # convert the grayscale image to binary image
            ret,thresh = cv2.threshold(gray_image,127,255,cv2.THRESH_BINARY)

            # find contours in the binary image
            contours, hierarchy = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

            offx = 0
            offy = 0
            height, width = img.shape[:2]

            for c in contours:
                # calculate moments for each contour
                M = cv2.moments(c)
                #rospy.loginfo("m10: %s m01: %s m00: %s mdiv:",M["m10"],M["m01"],M["m00"])
                #rospy.loginfo("mdiv: %s",M["m10"]/M["m00"])
                # calculate x,y coordinate of center
                if(M["m00"] != 0):
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                else:
                    cX = int(width/2)
                    cY = int(height/2)
                cv2.circle(img, (cX, cY), 5, (0, 0, 255), -1)
                offx = int(cX-width/2)
                offy = int(height/2-cY)
                break
#                rospy.loginfo("STICKY DETECTED!!!")
            if ir > 60:
                ir = 0.4
    #               rospy.loginfo("x= %s y= %s width= %s height= %s", cX, cY, width, height)
    #               rospy.loginfo("x-offset= %s  y-offset= %s", offx, offy)

####UNCOMMENT FOR IMAGE DISPLAY TROUBLESHOOTING
#        cv2.imshow("Image", mask)
#        cv2.waitKey(20)
#        rospy.sleep(1)
#        cv2.destroyAllWindows()

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


if __name__ == '__main__':
	main()
