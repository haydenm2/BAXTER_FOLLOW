ystem level imports

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

	cam = baxter_interface.camera.CameraController("right_hand_camera")
	cam.open()

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




	msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
	# image directory
        self.image_dir = image_directory
	# required hue accuracy/threshold
	   self.color_tolerance = 5
	   self.hue = 57
	   self.sat = 63
	   self.val = 55
	# callback image
        self.cv_image = cv.CreateImage((self.width, self.height), 8, 3)


	# colours
        self.white = (255, 255, 255)
        self.black = (0, 0, 0)
	# reset cameras
        self.reset_cameras()

        # close all cameras
        self.close_camera("left")
        self.close_camera("right")
        self.close_camera("head")

        # open required camera
        self.open_camera(self.limb, self.width, self.height)

        # subscribe to required camera
        self.subscribe_to_camera(self.limb)

        # reset cameras
        self.reset_cameras()

        # close all cameras
        self.close_camera("left")
        self.close_camera("right")
        self.close_camera("head")

        # open required camera
        self.open_camera(self.limb, self.width, self.height)

        # subscribe to required camera
        self.subscribe_to_camera(self.limb)

        # distance of arm to table and ball tray
        self.distance      = distance
        self.tray_distance = distance - 0.075

        # move other arm out of harms way
        if arm == "left":
            self.baxter_ik_move("right", (0.25, -0.50, 0.2, math.pi, 0.0, 0.0))
        else:
            self.baxter_ik_move("left", (0.25, 0.50, 0.2, math.pi, 0.0, 0.0))

    # reset all cameras (incase cameras fail to be recognised on boot)
    def reset_cameras(self):
        reset_srv = rospy.ServiceProxy('cameras/reset', std_srvs.srv.Empty)
        rospy.wait_for_service('cameras/reset', timeout=10)
        reset_srv()

    # open a camera and set camera parameters
    def open_camera(self, camera, x_res, y_res):
        if camera == "left":
            cam = baxter_interface.camera.CameraController("left_hand_camera")
        elif camera == "right":
            cam = baxter_interface.camera.CameraController("right_hand_camera")
        elif camera == "head":
            cam = baxter_interface.camera.CameraController("head_camera")
        else:
            sys.exit("ERROR - open_camera - Invalid camera")

        # close camera
        cam.close()

        # set camera parameters
        cam.resolution          = (int(x_res), int(y_res))
        cam.exposure            = -1             # range, 0-100 auto = -1
        cam.gain                = -1             # range, 0-79 auto = -1
        cam.white_balance_blue  = -1             # range 0-4095, auto = -1
        cam.white_balance_green = -1             # range 0-4095, auto = -1
        cam.white_balance_red   = -1             # range 0-4095, auto = -1

        # open camera
        cam.open()

    # close a camera
    def close_camera(self, camera):
        if camera == "left":
            cam = baxter_interface.camera.CameraController("left_hand_camera")
        elif camera == "right":
            cam = baxter_interface.camera.CameraController("right_hand_camera")
        elif camera == "head":
            cam = baxter_interface.camera.CameraController("head_camera")
        else:
            sys.exit("ERROR - close_camera - Invalid camera")

        # set camera parameters to automatic
        cam.exposure            = -1             # range, 0-100 auto = -1
        cam.gain                = -1             # range, 0-79 auto = -1
        cam.white_balance_blue  = -1             # range 0-4095, auto = -1
        cam.white_balance_green = -1             # range 0-4095, auto = -1
        cam.white_balance_red   = -1             # range 0-4095, auto = -1

        # close camera
        cam.close()

    # convert Baxter point to image pixel
    def baxter_to_pixel(self, pt, dist):
        x = (self.width / 2)                                                         \
          + int((pt[1] - (self.pose[1] + self.cam_y_offset)) / (self.cam_calib * dist))
        y = (self.height / 2)                                                        \
          + int((pt[0] - (self.pose[0] + self.cam_x_offset)) / (self.cam_calib * dist))

        return (x, y)

    # convert image pixel to Baxter point
    def pixel_to_baxter(self, px, dist):
        x = ((px[1] - (self.height / 2)) * self.cam_calib * dist)                \
          + self.pose[0] + self.cam_x_offset
        y = ((px[0] - (self.width / 2)) * self.cam_calib * dist)                 \
          + self.pose[1] + self.cam_y_offset

        return (x, y)

	while len(to_do) > 0:
            x, y = to_do.pop()                             # get next pixel to test
            if cv.Get2D(image, y, x)[0] == self.black[0]:  # if black pixel found
                pixel_list.append([x, y])                  # save pixel position
                cv.Set2D(image, y, x, almost_black)        # set pixel to almost black
                to_do.append([x, y - 1])                   # add neighbours to to do list
                to_do.append([x, y + 1])
                to_do.append([x - 1, y])
                to_do.append([x + 1, y])

# camera call back function
    def camera_callback(self, data, camera_name):
        # Convert image from a ROS image message to a CV image
        try:
            self.cv_image = cv_bridge.CvBridge().imgmsg_to_cv(data, "bgr8")
        except cv_bridge.CvBridgeError, e:
            print e

        # 3ms wait
        cv.WaitKey(3)

    # left camera call back function
    def left_camera_callback(self, data):
        self.camera_callback(data, "Left Hand Camera")
