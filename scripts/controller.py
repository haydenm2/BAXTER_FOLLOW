import sys, os
from collections import deque
import numpy as np
#!/usr/local/bin/python

#from copy import deepcopy #for copying numpy arrays as seperate variables
#from threading import RLock, Timer
#import time
from math import pi
from baxter_interface.limb import Limb
from rad_baxter_limb.rad_baxter_limb import RadBaxterLimb
from baxter_pykdl import baxter_kinematics as b_kin
import rospy
import tf

from sensor_msgs.msg import Image
from sensor_msgs.msg import Range
from std_msgs.msg import String
from baxter_follow.msg import CamArray
import baxter_interface
import cv2
from cv_bridge import CvBridge, CvBridgeError

rospy.init_node('controller')

limb = RadBaxterLimb('right')
startPose = limb.get_kdl_forward_position_kinematics()
# startR = tf.transformations.quaternion_matrix(startPose[3:])[0:3,0:3]


global visible
global xOffset
global yOffset
global distance

def camera_callback(message):
    global visible
    global xOffset
    global yOffset
    global distance

    visible = message.visible
    xOffset = message.xoffset
    yOffset = message.yoffset
    distance = message.distance

    rospy.loginfo('%s %s %s %s',xOffset, yOffset, distance, visible)

def main():
    global visible
    global xOffset
    global yOffset
    global distance

    xOffset = .5
    yOffset = -.5
    errorMag = sqrt(xOffset^2 + yOffset^2)

    # rospy.sleep(1)
    camsub = rospy.Subscriber('/cam_data', CamArray, camera_callback, queue_size=1)
    c = .2

    while not rospy.is_shutdown():
	    # if visible?
	    # get fk
        fk = limb.get_kdl_forward_position_kinematics()
        # rospy.loginfo("%s",fk)
	    # scale camera data to position and find error
        error = [xOffset*c,yOffset*c]
	    # get angles
        q = limb.get_joint_angles()
        s = "***Actual q***"
        rospy.loginfo('%s',s)
        rospy.loginfo('%s',q)
	    # get pseudo inverse of J
        Jinv = limb.get_kdl_jacobian_pseudo_inverse()
        # rospy.loginfo("%s",Jinv)
        twist = np.transpose([[error[0], error[1], 0, 0, 0, 0]])
        # rospy.loginfo('%s',twist)
        q = q + np.transpose(Jinv*twist)
        q = np.squeeze(np.asarray(q))
        s2 = "***Projected q***"
        rospy.loginfo('%s',s2)
        rospy.loginfo('%s',q)
	    # command baxter to move
        # q = [0.09,-1,1.16,1.94,-.66,1.02,0]
        control_rate = rospy.Rate(500)
        limb.set_joint_positions_mod(q)
        control_rate.sleep()

if __name__ == '__main__':
	main()
