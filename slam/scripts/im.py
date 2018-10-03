#! /usr/bin/env python

__author__ =  'Simon Haller <simon.haller at uibk.ac.at>'
__version__=  '0.1'
__license__ = 'BSD'
# Python libs
import sys, time

# numpy and scipy
import numpy as np
from scipy.ndimage import filters
import math

# OpenCV
import cv2

# Ros libraries
import roslib
import rospy
import tf

# Ros Messages
from sensor_msgs.msg import CompressedImage
# We do not use cv_bridge it does not support CompressedImage in python
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from std_msgs.msg import String

VERBOSE=False

def Constrain(val, MAX_val, MIN_val):
    if val > MAX_val:
        return MAX_val
    elif val < MIN_val:
        return MIN_val
    else:
        return val

class image_feature:

    def __init__(self):
        '''Initialize ros publisher, ros subscriber'''
        # topic where we publish
        self.image_pub = rospy.Publisher("/output/image_raw/compressed", CompressedImage)
        self.bridge = CvBridge()

        # subscribed Topic
        self.subscriber = rospy.Subscriber("/camera/image/compressed", CompressedImage, self.callback,  queue_size = 5)
        if VERBOSE :
            print "subscribed to /camera/image/compressed"

        self._broadcaster = tf.TransformBroadcaster()

        self._camera_pub = rospy.Publisher('chatter', String, queue_size=10)
        self.servo = [90.0, 90.0]


    def callback(self, ros_data):
        aruco = cv2.aruco
        dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        arucoMarkerLength = 0.096
        PI = 3.1415
        self.cameraMatrix = np.asarray([[255.64885718, 0., 163.44315533], [0., 255.85790733, 118.62161079], [0., 0., 1.]])
        self.distanceCoefficients = np.asarray( [0.20120402,-0.43881712, 0.00463485, 0.00622489, 0.12081623])

        '''Callback function of subscribed topic. 
        Here images get converted and features detected'''
        if VERBOSE :
            print 'received image of type: "%s"' % ros_data.format

        #### direct conversion to CV2 ####
        np_arr = np.fromstring(ros_data.data, np.uint8)
        # image_np = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # OpenCV >= 3.0:

        # hsv_img = cv2.cvtColor(image_np, cv2.COLOR_BGR2HSV)

        # thresholded_img = cv2.inRange(hsv_img, (35, 80, 110), (60, 255,  255),)

        corners, ids, rejectedImgPoints = aruco.detectMarkers(image_np, dictionary)
        aruco.drawDetectedMarkers(image_np, corners, ids, (0,255,0))

        if len(corners) > 0:
            self.rvec, self.tvec, _ = aruco.estimatePoseSingleMarkers(corners[0], arucoMarkerLength, self.cameraMatrix, self.distanceCoefficients)
            image_np = aruco.drawAxis(image_np, self.cameraMatrix, self.distanceCoefficients, self.rvec, self.tvec, 0.1)
            # (roll_angle, pitch_angle, yaw_angle) =  self.rvec[0][0][0], self.rvec[0][0][1], self.rvec[0][0][2]
            # if pitch_angle < 0:
            #     roll_angle, pitch_angle, yaw_angle = -roll_angle, -pitch_angle, -yaw_angle
            # print (roll_angle, pitch_angle, yaw_angle)
            # print(self.tvec)
            _quat = tf.transformations.quaternion_from_euler(-PI / 2.0 , -PI , PI / 2.0)
            # _quat = tf.transformations.quaternion_from_euler(-PI / 2.0 + (roll_angle + PI), -PI + pitch_angle, PI / 2.0 + yaw_angle)

            # _quat = (0., 0., 0., 1.)
            self._broadcaster.sendTransform((self.tvec[0][0][2], self.tvec[0][0][0], self.tvec[0][0][1]), _quat, rospy.Time.now(), "ar", "camera_face")
            # self._broadcaster.sendTransform((-0, 0, 0), _quat, rospy.Time.now(), "base_link", "ar")
            self.servo[0] -= 90.0 - math.degrees(math.atan2(self.tvec[0][0][2], self.tvec[0][0][0]))
            self.servo[1] += 90.0 - math.degrees(math.atan2(self.tvec[0][0][2], self.tvec[0][0][1]))

        self.servo[1] = Constrain(self.servo[1], 110, 30)
        self.servo[0] = Constrain(self.servo[0], 135, 45)

        self._camera_pub.publish(str(self.servo[0]) + ", " + str(self.servo[1]) + ",")
        # print(self.servo)


        # cv2.imshow('cv_img', image_np)
        # cv2.waitKey(2)

        #### Create CompressedIamge ####
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', image_np)[1]).tostring()
        # Publish new image
        self.image_pub.publish(msg)

        #self.subscriber.unregister()

def main(args):
    '''Initializes and cleanup ros node'''
    ic = image_feature()
    rospy.init_node('image_feature', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Image feature detector module"
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)