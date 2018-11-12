import time
import roslib
import rospy
import sys
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class TrackRed:
    def __init__(self):
        # initialize
        rospy.init_node('ShowCamera', anonymous=False)
        self.bridge = CvBridge()
        self.lower = np.array([10,1,100], dtype = "uint8")
        self.upper = np.array([38,15,255], dtype = "uint8")
        # rospy.init_node('CheckOdometry', anonymous=False)
        # we want a straight line with a phase of straight
        rgb = rospy.Subscriber('/camera/rgb/image_raw', Image, self.display_rgb)
        odom = rospy.Subscriber('odom', Odometry, self.OdometryCallBack)
        # tell user how to stop TurtleBot
        self.threshold = 0
        self.avgx = 0
        self.avgy = 0
        rospy.loginfo("To stop TurtleBot CTRL + C")

        rospy.spin()

    def display_rgb(self, msg):
        """display rgb information"""
        # rospy.loginfo("Received Image Data")
        try:
            image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            rospy.loginfo("Converted color to cv2 image")
            mask = cv2.inRange(image, self.lower, self.upper)
            cv2.imshow("RBG Window", mask)
            cv2.waitKey(1)
        except CvBridgeError as e:
            print(e)


if __name__ == '__main__':
    #try:
    ShowCamera()
