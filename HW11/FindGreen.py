import time
import roslib
import rospy
import sys
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class ShowCamera():
    def __init__(self):
        # initiliaze
        rospy.init_node('ShowCamera', anonymous=False)
        self.bridge = CvBridge()
        self.lower = np.array([180,235,45], dtype = "uint8")
        self.upper = np.array([245,255,60], dtype = "uint8")
        # rospy.init_node('CheckOdometry', anonymous=False)
        # we want a straight line with a phase of straight
        rgb = rospy.Subscriber('/camera/rgb/image_raw', Image, self.displayRGB)
        # tell user how to stop TurtleBot
        rospy.loginfo("To stop TurtleBot CTRL + C")

        rospy.spin()

    def displayRGB(self,msg):
        #rospy.loginfo("Received Image Data")
        try:
            image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            rospy.loginfo("Converted color to cv2 image")
            mask = cv2.inRange(image, self.lower, self.upper)
            #output = cv2.bitwise_and(image, image, mask = mask)
            cv2.imshow("RBG Window", mask)
            #cv2.imwrite('TestImage.png',image)
            cv2.waitKey(1)
	    #cv2.destroyWindow("RGB Window")
        except CvBridgeError as e:
            print(e)

if __name__ == '__main__':
    #try:
    ShowCamera()
