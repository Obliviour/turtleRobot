import time
import roslib
import rospy
import sys
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class ShowCamera():
    def __init__(self):
        # initiliaze
        rospy.init_node('ShowCamera', anonymous=False)
        self.bridge = CvBridge()
        # rospy.init_node('CheckOdometry', anonymous=False)
        # we want a straight line with a phase of straight
        rgb = rospy.Subscriber('/camera/rgb/image_raw', Image, self.displayRGB)
        depth = rospy.Subscriber('/camera/depth/image', Image, self.displayDepth)
        # tell user how to stop TurtleBot
        rospy.loginfo("To stop TurtleBot CTRL + C")

        rospy.spin()

    def displayRGB(self,msg):
        #rospy.loginfo("Received Image Data")
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            #rospy.loginfo("Converted to cv2 image")
            cv2.imshow("RBG Window", cv_image)
            cv2.waitKey(3)
        except CvBridgeError as e:
            print(e)

    def displayDepth(self,msg):
        #rospy.loginfo("Received Image Data")
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            #rospy.loginfo("Converted to cv2 image")
            cv2.imshow("Depth Window", cv_image)
            cv2.waitKey(3)
        except CvBridgeError as e:
            print(e)

if __name__ == '__main__':
    #try:
    ShowCamera()
