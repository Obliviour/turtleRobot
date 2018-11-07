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
        #depth = rospy.Subscriber('/camera/depth/image', Image, self.displayDepth)
        # tell user how to stop TurtleBot
        rospy.loginfo("To stop TurtleBot CTRL + C")
        rospy.on_shutdown(self.shutdown)

        rospy.spin()

    def self.displayRGB(self,msg):
        rospy.loginfo("Received image data")
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg.data, msg.encoding)
            rospy.loginfo("Converted to cv2 image")
            cv2.imshow("Image window", cv_image)
        except CvBridgeError as e:
            print(e)

if __name__ == '__main__':
    #try:
    ShowCamera()