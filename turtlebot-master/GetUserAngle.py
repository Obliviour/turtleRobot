import rospy
from std_msgs.msg import Int16

def getAngle():
    pub = rospy.Publisher('chatter', Int16, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    #rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
    	angle = input("Enter an Angle in Degrees from 0 to 360: ")
    	if(angle > 360 || angle < 0):
    		angle = angle % 360
        pub.publish(angle)
        #rate.sleep()

if __name__ == '__main__':
    getAngle()