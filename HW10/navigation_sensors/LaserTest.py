import rospy
from sensor_msgs.msg import LaserScan

def callback(msg):
    rospy.log("starting callback")
    #print len(msg.ranges)
    range_ahead = msg.ranges[len(msg.ranges)/2]
    print(range_ahead)


rospy.init_node('scan_values')
sub = rospy.Subscriber('/scan',LaserScan,callback)

rospy.spin()
