#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from kobuki_msgs.msg import BumperEvent
from kobuki_msgs.msg import WheelDropEvent
from kobuki_msgs.msg import CliffEvent


class Scan_msg:

    
    def __init__(self):
        '''Initializes an object of this class.

        The constructor creates a publisher, a twist message.
        3 integer variables are created to keep track of where obstacles exist.
        3 dictionaries are to keep track of the movement and log messages.'''
        self.pub = rospy.Publisher('/cmd_vel_mux/input/navi',Twist)
        self.msg = Twist()
        self.sect_1 = 0
        self.sect_2 = 0
        self.sect_3 = 0
        self.sect_4 = 0
        self.sect_5 = 0
        keys = [b0+b1+b2+b3+b4 for b0 in range(0,2,1) for b1 in range(0, 11, 10) for b2 in range(0, 101, 100) for b3 in range(0, 1001, 1000) for b4 in range(0, 10001, 10000)]
        ang_increment = 1.7/len(keys)/2
        foward_increment = 1/len(keys)/2
        ang_vals = [i * ang_increment for i in range(len(keys)/2)] + [-i * ang_increment for i in range(len(keys)/2)]
        fwd_vals = [i * forward_increment for i in range(len(keys)/2)] + [-i * forward_increment for i in range(len(keys)/2)]  
        for i in range(len(keys)):
            key = keys[i]
            self.ang(key) = ang_vals[i]
            self.fwd(key) = fwd_vals[i]
        
        for key, val in ang_vals:
            if val == 0:
                self.dbgmsg(key) = 'Move forward'
            elif val < 0:
                self.dbgmsg(key) = 'Veer Right'
            elif val > 0:
                self.dbgmsg(key) = 'Veer Left'
        # self.ang = {0:0,1:-1.2,10:-1.2,11:-1.2,100:1.5,101:1.0,110:1.0,111:1.2, 1000:-.5,1001:.5,}
        # self.fwd = {0:.25,1:0,10:0,11:0,100:0.1,101:0,110:0,111:0}
        # self.dbgmsg = {0:'Move forward',1:'Veer right',10:'Veer right',11:'Veer right',100:'Veer left',101:'Veer left',110:'Veer left',111:'Veer right'}
        # print(keys + ":keys")
        # subscriber callbacks
        rospy.Subscriber("/mobile_base/events/bumper", BumperEvent, self.bumper_callback)
        rospy.Subscriber("/mobile_base/events/wheel_drop", WheelDropEvent, self.wheel_callback)
        rospy.Subscriber("/mobile_base/events/cliff",CliffEvent, self.cliff_callback)
        
        self.bhit = 0
        self.whit = 0
        self.chit = 0

    def bumper_callback(self,data):
        if ( data.state == BumperEvent.PRESSED ) :
            self.bhit = 1
        elif (data.state == BumperEvent.RELEASED):
            self.bhit =0
        else:
            self.bhit = 0

    def wheel_callback(self,data):
        if (data.state == WheelDropEvent.RAISED):
            self.whit = 0
        elif (data.state == WheelDropEvent.DROPPED):
            self.whit = 1
        else:
            self.whit = 0

    def cliff_callback(self, data):
        if (data.state == CliffEvent.CLIFF):
            self.chit = 1
        elif (data.state == CliffEvent.FLOOR):
            self.chit = 0
        else:
            self.chit = 0
        
    def reset_sect(self):
        '''Resets the below variables before each new scan message is read'''
        self.sect_1 = 0
        self.sect_2 = 0
        self.sect_3 = 0
        self.sect_4 = 0
        self.sect_5 = 0
        

    def sort(self, laserscan):
        '''Goes through 'ranges' array in laserscan message and determines 
        where obstacles are located. The class variables sect_1, sect_2, 
        and sect_3 are updated as either '0' (no obstacles within 0.7 m)
        or '1' (obstacles within 0.7 m)

        Parameter laserscan is a laserscan message.'''
        entries = len(laserscan.ranges)
        for entry in range(0,entries):
            if 0.4 < laserscan.ranges[entry] < 0.75:
                self.sect_1 = 1 if (0 < entry < entries/5) else 0 
                self.sect_2 = 1 if (entries/5 < entry < 2*entries/5) else 0
                self.sect_3 = 1 if (2*entries/5 < entry < 3*entries/5) else 0
                self.sect_4 = 1 if (3*entries/5 < entry < 4*entries/5) else 0
                self.sect_5 = 1 if (4*entries/5 < entry < entries) else 0
        info_string = """sort complete,sect_1: " + {} + " sect_2: " + {} + " sect_3: " + {} + " sect_4: " + {} + " sect_5: " + {} """
        info_string = info_string.format(str(self.sect_1),str(self.sect_2),str(self.sect_3),str(self.sect_4),str(self.sect_5)) 
        rospy.loginfo(info_string)

    def movement(self, sect1, sect2, sect3, sect4, sect5):
        '''Uses the information known about the obstacles to move robot.

        Parameters are class variables and are used to assign a value to
        variable sect and then  set the appropriate angular and linear 
        velocities, and log messages.
        These are published and the sect variables are reset.'''
        sect = int(str(self.sect_1) + str(self.sect_2) + str(self.sect_3) + str(self.sect_4) + str(self.sect_5))
        rospy.loginfo("Sect = " + str(sect)) 
        
        self.msg.angular.z = self.ang[sect]
        self.msg.linear.x = self.fwd[sect]
        rospy.loginfo(self.dbgmsg[sect])
        self.pub.publish(self.msg)

        self.reset_sect()
 
    def for_callback(self,laserscan):
        '''Passes laserscan onto function sort which gives the sect 
        variables the proper values.  Then the movement function is run 
        with the class sect variables as parameters.

        Parameter laserscan is received from callback function.'''
        self.sort(laserscan)
        self.movement(self.sect_1, self.sect_2, self.sect_3, self.sect_4, self.sect_5)
        

def call_back(scanmsg):
    '''Passes laser scan message to for_callback function of sub_obj.

    Parameter scanmsg is laserscan message.'''
    sub_obj.for_callback(scanmsg)

def listener():
    '''Initializes node, creates subscriber, and states callback 
    function.'''
    rospy.init_node('navigation_sensors')
    rospy.loginfo("Subscriber Starting")
    sub = rospy.Subscriber('/scan', LaserScan, call_back)
    rospy.spin()

if __name__ == "__main__":
    '''A Scan_msg class object called sub_obj is created and listener
    function is run''' 
    sub_obj = Scan_msg()
    listener()
