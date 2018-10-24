#!/usr/bin/env python

# This script is inspired by https://github.com/yujinrobot/kobuki/blob/f99e495b2b3be1e62495119809c58ccb58909f67/kobuki_testsuite/scripts/test_events.py
# They deserve all the credit therefore I'm including their copyright notice / BSD - Mark Silliman

'''
# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Yujin Robot
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Yujin Robot nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
'''

# Monitor the kobuki's button status
import time
import roslib
import rospy
from kobuki_msgs.msg import ButtonEvent
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
import cmath
import math
from std_msgs.msg import Int16

class GoStraight():
    def __init__(self):
        # initiliaze
        rospy.init_node('GoStraight', anonymous=True)
        # rospy.init_node('CheckOdometry', anonymous=False)
        # we want a straight line with a phase of straight
        odom = rospy.Subscriber('odom', Odometry, self.OdometryCallBack)
        # tell user how to stop TurtleBot
        rospy.Subscriber("chatter",Int16,self.UpdateAngle)

    	self.isFirstRun = 1
        rospy.loginfo("To stop TurtleBot CTRL + C")

        self.objective_angle = 0

        

        # # as long as you haven't ctrl + c keeping doing...
        # while not rospy.is_shutdown():
            # # publish the velocity
            # self.cmd_vel.publish(move_cmd)
            # # check for phase err
            # self.adjustPhase()
            # # wait for 0.1 seconds (10 HZ) and publish again
            # r.sleep()
        
        #rospy.spin() tells the program to not exit until you press ctrl + c.  If this wasn't there... it'd subscribe and then immediatly exit (therefore stop "listening" to the thread).
    	rospy.spin()
        
    def OdometryCallBack(self, msg):
        #self.r = rospy.Rate(20)
        self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
        current_orientation = msg.pose.pose.orientation
        #print(current_orientation.y)
        #print(current_orientation.x)
        #print(current_orientation.z)
        w = current_orientation.w
        z = current_orientation.z
        angle = 2*math.atan2(z,w)
        zcur = cmath.rect(1,angle)
        #if (self.isFirstRun):
            # Set zdes to a certain value
        self.zdes = cmath.rect(1, self.objective_angle)
        #self.isFirstRun = 0 
        zerr = self.zdes/zcur
        phase_err = cmath.phase(zerr)
        w = self.adjustPhase(phase_err)
        rospy.loginfo("destination angle: %f current angle: %f error: %f adjusted phase: %f"%(cmath.phase(self.zdes),angle,phase_err,w))
        # Twist is a datatype for velocity
        range = 0
        if (cmath.phase(zerr) == range):
            error_cmd = Twist()
            error_cmd.linear.x = 0
            error_cmd.angular.z = 0
        else:
            error_cmd = Twist()
            error_cmd.linear.x = 0
            error_cmd.angular.z = w
        
        self.cmd_vel.publish(error_cmd)
        
        #self.r.sleep()
        
         # Twist is a datatype for velocity
        # move_cmd = Twist()
        # move_cmd.linear.x = 0.2
        # move_cmd.angular.z = 0
        
        # self.cmd_vel.publish(move_cmd)
        # time.sleep(2)
        
        
    def adjustPhase(self, phase_err):
        k_turn = 2.5
        w = phase_err * k_turn
        wmax = 3.14
        w = self.saturation(w, wmax)
        return w
        
    def saturation(self, w, wmax):
        if(w > wmax):
            w = wmax
        elif(w < -wmax):
            w = - wmax
        return w
        
    def UpdateAngle(self, msg):
        print(msg.data)
	    self.objective_angle = math.pi*int(msg.data)/180        

 
if __name__ == '__main__':
    #try:
    GoStraight()
    #except:
    #    rospy.loginfo("GoForward node terminated.")

