#!/usr/bin/env python

'''
Copyright (c) 2015, Mark Silliman
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
'''

# A very basic TurtleBot script that moves TurtleBot forward indefinitely. Press CTRL + C to stop.  To run:
# On TurtleBot:
# roslaunch turtlebot_bringup minimal.launch
# On work station:
# python goforward.py

import rospy
from kobuki_msgs.msg import BumperEvent
from kobuki_msgs.msg import WheelDropEvent
from geometry_msgs.msg import Twist
import time

bump_hit = 0
wheel = 0

class GoForward():
    def __init__(self):
        # initiliaze
        print('Start')
        rospy.init_node('GoForward', anonymous=False)

        # tell user how to stop TurtleBot
        rospy.loginfo("To stop TurtleBot CTRL + C")

        # What function to call when you ctrl + c    
        rospy.on_shutdown(self.shutdown)
        
        # Create a publisher which can "talk" to TurtleBot and tell it to move
        self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
           
        #Subscribe to Bumper Events
        rospy.Subscriber('mobile_base/events/bumper',BumperEvent,self.BumperEventCallback)

        #Subscribe to Wheel Drop Events
        rospy.Subscriber('mobile_base/events/wheel_drop',WheelDropEvent,self.WheelDropEventCallback)

        #TurtleBot will stop if we don't keep telling it to move.  How often should we tell it to move? 10 HZ
        r = rospy.Rate(10);

        # Twist is a datatype for velocity
        move_cmd = Twist()
        # let's go forward at 0.2 m/s
        move_cmd.linear.x = 0.2
        # let's turn at 0 radians/s
        move_cmd.angular.z = 0

        stop_cmd = Twist()

        curr_state = 0
        next_state = 0

        # as long as you haven't ctrl + c keeping doing...
        while not rospy.is_shutdown():
            if(curr_state == 0): #Go_Forward
                self.cmd_vel.publish(move_cmd)
                if(bump_hit == 1):
                    next_state = 1 #Go to Wait_For_Button
                elif(wheel == 1):
                    next_state = 2 #Go to Wait_For_Wheel
                else:
                    next_state = 0 #Stay in current state
            elif(curr_state == 1): #Wait_For_Button
                self.cmd_vel.publish(stop_cmd)
                if(bump_hit == 0):
                    t0 = time.time() #start timer
                    next_state = 3 #Go to Count_Time
                else:
                    next_state = 1 #Stay in current state
            elif(curr_state == 2): #Wait_For_Wheel
                self.cmd_vel.publish(stop_cmd)
                if(wheel == 0):
                    t0 = time.time() #start timer
                    next_state = 3 #Go to Count_Time 
                else:
                    next_state = 2 #Stay in current state
            elif(curr_state == 3): #Count_Time
                self.cmd_vel.publish(stop_cmd)
                t1 = time.time()
                if((t1 - t0) >= 2):
                    next_state = 0 #Go to Go_Forward
                elif(bump_hit == 1):
                    next_state = 1 #Go to Wait_For_Button
                elif(wheel == 1):
                    next_state = 2 #Go to Wait_For_Wheel
                else:
                    next_state = 3 #Stay in current state
            else:
                next_state = 0 #Something went wrong


            curr_state = next_state
            # wait for 0.1 seconds (10 HZ) and publish again
            r.sleep()
                        
        
    def shutdown(self):
        # stop turtlebot
        rospy.loginfo("Stop TurtleBot")
        # a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop TurtleBot
        self.cmd_vel.publish(Twist())
        # sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
        rospy.sleep(1)

    def WheelDropEventCallback(self,data):
        if(data.state == WheelDropEvent.RAISED):
            wheel = 1
        else:
            wheel = 0
 
    def BumperEventCallback(self,data):
        if(data.state == BumperEvent.PRESSED):
            bump_hit = 1
        else:
            bump_hit = 0

if __name__ == '__main__':
        GoForward()
