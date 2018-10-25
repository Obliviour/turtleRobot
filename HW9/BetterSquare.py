import time
import roslib
import rospy
from kobuki_msgs.msg import ButtonEvent
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
import cmath
import math

class BetterSquare():
    def __init__(self):
        # initiliaze
        rospy.init_node('BetterSquare', anonymous=False)
        # rospy.init_node('CheckOdometry', anonymous=False)
        # we want a straight line with a phase of straight
        odom = rospy.Subscriber('odom', Odometry, self.OdometryCallBack)
        # tell user how to stop TurtleBot
        rospy.loginfo("To stop TurtleBot CTRL + C")
        rospy.on_shutdown(self.shutdown)

        self.isFirstRun = 1
        self.enableDrive = 0
        self.enableRotate = 0
        self.objective_angle = 0

        rotateTime = 1.5
        driveTime = 3

        # # as long as you haven't ctrl + c keeping doing...
        # while not rospy.is_shutdown():
            # # publish the velocity
            # self.cmd_vel.publish(move_cmd)
            # # check for phase err
            # self.adjustPhase()
            # # wait for 0.1 seconds (10 HZ) and publish again
            # r.sleep()
        
        #rospy.spin() tells the program to not exit until you press ctrl + c.  If this wasn't there... it'd subscribe and then immediatly exit (therefore stop "listening" to the thread).
    	#rospy.spin()
	
	curr_state = 0

        while not rospy.is_shutdown():
            #print(curr_state)
	    if(curr_state == 0): #Rotate to Angle
                t0 = time.time()
                self.enableRotate = 1
                next_state = 1
            elif(curr_state == 1): #Count time
                t1 = time.time()
                if((t1-t0)>=rotateTime):
                    self.enableRotate = 0
                    self.objective_angle = (self.objective_angle + math.pi/2) % (2*math.pi)
                    #self.objective_angle = self.objective_angle * math.pi / 180
                    print(self.objective_angle)
		    next_state = 2
                else:
                    next_state = 1
            elif(curr_state == 2): #Start Drive
                t0 = time.time()
                self.enableDrive = 1
                next_state = 3
            elif(curr_state == 3): #Count Time
                t1 = time.time()
                if((t1-t0)>=driveTime):
                    self.enableDrive = 0
                    self.isFirstRun = 1
                    next_state = 0
                else:
                    next_state = 3
            curr_state = next_state

    def shutdown(self):
        # stop turtlebot
        rospy.loginfo("Stop TurtleBot")
        # a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop TurtleBot
        self.cmd_vel.publish(Twist())
        # sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
        rospy.sleep(1)

        
    def OdometryCallBack(self, msg):
        self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
        #self.r = rospy.Rate(20)
        if(self.enableDrive):
            current_orientation = msg.pose.pose.orientation
            #print(current_orientation.y)
            #print(current_orientation.x)
            #print(current_orientation.z)
            w = current_orientation.w
            z = current_orientation.z
            angle = 2*math.atan2(z,w)
            zcur = cmath.rect(1,angle)
            if (self.isFirstRun):
            	self.zdes = cmath.rect(1, angle)
            	self.isFirstRun = 0 
            zerr = self.zdes/zcur
            phase_err = cmath.phase(zerr)
            w = self.adjustPhase(phase_err)
            #rospy.loginfo("destination angle: %f current angle: %f error: %f adjusted phase: %f"%(cmath.phase(self.zdes),angle,phase_err,w))
            # Twist is a datatype for velocity
            error_cmd = Twist()
            error_cmd.linear.x = 0.2
            error_cmd.angular.z = w
            
            self.cmd_vel.publish(error_cmd)
        elif(self.enableRotate):
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
            #rospy.loginfo("destination angle: %f current angle: %f error: %f adjusted phase: %f"%(cmath.phase(self.zdes),angle,phase_err,w))
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
        else:
            stay_put = Twist()
            self.cmd_vel.publish(stay_put)
        
        
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
        
        

 
if __name__ == '__main__':
    #try:
    BetterSquare()
    #except:
    #    rospy.loginfo("GoForward node terminated.")

