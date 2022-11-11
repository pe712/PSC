#!/usr/bin/env python
import roslib
roslib.load_manifest('f1tenth_simulator')
import sys
from math import pi, atan2, sin, cos, radians
import numpy as np

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive


#WALL FOLLOW PARAMS
ANGLE_RANGE = 270 # Hokuyo 10LX has 270 degrees scan
DESIRED_DISTANCE_RIGHT = 0.9 # meters
DESIRED_DISTANCE_LEFT = 0.55
VELOCITY = 1.5 # meters per second
CAR_LENGTH = 0.50 # Traxxas Rally is 20 inches or 0.5 meters

class WallFollow:
    #PID CONTROL PARAMS
    ku = 4
    tu = 1.2 # sec
    kp = 3.2
    ki = 0.0 # 0.005
    kd = 0.004
    """ 
    kp = 5
    kd = 0.09
    ki = 0.01
    """
    average_delta_callback = 0.005
    prev_error = 0.0 
    integral = 0.0
    velocity = 0.0
    """ Implement Wall Following on the car
    """
    def __init__(self):
        #Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/nav'
        self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback)
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=10)
        
        self.last_callback = rospy.get_time()

    def getRange(self, data, angle):
        k = int((angle-data.angle_min)/data.angle_increment)
        return data.ranges[k]

    def pid_control(self, error):
        if abs(self.integral)<100:
            self.integral+=error
        delta = rospy.get_time() - self.last_callback
        derivative = (error - self.prev_error)/delta
        self.last_callback = rospy.get_time()
        self.prev_error = error
        angle = self.kp * error + self.ki*self.integral + self.kd*derivative
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = angle
        if (abs(angle)<radians(20)):
            self.velocity = VELOCITY
        elif (abs(angle)<radians(30)):
            self.velocity = VELOCITY / 1.5
        else:
            self.velocity = VELOCITY /3
        drive_msg.drive.speed = self.velocity
        # print("derivative: "+str(derivative) + " integral "+str(self.integral) + " angle: " + str(angle) +" delta "+str(delta))
        # print("D: "+str(self.kd*derivative) + " I "+str(self.ki*self.integral) + " P: " + str(self.kp * error))
        self.drive_pub.publish(drive_msg)

    def followLeft(self, data):
        #Follow left wall as per the algorithm
        # We want the measures at -pi/8 and -pi/2
        teta0 = -pi/2
        teta = pi/3
        a = self.getRange(data, teta0+teta)
        b = self.getRange(data, teta0)
        alpha = atan2((a*cos(teta)-b), (a*sin(teta)))
        D_t = b*cos(alpha)
        L = VELOCITY*self.average_delta_callback
        D_t1 = D_t + L*sin(alpha)
        error = DESIRED_DISTANCE_RIGHT - D_t1
        # print("a: "+str(a)+" b: "+str(b)+" D_t:"+str(D_t) + " error: "+str(error))
        return error
        
    def lidar_callback(self, data):
        error = self.followLeft(data)
        #send error to pid_control
        self.pid_control(error)

def main(args):
    rospy.init_node("wall_following", anonymous=True)
    WallFollow()
    rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)