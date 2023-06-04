#!/usr/bin/env python
import roslib
roslib.load_manifest('f1tenth_simulator')
import sys
from math import pi, atan2, sin, cos, radians

from switching_params import topics

#ROS Imports
import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive


#WALL FOLLOW PARAMS

class WallFollow:
    """
    Implements Wall Following on the car.

    Attributes:
    DESIRED_DISTANCE_RIGHT (float): desired distance from the right wall in meters
    DESIRED_DISTANCE_LEFT (float): desired distance from the left wall in meters
    MAX_VELOCITY (float): desired maximum velocity in meters per second
    KU (float): ultimate gain for tuning the PID controller
    TU (float): ultimate period for tuning the PID controller in seconds
    KP (float): proportional gain for the PID controller
    KI (float): integral gain for the PID controller
    KD (float): derivative gain for the PID controller
    average_delta_callback (float): average delta time between lidar callbacks
    prev_error (float): previous error for the PID controller
    integral (float): integral term for the PID controller
    velocity (float): current velocity of the car
    """

    DESIRED_DISTANCE_RIGHT = 0.9 # meters
    DESIRED_DISTANCE_LEFT = 0.55
    MAX_VELOCITY = 1.5 # desired maximum velocity in meters per second
    #PID CONTROL PARAMS
    KU = 4
    TU = 1.2 # sec
    KP = 3.2
    KI = 0.0
    KD = 0.004
    average_delta_callback = 0.005
    prev_error = 0.0 
    integral = 0.0
    velocity = 0.0
    """ Implement Wall Following on the car
    """
    def __init__(self):
        #Topics & Subs, Pubs
        self.lidar_sub = rospy.Subscriber(topics.LIDARSCAN, LaserScan, self.lidar_callback)
        self.drive_pub = rospy.Publisher(topics.DRIVE, AckermannDriveStamped, queue_size=10)
        self.last_callback = rospy.get_time()

    def getRange(self, data, angle):
        """
        Retrieves the range measurement at a specific angle from the LaserScan data.

        Args:
            data (LaserScan): LaserScan data
            angle (float): angle in radians

        Returns:
            float: range measurement at the specified angle
        """
        k = int((angle-data.angle_min)/data.angle_increment)
        return data.ranges[k]

    def pid_control(self, error):
        """
        Performs PID control to calculate the steering angle.

        Args:
            error (float): error value

        Returns:
            float: calculated steering angle
        """
        # avoid integral to scale infinitely
        if abs(self.integral)<100:
            self.integral+=error
        delta = rospy.get_time() - self.last_callback
        derivative = (error - self.prev_error)/delta
        self.last_callback = rospy.get_time()
        self.prev_error = error
        angle = self.KP * error + self.KI*self.integral + self.KD*derivative
        return angle

    def publish_drive_msg(self, angle):
        """
        Publishes the drive message with the calculated steering angle and velocity.

        Args:
            angle (float): steering angle
        """
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = angle
        if (abs(angle)<radians(20)):
            self.velocity = self.MAX_VELOCITY
        elif (abs(angle)<radians(30)):
            self.velocity = self.MAX_VELOCITY / 1.5
        else:
            self.velocity = self.MAX_VELOCITY /3
        drive_msg.drive.speed = self.velocity
        # print("derivative: "+str(derivative) + " integral "+str(self.integral) + " angle: " + str(angle) +" delta "+str(delta))
        # print("D: "+str(self.KD*derivative) + " I "+str(self.KI*self.integral) + " P: " + str(self.KP * error))
        self.drive_pub.publish(drive_msg)

    def followLeft(self, data):
        """
        Implements the wall-following algorithm by following the left wall. 
        
        Go to https://f1tenth.org/learn.html and do some gometry to get to those results.

        Args:
            data (LaserScan): LaserScan data

        Returns:
            float: error value for the PID controller
        """
        teta0 = -pi/2
        teta = pi/3
        a = self.getRange(data, teta0+teta)
        b = self.getRange(data, teta0)
        alpha = atan2((a*cos(teta)-b), (a*sin(teta)))
        D_t = b*cos(alpha)
        L = self.MAX_VELOCITY*self.average_delta_callback
        D_t1 = D_t + L*sin(alpha)
        error = self.DESIRED_DISTANCE_RIGHT - D_t1
        # print("a: "+str(a)+" b: "+str(b)+" D_t:"+str(D_t) + " error: "+str(error))
        return error

    def lidar_callback(self, data):
        error = self.followLeft(data)
        # send error to pid_control
        angle_to_drive= self.pid_control(error)
        self.publish_drive_msg(angle_to_drive)

def main(args):
    rospy.init_node("wall_following", anonymous=True)
    WallFollow()
    rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)