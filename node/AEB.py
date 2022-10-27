#!/usr/bin/env python
from time import time
import roslib
roslib.load_manifest('f1tenth_simulator')
import rospy
from math import cos
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

class AEB:
    def __init__(self):
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.callback_odom)
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.callback_scan)
        self.brake_pub = rospy.Publisher("/brake", AckermannDriveStamped, queue_size=10)
        self.brake_bool_pub = rospy.Publisher("/brake_bool", Bool, queue_size=10)
        self.velocity=0
        
    def callback_scan(self, data):
        scanner = data.ranges
    
        """ # we want 60 degrees ahead of the car to avoid collision i.e. pi/3 rad
        start = int(5*pi/6/data.angle_increment)
        stop = int(start + pi/3/data.angle_increment)
        self.medium_distance=0
        for i in range(start, stop):
            self.medium_distance+=scanner[i]
        self.medium_distance/=stop-start 
        self.breaking()"""
        
                
        # to avoid noise we agregate the data of 10 consecutive lidar points to make a 0.05817764173314432 rad angle for each
        # we take into account the angle that will mofidiy the speed
        self.agregate = []
        n = len(scanner)
        for i in range(0, n-10, 5):
            angle = data.angle_min + data.angle_increment*(i+5)
            dist_agreg = sum(scanner[i:i+10])/10
            self.agregate.append(dist_agreg/cos(angle)) 
        self.breaking()
    
    def callback_odom(self, data):
        self.velocity = data.twist.twist.linear.x
    
    def breaking(self):
        # print(str(self.agregate))
        print(self.velocity)
        if self.velocity==0:
            return
        for medium_distance in self.agregate:
            timeToCollision = medium_distance/self.velocity
            if (timeToCollision>0 and timeToCollision<0.4) or abs(medium_distance)<0.25:
                self.emergency_stop()
                break
        else:
            self.brake_bool_pub.publish(Bool(False))
        # print(timeToCollision, self.velocity, medium_distance)
    
    def emergency_stop(self):
        msg = AckermannDriveStamped()
        msg.drive.speed = 0
        self.brake_pub.publish(msg)
        self.brake_bool_pub.publish(Bool(True))
        print("freinage d'urgence effectue")


def main():
    rospy.init_node("AEB", anonymous=True)
    AEB()
    rospy.spin()

if __name__=='__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

