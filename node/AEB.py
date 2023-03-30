#!/usr/bin/env python
from time import time
import roslib
roslib.load_manifest('f1tenth_simulator')
import rospy
from math import cos
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String

from switching_params import topics
from switcher import SIMULATION

"""Automatic emergency brake (AEB)
This class is intended to stop the car when it is too close to an obstacle.
It relies on the computation of the time to collision (TTC) in every direction to decide wether to stop.
Depending on the speed of the car and the environnement you should modify parameters MINIMUM_TTC (minimum time to collasion allowed) and MINIMUM_DISTANCE (minimum distance to the closest obstacle)
"""
class AEB:
    MINIMUM_TTC = 0.22 # in sec
    MINIMUM_DISTANCE = 0.2 # in meters
    def __init__(self):
        self.odom_sub = rospy.Subscriber(topics.ODOMETRY, Odometry, self.callback_odom)
        self.scan_sub = rospy.Subscriber(topics.LIDARSCAN, LaserScan, self.callback_scan)
        self.brake_pub = rospy.Publisher(topics.SAFETY, AckermannDriveStamped, queue_size=10)
        if SIMULATION:
            self.brake_bool_pub = rospy.Publisher(topics.SAFETY_BOOL, Bool, queue_size=10)
        else:
            self.brake_bool_pub = rospy.Publisher(topics.SAFETY_BOOL, String, queue_size=10)
        self.velocity=0

    def callback_scan(self, data):
        scanner = data.ranges
        # we take into account the angle that will mofidiy the speed
        n = len(scanner)
        directionned_scanner = []
        for i, distance in enumerate(scanner):
            angle = data.angle_min + data.angle_increment*i
            directionned_scanner.append(distance/cos(angle)) 
        self.breaking(directionned_scanner)

    def callback_odom(self, data):
        self.velocity = data.twist.twist.linear.x

    def breaking(self, directionned_scanner):
        # print(str(directionned_scanner))
        # print(self.velocity)
        # We need to keep in memory the velocity because it can change during calculation
        speed = self.velocity
        # print(speed)
        if speed==0:
            return
        for distance in directionned_scanner:
            timeToCollision = distance/speed
            if (timeToCollision>0 and timeToCollision<self.MINIMUM_TTC) or abs(distance)<self.MINIMUM_DISTANCE:
                if (abs(distance)<self.MINIMUM_DISTANCE):
                    print("too close")
                if (timeToCollision>0 and timeToCollision<self.MINIMUM_TTC):
                    print("too fast ", timeToCollision, "s to collision")
                self.emergency_stop()
                break
        else:
            if SIMULATION:
                self.brake_bool_pub.publish(Bool(False))
        # print(timeToCollision, self.velocity)

    def emergency_stop(self):
        msg = AckermannDriveStamped()
        msg.drive.speed = 0
        self.brake_pub.publish(msg)
        if SIMULATION:
            self.brake_bool_pub.publish(Bool(True))
        else:
            self.brake_bool_pub.publish(String("Safety"))
        print("freinage d'urgence effectue\n\n\n")


def main():
    rospy.init_node("AEB", anonymous=True)
    AEB()
    rospy.spin()

if __name__=='__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

