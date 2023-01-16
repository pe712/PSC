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
        if self.velocity==0:
            return
        for distance in directionned_scanner:
            timeToCollision = distance/self.velocity
            if (timeToCollision>0 and timeToCollision<0.3) or abs(distance)<0.30:
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

