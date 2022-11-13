#!/usr/bin/env python
import sys
from math import tan, degrees, radians

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

VELOCITY = 1.5 # desired maximum velocity in meters per second
CAR_WIDTH = rospy.get_param("f1tenth_simulator/width", 0.0)
BARRIER_WIDTH = 0.5 * CAR_WIDTH

class reactive_follow_gap:
    GAP_DISTANCE = 1.5
    def __init__(self):
        #Topics & Subscriptions,Publishers
        lidarscan_topic = '/scan'
        drive_topic = '/nav'
        self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback)
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=10)
    
    def preprocess_lidar(self, data):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """
        jump_to = 0
        ranges = list(data.ranges)
        for k in range(len(ranges)-1):
            if k>=jump_to and abs(ranges[k+1]-ranges[k])>self.GAP_DISTANCE:
                if ranges[k+1]-ranges[k]>0:
                    # Gap on the right
                    gap_sign = 1
                    start = k
                else:
                    gap_sign=  -1
                    start = k+1
                jump_to = self.avoid_gap(ranges, start, gap_sign, data.angle_increment) # Last index of the barrier created
                angle_start = data.angle_min + data.angle_increment * start
                angle_stop = data.angle_min + data.angle_increment * jump_to
                print("jump from angle "+str(int(degrees(angle_start)))+"deg to angle "+str(int(degrees(angle_stop)))+"deg")
        print("\n")
        return ranges

    def avoid_gap(self, ranges, start, gap_sign, angle_increment):
        """Modifies ranges to create a barrier to be avoided
        """
        dist = ranges[start]
        width_increment = dist * tan(angle_increment)
        if width_increment==0:
            index_increment = gap_sign
        else:
            index_increment = int(BARRIER_WIDTH/width_increment)*gap_sign
        stop  = start+index_increment
        if gap_sign<0:
            stop = max(-1, stop)
        else:
            stop = min(len(ranges), stop)
        for i in range(start, stop, gap_sign):
            ranges[i] = dist
        return stop
        
    def find_max_gap(self, free_space_ranges):
        """ Return the start index & end index of the max gap in free_space_ranges
        """
        return start_i, end_i
    
    def find_best_angle(self, ranges, data):
        """
        Return index of best angle
	Naive: Choose the furthest point within ranges and go there
        """
        start_i, end_i = self.find_max_gap(ranges)
        return None

    def publish_drive_msg(self, angle):        
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
        self.drive_pub.publish(drive_msg)

    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        proc_ranges = self.preprocess_lidar(data)
        desired_angle = self.find_best_angle(proc_ranges, data)
        self.publish_drive_msg(desired_angle)

def main(args):
    rospy.init_node("reactive_gap_follow", anonymous=True)
    reactive_follow_gap()
    rospy.sleep(0.1)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
