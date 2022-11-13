#!/usr/bin/env python
import sys
from math import tan, degrees, radians, pi, cos, sin

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg._Quaternion import Quaternion

CAR_WIDTH = rospy.get_param("f1tenth_simulator/width", 0.0)
BARRIER_WIDTH = CAR_WIDTH*4

class reactive_follow_gap:
    MAX_VELOCITY = 4 # Desired maximum velocity in meters per second
    GAP_DISTANCE = 1.5 # Distance in meters between consecutive lidar beams to consider there is an edge here
    MAX_DISTANCE = 40 # The maximum possible distance in the map, greater is an error
    MAX_ROT = pi/2 #the maximum rotation allowed to avoid going backward
    n=0
    def __init__(self):
        #Topics & Subscriptions,Publishers
        lidarscan_topic = '/scan'
        drive_topic = '/nav'
        self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback)
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=10)
        self.last_callback = rospy.get_time()
        self.markerPub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.callback_odom)
    
    def preprocess_lidar(self, data):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """
        self.start_i = int((-self.MAX_ROT-data.angle_min)/data.angle_increment)
        self.stop_i = int((self.MAX_ROT-data.angle_min)/data.angle_increment)
        jump_to = 0
        ranges = list(data.ranges)
        for k in range(self.start_i, self.stop_i):
            angle_start = data.angle_min + data.angle_increment * k
            if ranges[k]>self.MAX_DISTANCE:
                # Erreur ou sortie de carte
                ranges[k] = 0
                # print("beam out of the map at angle "+str(int(degrees(angle_start))))
            elif k>=jump_to and abs(ranges[k+1]-ranges[k])>self.GAP_DISTANCE:
                if ranges[k+1]-ranges[k]>0:
                    # Gap on the right
                    edge_sign = 1
                    start = k
                else:
                    edge_sign=  -1
                    start = k+1
                # print(ranges[k], ranges[k+1])
                jump_to = self.avoid_edge(ranges, start, edge_sign, data.angle_increment) # Last index of the barrier created
                # if edge_sign:
                #     print(ranges[start: jump_to])
                # else:
                #     print(ranges[jump_to: start])
                angle_stop = data.angle_min + data.angle_increment * jump_to
                # print("jump from angle "+str(int(degrees(angle_start)))+"deg to angle "+str(int(degrees(angle_stop)))+"deg")
        return ranges

    def avoid_edge(self, ranges, start, edge_sign, angle_increment):
        """Modifies ranges to create a barrier to be avoided
        """
        dist = ranges[start]
        width_increment = dist * tan(angle_increment)
        if width_increment==0:
            index_increment = edge_sign
        else:
            index_increment = int(BARRIER_WIDTH/width_increment)*edge_sign
        stop  = start+index_increment
        if edge_sign<0:
            stop = max(-1, stop)
        else:
            stop = min(len(ranges), stop)
        for i in range(start, stop, edge_sign):
            ranges[i] = dist
        return stop
        
    def find_max_gap(self, ranges, data):
        """ Return the start index & end index of the max gap in ranges
        It will look for the deepest gap with a width of at least the car
        """
        return self.start_i, self.stop_i
    
    def find_best_angle(self, ranges, data):
        """
        Return index of best angle
	Naive: Choose the furthest point within ranges and go there
        """
        start, stop = self.find_max_gap(ranges, data)
        best_index = start
        for k in range(start, stop):
            if ranges[k]>ranges[best_index]:
                best_index = k
        angle_to_drive = data.angle_min + data.angle_increment*best_index
        x = self.x + ranges[best_index]*cos(angle_to_drive+self.angle)
        y = self.y + ranges[best_index]*sin(angle_to_drive+self.angle)
        self.create_marker(x, y)
        return angle_to_drive

    def create_marker(self, x, y):
        marker = Marker()
        marker.header.frame_id = "/map"
        marker.header.stamp = rospy.Time()
        marker.ns = "my_namespace"
        marker.type = Marker.CUBE
        marker.id = 0
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.orientation = Quaternion()
        self.markerPub.publish(marker)

    def publish_drive_msg(self, angle):        
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
        self.drive_pub.publish(drive_msg)

    def callback_odom(self, data):
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        o = data.pose.pose.orientation
        self.angle = euler_from_quaternion([o.x, o.y, o.z, o.w])[2]

    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        if self.n==0:
            self.n=0
            proc_ranges = self.preprocess_lidar(data)
            desired_angle = self.find_best_angle(proc_ranges, data)
            self.publish_drive_msg(desired_angle)
            print("steering to "+str(round(degrees(desired_angle), 5))+"\n")
        else:
            self.n+=1

def main(args):
    rospy.init_node("reactive_gap_follow", anonymous=True)
    reactive_follow_gap()
    rospy.sleep(0.1)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
