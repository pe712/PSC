#!/usr/bin/env python
import sys
from math import tan, degrees, radians, pi, cos, sin, sqrt

#ROS Imports
import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from utils import simple_markers
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose
from switcher import SIMULATION

from switching_params import topics

CAR_WIDTH = rospy.get_param("f1tenth_simulator/width", 0.0)
if CAR_WIDTH == 0:
    CAR_WIDTH = 0.2
BARRIER_WIDTH = CAR_WIDTH*2

class reactive_follow_gap:
    MAX_VELOCITY = 3 # Desired maximum velocity in meters per second
    TURN_VELOCITY = 1.3
    GAP_DISTANCE = 1.5 # Distance in meters between consecutive lidar beams to consider there is an edge here
    MAX_DISTANCE = 40 # The maximum possible distance in the map, greater is an error
    MAX_ROT = pi/3 #the maximum rotation allowed to avoid going backward
    DIST_FROM_LIDAR = rospy.get_param("f1tenth_simulator/scan_distance_to_base_link", 0.0)
    n=0
    def __init__(self):
        #Topics & Subscriptions,Publishers
        self.lidar_sub = rospy.Subscriber(topics.LIDARSCAN, LaserScan, self.lidar_callback)
        self.drive_pub = rospy.Publisher(topics.DRIVE, AckermannDriveStamped, queue_size=10)
        self.last_callback = rospy.get_time()
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.callback_odom)
        if SIMULATION:
            self.processed_ranges = rospy.Publisher("/processedRanges", LaserScan, queue_size=100)
            self.targetPointPub = rospy.Publisher('/targetPoint', Marker, queue_size=40)
            self.edgePub = rospy.Publisher('/edges_array', MarkerArray, queue_size=40)
        """ Should use this instead of doing tf manually
        self.tfBuffer = Buffer()
        self.tf_listener = TransformListener(self.tfBuffer) 
        """

    def preprocess_lidar(self, data):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """
        # those are for heading to (finding the target point)
        self.start_i = int((-self.MAX_ROT-data.angle_min)/data.angle_increment)
        self.stop_i = int((self.MAX_ROT-data.angle_min)/data.angle_increment)
        # those are for avoiding edged
        start_avoiding = int((-self.MAX_ROT*2-data.angle_min)/data.angle_increment)
        stop_avoiding = int((self.MAX_ROT*2-data.angle_min)/data.angle_increment)
        ranges = list(data.ranges)
        edges_to_avoid = []
        for k in range(start_avoiding, stop_avoiding):
            angle_start = data.angle_min + data.angle_increment * k
            if ranges[k]>self.MAX_DISTANCE:
                # Erreur ou sortie de carte
                ranges[k] = 0
                print("beam out of the map at angle "+str(int(degrees(angle_start))))
            elif abs(ranges[k+1]-ranges[k])>self.GAP_DISTANCE:
                if ranges[k+1]-ranges[k]>0:
                    # Gap on the right
                    edge_sign = 1
                    start = k
                    # print(ranges[start: jump_to])
                else:
                    edge_sign = -1
                    start = k+1
                    # print(ranges[jump_to: start])
                edges_to_avoid.append((start, edge_sign, angle_start, ranges[start]))
        self.avoid_edge(ranges, edges_to_avoid, data)
        return ranges

    def avoid_edge(self, ranges, edges_to_avoid, data):
        """Modifies ranges to create a barrier to be avoided
        """
        X_from, Y_from, X_to, Y_to = [], [], [], []
        n = len(edges_to_avoid)
        for count, (start, edge_sign, angle_start, dist) in enumerate(edges_to_avoid):
            width_increment = dist * tan(data.angle_increment)
            if width_increment==0:
                index_increment = edge_sign # au moins 1
            else:
                index_increment = int(BARRIER_WIDTH/width_increment)*edge_sign
            stop  = start+index_increment
            if edge_sign<0:
                stop = max(-1, stop)
            else:
                stop = min(len(ranges), stop)
            for i in range(start, stop, edge_sign):
                # if another barrier is in front of this one, this one is not needed
                ranges[i] = min(dist, ranges[i])
            angle_stop = data.angle_min + data.angle_increment * stop
            if SIMULATION and hasattr(self, 'x'):
                X_from.append(self.x + dist*cos(angle_start+self.angle) + self.DIST_FROM_LIDAR*cos(self.angle))
                Y_from.append(self.y + dist*sin(angle_start+self.angle) + self.DIST_FROM_LIDAR*sin(self.angle))
                X_to.append(self.x + dist*cos(angle_stop+self.angle) + self.DIST_FROM_LIDAR*cos(self.angle))
                Y_to.append(self.y + dist*sin(angle_stop+self.angle) + self.DIST_FROM_LIDAR*sin(self.angle))
            # print("jump from angle "+str(int(degrees(angle_start)))+"deg to angle "+str(int(degrees(angle_stop)))+"deg")
        if SIMULATION:
            simple_markers.create_arrow(X_from, Y_from, X_to, Y_to, self.edgePub)

    def find_max_gap(self, ranges, data):
        """ Return the start index & end index of the max gap in ranges
        It will look for the deepest gap with a width of at least the car
        """
        return self.start_i, self.stop_i

    def find_best_angle(self, ranges, data):
        """ Return index of best angle
	    Naive: Choose the furthest point within ranges and go there
        """
        start, stop = self.find_max_gap(ranges, data)
        best_index = start
        for k in range(start, stop):
            if ranges[k]>ranges[best_index]:
                best_index = k
        # best_index, dist = max(enumerate(ranges[start:stop]), key=lambda x: x[1])
        # print(best_index, ranges[best_index], start, stop)
        angle_to_drive = data.angle_min + data.angle_increment*best_index
        if SIMULATION:
            msg = LaserScan()
            msg.ranges = ranges
            self.processed_ranges.publish(msg)
            if hasattr(self, 'x'):
                x = self.x + ranges[best_index]*cos(angle_to_drive+self.angle) + self.DIST_FROM_LIDAR*cos(self.angle)
                y = self.y + ranges[best_index]*sin(angle_to_drive+self.angle) + self.DIST_FROM_LIDAR*sin(self.angle)
                simple_markers.create_marker(x, y, self.targetPointPub)
        return angle_to_drive, ranges[best_index]

    def publish_drive_msg(self, angle, dist):
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = angle
        if (abs(angle)<radians(20)):
            self.velocity = self.MAX_VELOCITY
        elif (abs(angle)>radians(30)):
            self.velocity = self.TURN_VELOCITY
        else:
            # angle is between 20 and 30, affine function
            factor_angle = 1 + (abs(angle)-radians(20))/(radians(30)-radians(20))*(-1+self.TURN_VELOCITY/self.MAX_VELOCITY)
            self.velocity = self.MAX_VELOCITY * factor_angle
        factor_dist = sqrt(dist)/(sqrt(dist)+1)
        self.velocity *=factor_dist
        print("max velocity = "+str(self.MAX_VELOCITY)+" distance forward = "+str(dist)+" current velocity cmd = "+str(self.velocity))
        print("factor dist = "+str(factor_dist))
        drive_msg.drive.speed = self.velocity
        self.drive_pub.publish(drive_msg)

    def callback_odom(self, data):
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        o = data.pose.pose.orientation
        self.angle = euler_from_quaternion([o.x, o.y, o.z, o.w])[2]

        """ transform = self.tfBuffer.lookup_transform("map", data.header.frame_id, data.header.stamp, rospy.Duration(1))
        pose_transformed = do_transform_pose(data.pose, transform)
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        o = data.pose.pose.orientation
        self.angle = euler_from_quaternion([o.x, o.y, o.z, o.w])[2] """

    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        if self.n==0:
            self.n=0
            proc_ranges = self.preprocess_lidar(data)
            desired_angle, dist = self.find_best_angle(proc_ranges, data)
            self.publish_drive_msg(desired_angle, dist)
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
