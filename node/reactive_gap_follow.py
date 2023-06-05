#!/usr/bin/env python
import sys
from math import tan, degrees, radians, pi, cos, sin, sqrt

#ROS Imports
import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from tf.transformations import euler_from_quaternion
from utils import simple_markers
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose
from switcher import SIMULATION
from speed import speed_computation

from switching_params import topics

CAR_WIDTH = rospy.get_param("f1tenth_simulator/width", 0.0)
if CAR_WIDTH == 0:
    CAR_WIDTH = 0.2
if SIMULATION:
    BARRIER_WIDTH = CAR_WIDTH*2.5
else:
    BARRIER_WIDTH = CAR_WIDTH*1.8

class reactive_follow_gap:
    """
    Reactive Follow Gap class that implements the reactive gap following algorithm.

    This class subscribes to the LiDAR scan data and performs the reactive gap following algorithm
    to find the best steering angle based on the available gaps in the LiDAR data. It publishes the
    corresponding drive command with the desired steering angle and speed.

    Attributes:
        MAX_VELOCITY (float): Desired maximum velocity in meters per second.
        TURN_VELOCITY (float): Velocity for turning in meters per second.
        GAP_DISTANCE (float): Distance in meters between consecutive LiDAR beams to consider there is an edge.
        MAX_DISTANCE (float): The maximum possible distance in the map, where greater is considered an error.
        MAX_ROT (float): The maximum rotation allowed to avoid going backward.
        DIST_FROM_LIDAR (float): Distance from the LiDAR sensor to the base link of the car.
        lidar_sub (rospy.Subscriber): Subscriber for the LiDAR scan topic.
        drive_pub (rospy.Publisher): Publisher for the drive command topic.
        odom_sub (rospy.Subscriber): Subscriber for the odometry topic.
        closest_obstacle_sub (rospy.Subscriber): Subscriber for the closest obstacle distance topic.
        speed (speed_computation): Object for calculating the speed based on the desired velocity.
        processed_ranges_pub (rospy.Publisher): Publisher for the processed LiDAR ranges topic (simulation only).
        target_point_pub (rospy.Publisher): Publisher for the target point visualization marker topic (simulation only).
        edgePub (rospy.Publisher): Publisher for the edge visualization markers topic (simulation only).
        x (float): X-coordinate of the car's position from the odometry.
        y (float): Y-coordinate of the car's position from the odometry.
        angle (float): Orientation angle of the car from the odometry.
        dist_closest_obstacle (float): Closest obstacle distance from the closest obstacle topic.
    """

    MAX_VELOCITY = 3.5
    TURN_VELOCITY = 2.5
    GAP_DISTANCE = 0.5
    MAX_DISTANCE = 40 
    MAX_ROT = pi/3
    DIST_FROM_LIDAR = rospy.get_param("f1tenth_simulator/scan_distance_to_base_link", 0.0)
    n=0
    def __init__(self):
        #Topics & Subscriptions,Publishers
        self.lidar_sub = rospy.Subscriber(topics.LIDARSCAN, LaserScan, self.lidar_callback)
        self.drive_pub = rospy.Publisher(topics.DRIVE, AckermannDriveStamped, queue_size=10)
        self.last_callback = rospy.get_time()
        self.odom_sub = rospy.Subscriber(topics.ODOMETRY, Odometry, self.callback_odom)
        self.closest_obstacle_sub = rospy.Subscriber(topics.CLOSEST_OBSTACLE, Float64, self.callback_closest_obstacle)
        self.speed = speed_computation(self.MAX_VELOCITY, self.TURN_VELOCITY)
        if SIMULATION:
            self.processed_ranges_pub = rospy.Publisher("/processedRanges", LaserScan, queue_size=100)
            self.target_point_pub = rospy.Publisher('/target_point', Marker, queue_size=40)
            self.edgePub = rospy.Publisher('/edges_array', MarkerArray, queue_size=40)

    def preprocess_lidar(self, data):
        """
        Preprocesses the LiDAR scan array.
        Rejects high values of lidar given MAX_DISTANCE treshold.
        Keeps only an angle of 2*self.MAX_ROT ahead of the racecar to avoid going backwards.
        Modifies ranges to create barriers for edge avoidance.

        Args:
            data (sensor_msgs.msg.LaserScan): LiDAR scan data.

        Returns:
            list: Processed LiDAR ranges.
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
        """
        Modifies the LiDAR ranges to create barriers to be avoided.

        This method modifies the LiDAR ranges based on the detected edges to create barriers to be avoided. In particular, this function handle overlapping of barriers.

        If in simulation, this function publishes arrow Marker_array corresponding to the barriers.

        Args:
            ranges (list): LiDAR ranges.
            edges_to_avoid (list): List of edges to avoid, including their start index, edge sign, angle, and distance.
            data (sensor_msgs.msg.LaserScan): LiDAR scan data.
        """
        X_from, Y_from, X_to, Y_to = [], [], [], []
        for (start, edge_sign, angle_start, dist) in edges_to_avoid:
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

    def find_best_angle(self, ranges, data):
        """
        Finds the best steering angle based on the LiDAR ranges.

        This method finds the best steering angle by choosing the furthest point in the LiDAR ranges. There is no need to take margins because the barriers created in ranges already take safety into account.

        If in simulation, publishes the processed ranges as a Laserscan message and the traget point as a Marker.

        Args:
            ranges (list): LiDAR ranges.
            data (Odometry): callback message of Odometry subscriber.

        Returns:
            float: Best steering angle.
        """
        best_index = self.start_i
        for k in range(self.start_i, self.stop_i):
            if ranges[k]>ranges[best_index]:
                best_index = k
        # best_index, dist = max(enumerate(ranges[start:stop]), key=lambda x: x[1])
        # print(best_index, ranges[best_index], start, stop)
        angle_to_drive = data.angle_min + data.angle_increment*best_index
        if SIMULATION:
            msg = LaserScan()
            msg.ranges = ranges
            self.processed_ranges_pub.publish(msg)
            if hasattr(self, 'x'):
                x = self.x + ranges[best_index]*cos(angle_to_drive+self.angle) + self.DIST_FROM_LIDAR*cos(self.angle)
                y = self.y + ranges[best_index]*sin(angle_to_drive+self.angle) + self.DIST_FROM_LIDAR*sin(self.angle)
                simple_markers.create_marker(x, y, self.target_point_pub)
        return angle_to_drive, ranges[best_index]

    def publish_drive_msg(self, angle, dist):
        """
        Publishes the drive command message with the desired steering angle.

        Args:
            angle (float): Desired steering angle.
            dist (float): Distance from the racecar to the traget point.
        """
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = angle
        velocity  = self.speed.speed(angle, self.dist_closest_obstacle)
        print("max velocity = "+str(self.MAX_VELOCITY), "distance forward = "+str(dist), "distance closest_obstacle "+str(self.dist_closest_obstacle), "current velocity cmd = "+str(velocity))
        drive_msg.drive.speed = velocity
        self.drive_pub.publish(drive_msg)

    def callback_odom(self, data):
        """
        Callback function for the odometry. It updates the car's position and orientation attributes.

        Args:
            data (nav_msgs.msg.Odometry): Odometry data.
        """
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        o = data.pose.pose.orientation
        self.angle = euler_from_quaternion([o.x, o.y, o.z, o.w])[2]

        # if not in the frame 'map'
        # transform = self.tfBuffer.lookup_transform("map", data.header.frame_id, data.header.stamp, rospy.Duration(1))
        # pose_transformed = do_transform_pose(data.pose, transform)
        # self.x = data.pose.pose.position.x
        # self.y = data.pose.pose.position.y
        # o = data.pose.pose.orientation
        # self.angle = euler_from_quaternion([o.x, o.y, o.z, o.w])[2] 

    def callback_closest_obstacle(self, data):
        self.dist_closest_obstacle = data.data

    def lidar_callback(self, data):
        """
        Callback function for the LiDAR scan. It preprocesses the LiDAR ranges, determines the best steering angle, and publishes the drive command message with the desired steering angle.

        Args:
            data (sensor_msgs.msg.LaserScan): LiDAR scan data.
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
