#!/usr/bin/env python
import rospy
import numpy as np
from math import atan2, sin, cos, sqrt
from numpy import linalg as LA
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose
from tf.transformations import euler_from_quaternion
from utils import simple_markers

from visualization_msgs.msg._Marker import Marker
from visualization_msgs.msg._MarkerArray import MarkerArray 
from os.path import dirname, realpath
from switching_params import topics
# TODO: import ROS msg types and libraries
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry

look_head_distance = 1.5
kp = 1.2
file_waypoint = dirname(realpath(__file__)) + "/../fichiers_csv/waypoints.csv"
print(__file__)

class PurePursuit(object):
    VELOCITY = 1.5
    """
    The class that handles pure pursuit.
    """
    def __init__(self):
        self.index = 0
        self.way_points_list = np.loadtxt(file_waypoint, delimiter=",", usecols=(0, 1))
        self.n = len(self.way_points_list)
        self.drive_pub = rospy.Publisher(topics.DRIVE, AckermannDriveStamped, queue_size=1000)
        rospy.Subscriber(topics.ODOMETRY, Odometry, self.pose_callback)

        # Essai sur les donnees 
        self.waypoints_publisher = rospy.Publisher('waypoints', MarkerArray, queue_size=1000)
        self.marker_publisher_goal = rospy.Publisher('target_point', Marker, queue_size=1000)
        self.way_points_marker()


    def way_points_marker(self):
        simple_markers.create_marker_array(self.way_points_list, self.waypoints_publisher, persistent=True)

    def get_closest_way_points(self, current_position):
        k = 60
        if self.index+k<self.n:
            indexes_to_look = [j for j in range(self.index, self.index+k)]
        else:
            indexes_to_look = [j for j in range(0, (self.index+k)%self.n)]+[j for j in range(self.index, self.n)]
        min_dist=np.inf
        for i in indexes_to_look:
            point = self.way_points_list[i]
            dist = sqrt((current_position[0]-point[0])**2+ (current_position[1]-point[1])**2)
            if dist>look_head_distance and dist<min_dist:
                self.index = i
                min_dist = dist
        if min_dist==np.inf:
            print("look_head_distance too big")
            self.index=0
            point=self.way_points_list[0]
            return sqrt((current_position[0]-point[0])**2+ (current_position[1]-point[1])**2)
        else:
            return min_dist

    def publish_drive_msg(self, steering_angle):
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = topics.ODOMETRY
        drive_msg.drive.steering_angle = kp * steering_angle
        drive_msg.drive.speed = self.VELOCITY * 2 / (2 + abs(steering_angle))
        self.drive_pub.publish(drive_msg)

    def pose_callback(self, pose_msg):
        pose = pose_msg.pose.pose
        current_position_map = pose.position
        current_position_map = [current_position_map.x, current_position_map.y]

        # find goal and avoid backloop 
        distance_goal = self.get_closest_way_points(current_position_map)

        # goal in map frame
        goal = self.way_points_list[self.index]
        print("heading to waypoint "+str(self.index)+" at coordinates (" + str(goal[0]) + ", " + str(goal[1]) + ")")
        simple_markers.create_marker(goal[0], goal[1], self.marker_publisher_goal)

        # orientation of the car in map frame
        quaternion = np.array([pose.orientation.x, 
                           pose.orientation.y, 
                           pose.orientation.z, 
                           pose.orientation.w])
        yaw = euler_from_quaternion(quaternion)[2]

        # direction to the goal in map frame
        beta = atan2(goal[1]-current_position_map[1], goal[0]-current_position_map[0])

        # goal in base_link frame
        x = distance_goal * cos(beta-yaw)
        y = distance_goal * sin(beta-yaw)

        # calculate curvature/steering angle
        steering_angle = 2 * y / (distance_goal**2)

        # publish drive message, don't forget to limit the steering angle between -0.4189 and 0.4189 radians
        self.publish_drive_msg(steering_angle)
        print(steering_angle)


def main():
    rospy.init_node('pure_pursuit')
    pp = PurePursuit()
    rospy.spin()

if __name__ == '__main__':
    main()