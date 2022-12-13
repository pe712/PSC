from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Quaternion
from math import cos, sin
import rospy

class simple_markers:
    @staticmethod
    def create_marker(x, y, publisher):
        marker = Marker()
        marker.header.frame_id = "/map"
        marker.header.stamp = rospy.Time()
        marker.ns = "my_namespace"
        marker.type = Marker.CUBE
        marker.id = 0
        marker.scale.x= 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.orientation = Quaternion(0, 0, 0, 1)
        publisher.publish(marker)

    @staticmethod
    def create_arrow(X_from, Y_from, X_to, Y_to, publisher):
        marker_array = []
        for count, (x_from, y_from, x_to, y_to) in enumerate(zip(X_from, Y_from, X_to, Y_to)):
            marker = Marker()
            marker.header.frame_id = "/map"
            marker.header.stamp = rospy.Time()
            marker.ns = "my_namespace"
            marker.id = count
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.type = Marker.ARROW
            marker.points=[Point(x_from, y_from, 0), Point(x_to, y_to, 0)]
            marker.pose.orientation = Quaternion(0, 0, 0, 1)
            marker_array.append(marker)
        publisher.publish(MarkerArray(marker_array))