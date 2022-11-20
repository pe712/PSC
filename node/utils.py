from visualization_msgs.msg import Marker
from geometry_msgs.msg._Quaternion import Quaternion
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
        publisher.publish(marker)