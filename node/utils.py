from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Point
from tf.transformations import quaternion_from_euler
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
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.orientation = quaternion_from_euler(0, 0, 0)
        publisher.publish(marker)

    @staticmethod
    def create_arrow(x_from, y_from, x_to, y_to, publisher):
        marker = Marker()
        marker.header.frame_id = "/map"
        marker.header.stamp = rospy.Time()
        marker.ns = "my_namespace"
        marker.type = Marker.ARROW
        marker.id = 0
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        # marker.points=[Point(x_from, y_from, 0), Point(x_to, y_to, 0)]
        # marker.pose.orientation = quaternion_from_euler(0, 0, 0)
        publisher.publish(marker)