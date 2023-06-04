from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Quaternion
import rospy

class simple_markers:
    """
    Gives handy static methods to create markers for rviz.
    """
    @staticmethod
    def construct_marker(x, y, id, color_red=0.0, color_blue=1.0, color_green=0.0):
        """
        Constructs a Marker object with the specified properties.

        Args:
            x (float): x coordinate
            y (float): y coordinate
            id (int): id of the marker
            color_red (float): red component of the marker color (default: 0.0)
            color_blue (float): blue component of the marker color (default: 1.0)
            color_green (float): green component of the marker color (default: 0.0)

        Returns:
            Marker: constructed Marker object
        """
        marker = Marker()
        marker.header.frame_id = "/map"
        marker.header.stamp = rospy.Time()
        marker.ns = "my_namespace"
        marker.type = Marker.CUBE
        marker.id = id
        marker.scale.x= 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = color_red
        marker.color.g = color_green
        marker.color.b = color_blue
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.orientation = Quaternion(0, 0, 0, 1)
        return marker

    @staticmethod
    def create_marker(x,y,publisher, id=0):
        """
        Generates a square marker at the specified (x, y) coordinates and publishes it to the specified publisher.

        Args:
            x (float): x coordinate
            y (float): y coordinate
            publisher (Publisher): publisher
            id (int, optional): id of the marker (default: 0)
        """
        publisher.publish(simple_markers.construct_marker(x,y, id, color_red=1.0, color_blue=0.0))

    @staticmethod
    def create_marker_array(XY, publisher, persistent=False):
        """
        Generates a marker array based on the specified XY coordinates and publishes it to the specified publisher.

        Args:
            XY (list): list of (x, y) coordinate pairs
            publisher (Publisher): publisher
            persistent (bool, optional): indicates whether the marker array should be published persistently (default: False)
        """
        marker_array = []
        for i, (x,y) in enumerate(XY):
            marker_array.append(simple_markers.construct_marker(x,y,i))
        marker_array = MarkerArray(marker_array)
        if persistent:
            rate = rospy.Rate(10) # 10hz
            while not rospy.is_shutdown():
                publisher.publish(marker_array)
                rate.sleep()
        else:
            publisher.publish(marker_array)

    @staticmethod
    def create_arrow(X_from, Y_from, X_to, Y_to, publisher):
        """
        Generates an arrow marker_array from the specified starting and ending coordinates and publishes it to the specified publisher.

        Args:
            X_from (list): list of x coordinates for the starting points of the arrows
            Y_from (list): list of y coordinates for the starting points of the arrows
            X_to (list): list of x coordinates for the ending points of the arrows
            Y_to (list): list of y coordinates for the ending points of the arrows
            publisher (Publisher): publisher
        """
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