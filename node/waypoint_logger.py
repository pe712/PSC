#!/usr/bin/env python
import rospy
import numpy as np
from os.path import dirname
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
from sys import argv
from re import search

# Lecture de l'image PGM
def plot_fig(map_pathname):
    # show the map img
    with open(map_pathname, 'rb') as f:
        buffer = f.read()
        match = search(
            b"(^P5\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n]\s)*)", buffer)
        if match:
            header, width, height, maxval = match.groups()
        else:
            raise ValueError("Not a raw PGM file: '%s'" % map_pathname)
        image = np.frombuffer(buffer,
                            dtype='u1' if int(maxval) < 256 else '<u2',
                            count=int(width)*int(height),
                            offset=len(header)
                            ).reshape((int(height), int(width)))
    plt.imshow(image, cmap='gray', origin='lower')
    # get parameters of the img
    yaml_file_pathname = map_pathname.replace(".pgm", ".yaml")
    with open(yaml_file_pathname, 'r') as f:
        head = f.readline()
        resolution = float(f.readline().split(": ")[1])
        origin = f.readline().split(": ")[1].replace("[", "").split(", ")
        origin = [float(origin[0]), float(origin[1])]
    # Lecture des coordonnees
    coords = []
    with open(node_directory+'/../fichiers_csv/waypoints.csv', 'r') as f:
        for line in f:
            point = line.split(", ")
            x, y = float(point[0]), float(point[1])
            coords.append((x, y))
    # Transformation des coordonnees
    scale = 1/resolution
    dx, dy = origin[0], origin[1] # exemple de translation
    print(dx,dy)
    for i, (x, y) in enumerate(coords):
        newx = scale * (x + dx)
        newy = scale * (y + dy)
        coords[i]=(newx, newy)
    for i, (x, y) in enumerate(coords):
        plt.scatter(x, y, color='r')
        plt.annotate(str(i), (x,y))
        print(x,y)
    plt.savefig(node_directory+"/../fichiers_csv/waypoints.png")
    plt.show()

# Enregistrement du fichier csv apres avoir parcouru le circuit avec la voiture
def save_waypoint(data):
    quaternion = np.array([data.pose.pose.orientation.x, 
                           data.pose.pose.orientation.y, 
                           data.pose.pose.orientation.z, 
                           data.pose.pose.orientation.w])

    euler = euler_from_quaternion(quaternion)
    speed = (data.twist.twist.linear.x**2 +data.twist.twist.linear.y**2+data.twist.twist.linear.z**2)**0.5
    if data.twist.twist.linear.x>0.:
        print(data.twist.twist.linear.x)

    file.write('%f, %f, %f, %f\n' % (data.pose.pose.position.x,
                                     data.pose.pose.position.y,
                                     euler[2],
                                     speed))

def listener():
    rospy.init_node('waypoints_logger', anonymous=True)
    rospy.Subscriber('pf/pose/odom', Odometry, save_waypoint)
    rospy.spin()

if __name__ == '__main__':
    node_directory = dirname(__file__)
    ui_msg="The only arguments allowed are 'record' and 'show-map_filename'\nFor example './waypoint_logger.py show-circuit.pgm' or ./waypoint_logger.py record"
    if len(argv)!=2:
        print(ui_msg)
    else:
        args =argv[1].split("-")
        if args[0]=="record":
            print('Saving waypoints...')
            with open(node_directory+'/../fichiers_csv/waypoints.csv', 'w') as file:
                listener()
        elif args[0]=="show":
            map_filename = args[1]
            plot_fig(node_directory+'/../maps/'+map_filename)
        elif args[0]=="select":
            # TODO
            a=0
        else:
            print(ui_msg)