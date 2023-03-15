#!/usr/bin/env python
import rospy
import numpy as np
import atexit
from os import getcwd
from os.path import abspath
from time import gmtime, strftime
from numpy import linalg as LA
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import sys

# Lecture de l'image PGM
def plot_fig():
    # show the map img
    with open(map_pathname, 'r') as f:
        head = f.readline()
        head = f.readline()
        sz = f.readline().split()
        wdt, hgt = int(sz[0]), int(sz[1])
        maxval = int(f.readline().strip())
        image = np.zeros((hgt, wdt))
        for y in range(hgt):
            for x in range(wdt):
                image[y, x] = int(f.read(4).strip())
    # get parameters of the img
    yaml_file_pathname = map_pathname.replace(".pgm", ".yaml")
    with open(yaml_file_pathname, 'r') as f:
        head = f.readline()
        resolution = float(f.readline().split(": ")[1])
        origin = f.readline().split(": ")[1].replace("[", "").split(", ")
        origin = [float(origin[0]), float(origin[1])]
    # Lecture des coordonnees
    coords = []
    with open(getcwd()+'/../fichiers_csv/waypoints.csv', 'r') as f:
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
    # Affichage de l'image avec les points
    plt.imshow(image, cmap='gray', origin='lower')
    for i, (x, y) in enumerate(coords):
        plt.scatter(x, y, color='r')
        plt.annotate(str(i), (x,y))
        print(x,y)
    plt.savefig(getcwd()+"/../fichiers_csv/waypoints.png")
    plt.show()

# Enregistrement du fichier csv apres avoir parcouru le circuit avec la voiture
def save_waypoint(data):
    quaternion = np.array([data.pose.pose.orientation.x, 
                           data.pose.pose.orientation.y, 
                           data.pose.pose.orientation.z, 
                           data.pose.pose.orientation.w])

    euler = euler_from_quaternion(quaternion)
    speed = LA.norm(np.array([data.twist.twist.linear.x, 
                              data.twist.twist.linear.y, 
                              data.twist.twist.linear.z]),2)
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
    ui_msg="The only arguments allowed are 'record' and 'show-map_filename'\nFor example ./waypoint_logger.py show-circuit.pgm"
    if len(sys.argv)!=2:
        print(ui_msg)
    else:
        args =sys.argv[1].split("-")
        if args[0]=="record":
            print('Saving waypoints...')
            try:
                with open(getcwd()+'/../fichiers_csv/waypoints.csv', 'w') as file:
                    listener()
            except rospy.ROSInterruptException:
                pass
        elif args[0]=="show":
            map_filename = args[1]
            map_pathname = getcwd()+'/../maps/'+map_filename
            plot_fig()
        elif args[0]=="select":
            # TODO
            a=0
        else:
            print(ui_msg)