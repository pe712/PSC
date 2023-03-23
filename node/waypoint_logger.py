#!/usr/bin/env python
import rospy
import numpy as np
from os.path import dirname
from os import rename
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
from sys import argv
from re import search

from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose


# ___________________________________ Lecture de l'image PGM ___________________________________
def plot_fig(map_pathname):
    image = np.zeros((0,0))
    width, height = 0,0
    # show the map img
    print("reading map file...")
    with open(map_pathname, 'rb') as f:
        buffer = f.read()
        match = search(
            b"(^P5\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n]\s)*)", buffer)
        if match:
            header, width, height, maxval = match.groups()
            width, height, maxval = int(width), int(height), int(maxval)
            image = np.frombuffer(buffer,
                            dtype='u1' if maxval < 256 else '<u2',
                            count=width*height,
                            offset=len(header)
                            ).reshape((height, width))
    if not match:
        # it can be a P2 (ascii file)
        with open(map_pathname, 'r') as f:
            lines = f.readlines()
            # Ignores commented lines
            count=-1
            for l in lines:
                if l[0] == '#':
                    lines.remove(l)
                    count+=1
            # Makes sure it is ASCII format (P2)
            assert lines[0].strip() == 'P2' 
            sz = lines[1].split()
            width, height = int(sz[0]), int(sz[1])
            # Converts data to a list of integers
            image = np.zeros((height, width))
            x,y = 0,0
            for line in lines[2:]:
                for value in line.split():
                    image[y,x]=value
                    x+=1
                    if x==width:
                        y+=1
                        x=0
                    if y==height:
                        print("wrong dimensions")
                        break
    image=np.transpose(image)
    plt.imshow(image, cmap='gray', origin='lower')
    # get parameters of the img
    print("plotting waypoints...")
    yaml_file_pathname = map_pathname.replace(".pgm", ".yaml")
    with open(yaml_file_pathname, 'r') as f:
        head = f.readline()
        resolution = float(f.readline().split(": ")[1])
        origin = f.readline().split(": ")[1].replace("[", "").split(", ")
        origin = [float(origin[0]), float(origin[1])]
    # Lecture des coordonnees
    coords=np.array((0, 2))
    try:
        coords = np.loadtxt("/home/pe/catkin_ws/src/f1tenth_simulator/fichiers_csv/waypoints.csv", delimiter=",", usecols=(0, 1))
    except IOError:
        # file do not exist yet
        pass
    # Transformation des coordonnees
    scale = 1/resolution
    for i, (dx, dy) in enumerate(coords):
        newy = scale * (-origin[0] + dx)
        newx = -scale * (-origin[1] + dy)+height
        plt.scatter(newx, newy, color='r')
        plt.annotate(str(i), (newx, newy))
    plt.savefig(node_directory+"/../fichiers_csv/waypoints.png")
    plt.show()

# ___________________________________ Enregistrement du fichier csv apres avoir parcouru le circuit avec la voiture ___________________________________
def save_waypoint(data):
    # transform = tfBuffer.lookup_transform("odom", "map", data.header.stamp)
    # pose_transformed = do_transform_pose(data.pose, transform)
    # print(pose_transformed.pose.position.x, data.pose.pose.position.x)
    # print("looping...")
    quaternion = np.array([data.pose.pose.orientation.x, 
                           data.pose.pose.orientation.y, 
                           data.pose.pose.orientation.z, 
                           data.pose.pose.orientation.w])

    euler = euler_from_quaternion(quaternion)
    speed = (data.twist.twist.linear.x**2 +data.twist.twist.linear.y**2+data.twist.twist.linear.z**2)**0.5
    if data.twist.twist.linear.x>0.:
        print(data.twist.twist.linear.x)
    
    if (float(data.pose.pose.position.x)!=0 or float(data.pose.pose.position.y)!=0):
        file.write('%f, %f, %f, %f\n' % (data.pose.pose.position.x,
                                        data.pose.pose.position.y,
                                        euler[2],
                                        speed))

def listener():
    # rospy.Subscriber('/odom', Odometry, save_waypoint)
    rospy.Subscriber('pf/pose/odom', Odometry, save_waypoint)
    rospy.spin()

# ___________________________________ Conserver uniquement les waypoints utiles ___________________________________
def tronque_boucle_csv(start, end):
    print("truncating...")
    with open(node_directory+'/../fichiers_csv/waypoints.csv', 'r') as f:
        lines = f.readlines()
        with open(node_directory+'/../fichiers_csv/truncated_waypoints.csv', 'w') as new_f:
            for line in lines[start:end]:
                new_f.write(line)
    rename(node_directory+'/../fichiers_csv/waypoints.csv', node_directory+'/../fichiers_csv/old_waypoints.csv')
    rename(node_directory+'/../fichiers_csv/truncated_waypoints.csv', node_directory+'/../fichiers_csv/waypoints.csv')


if __name__ == '__main__':
    node_directory = dirname(__file__)
    ui_msg="The only arguments allowed are 'record' 'truncate-start-stop' and 'show-map_filename'\nExamples\n./waypoint_logger.py record\n./waypoint_logger.py show-circuit.pgm\n./waypoint_logger.py truncate-57-230"
    if len(argv)!=2:
        print(ui_msg)
    else:
        args =argv[1].split("-")
        if args[0]=="record":
            print('Saving waypoints...')
            rospy.init_node('waypoints_logger', anonymous=True)
            # tfBuffer = Buffer()
            # tf_listener = TransformListener(tfBuffer) 
            with open(node_directory+'/../fichiers_csv/waypoints.csv', 'w') as file:
                listener()
        elif args[0]=="show":
            map_filename = args[1]
            plot_fig(node_directory+'/../maps/'+map_filename)
        elif args[0]=="truncate":
            tronque_boucle_csv(int(args[1]), int(args[2]))
        else:
            print(ui_msg)