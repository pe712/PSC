# Introduction

This page explains how to run the racecar. You will need to have your computer, the racecar and a dedicated configured network.

# Remote computer settings
* Connect your remote computer to the configured wifi (ask mainteners for password)
* configure environnement variables
```sh
nano ~/.bashrc
```
Modify the lines (or add if they do not exist). You cannot let localhost because localhost = 127.0.0.1 and do not correspond to the IP in the network.
```
export ROS_MASTER_URI=http://[remote PC ip]:11311
export ROS_HOSTNAME=[remote PC ip]
```
Then exit and source the parameters.
```sh
source ~/.bashrc
```


* Open a roscore
You can decide wether you want the roscore to be on the car or on the remote computer. You need to adjust the environnement variables accordingly. For the purpose of this tutorial, we decided to have the roscore on the car. Like this you can monitor the car with graphics (like [rqt](http://wiki.ros.org/rqt) for example).
Run
```sh
roscore
```

# Racecar settings

* Turn on the car. There is two buttons to press.
* ssh to the car
This is the ssh config:
```
Host f1tenth_car_2
  HostName 192.168.1.22
  User nvidia
```

* configure environnement variables
```sh
nano ~/.bashrc
```
Modify the lines (or add if they do not exist)
```
export ROS_MASTER_URI=http://[remote PC ip]:11311
export ROS_HOSTNAME=192.168.1.22
```
Then exit and source the parameters.
```sh
source ~/.bashrc
```

* launch the f1tenth_system
Run:
```sh
roslaunch racecar teleop.launch
```

# Conclusion

You can now copy some nodes and the racecar and run them:
```sh
rosrun mypackage mynode
```

Be aware that there are some modification to make (in topics mostly) to adapt simulation algorithms to run on the f1tenth_system. I heavily suggest you to check [Moodle_page](https://moodle.polytechnique.fr/course/view.php?id=12204). To know more about it.















## Reactive method
Choose between
```
rosrun f1tenth_simulator reactive_gap_follow.py
rosrun f1tenth_simulator wall_following.py
```

## Mapping
That will localize the car in a map it is creating (SLAM).
First, in gmapping package, in gmapping/launch, modify the slam_gmapping_pr2.launch file by commenting this line
```
<remap from="scan" to="base_scan"/>
```

Then you can launch
```
roslaunch gmapping slam_gmapping_pr2.launch scan:=/scan
```
if in real world (default is odom/scan):
```
roslaunch gmapping slam_gmapping_pr2.launch 
```

When done you can copy it in maps (it download to current directory by default)
```
rosrun map_server map_saver -f circuit
```

You can then see it with
```
rosrun f1tenth_simulator waypoint_logger.py show-circuit.pgm
```
Waypoint_logger has been designed to show both P2 (ASCII) and P5 (binary) .pgm files.
If you have recorded any waypoints earlier in fichiers_csv/waypoints.csv, they will appear and the map and may be not relevant. The plot is automatically saved in fichiers_csv

## waypoints
You can see waypoints on a map map.pgm with
```
rosrun f1tenth_simulator waypoint_logger.py show-circuit.pgm
```
You can select some of them with (replace start and end by indices of waypoints)
```
rosrun f1tenth_simulator waypoint_logger.py truncate-start-end
``` 
The waypoints.csv is saved and then modified to keep only waypoints between start and end

To record waypoints
```
rosrun f1tenth_simulator waypoint_logger.py record
``` 

## Localization
Gmapping is doing SLAM. But to be more efficient when the map is created, you can use particle filters algo to localize the car.
Change map in particle_filter/launch/map_server.launch
```
roslaunch particle_filter localize.launch
```
exporting to pf/pose/odom

## Planning
```
rosrun f1tenth pure pursuit

```


