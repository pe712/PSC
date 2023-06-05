# Setting up the codebase

You need to copy the files you want to use. Place them in a folder in the `\src` of a catkin workspace. This folder will be your package. Then compile the catkin with catkin_make. You can find more informations about this [here](http://wiki.ros.org/ROS/Tutorials/CreatingPackage).

Another option (**recommended**) is to clone the entire repo in the `\src` of a catkin workspace. Then compile the catkin. It can be easier to use our code then. To do so run:
```sh
git clone https://github.com/pe712/PSC.git
```

Some of our algorithms are designed for the racecar. Others work both in simulation and on the racecar.

You can see detailled explanations about classes and functions in the main_classes folder under.

## Creating the switcher.py file

In order that some algorithms work both in simulation and on the racecar, we have created a switcher.py file. The file `switcher.py` is supposed to contain only one line :
```py
SIMULATION = True
```
Obviously, this file is not in our repo to not be copied from a computer to the racecar.

You should create that file and place it the `\node` folder.

# Running a reactive method
You can run this to make a navigation with follow_the_gap or wall_following methods.
```
rosrun f1tenth_simulator reactive_gap_follow.py
```
Or this
```
rosrun f1tenth_simulator wall_following.py
```
Each of those methods stands alone.

# Running the planned method
You will need to:

- create a map of the racing environnement
- record waypoints (or create any type of trajectory)
- localize the racecar in that map in live
- use a planning algorithm to follow waypoints (or trajectory)

## Mapping
We use the gmapping package that you can clone from [here](https://github.com/ros-perception/slam_gmapping). You will also need to compile the catkin etc...
That will localize the car in a map it is creating (SLAM).

First, in gmapping package, in gmapping/launch, modify the slam_gmapping_pr2.launch file by commenting this line
```
<remap from="scan" to="base_scan"/>
```

Then you can launch :

- If you are in simulation
```
roslaunch gmapping slam_gmapping_pr2.launch scan:=/scan
```

- If you are on the racecar (because by default it is odom/scan):
```
roslaunch gmapping slam_gmapping_pr2.launch 
```

When done you can download it (it downloads by default in current directory)
```
rosrun map_server map_saver -f circuit
```

You can then see it with
```
rosrun f1tenth_simulator waypoint_logger.py show-circuit.pgm
```
Waypoint_logger has been designed to show both P2 (ASCII) and P5 (binary) .pgm files.
If you have recorded any waypoints earlier in fichiers_csv/waypoints.csv, they will appear and the map and may be not relevant. The plot is automatically saved in fichiers_csv.

## waypoints
You can see waypoints on a map `map_example.pgm` with
```
rosrun f1tenth_simulator waypoint_logger.py show-map_example.pgm
```
You can select some waypoints among them with (replace start and end by indices of waypoints)
```
rosrun f1tenth_simulator waypoint_logger.py truncate-start-end
``` 
A copy of waypoints.csv is saved and then the file is modified to keep only waypoints between start and end

To record waypoints, run:
```
rosrun f1tenth_simulator waypoint_logger.py record
``` 
While recording, you can move your car to create a trajectory (with teleop for example or another navigation node).

## Localization
Gmapping is doing SLAM. But to be more efficient when the map is created, you can use particle filters algo to localize only the car.
Change map in particle_filter/launch/map_server.launch
```
roslaunch particle_filter localize.launch
```
The pose estimate is exported in live to pf/pose/odom

## Planning
You need to have recorded waypoints previously. You can also place waypoints one by one but it will be quite long.
```
rosrun f1tenth pure_pursuit
```


