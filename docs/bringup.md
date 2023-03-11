## Adapting algorithm for f1tenth_system

I heavily suggest you to check [Moodle_page](https://moodle.polytechnique.fr/course/view.php?id=12204). There are some modification to make (in topics mostly) to adapt simulation algorithms to run on the f1tenth_system

## running the car

* take racecar 2
* ssh to the host (password is nvidia)

```
Host f1tenth_car_2
  HostName 192.168.1.22
  User nvidia
```

* wifi tomate and pwd=cornichon

* on the car:
```sh
nano ~/.bashrc
```
Write (replace 192.168.1.106 by remote PC ip)
```
export ROS_MASTER_URI=http://192.168.1.106:11311
export ROS_HOSTNAME=192.168.1.22
```
Then run:
```sh
source ~/.bashrc
roslaunch racecar teleop.launch
rosrun f1tenth_simulator my_node.py
```

* on the remmote PC (in order to be able to launch rqt from remote PC)(replace 192.168.1.106 by remote PC ip):
```sh
nano ~/.bashrc
```
Write
```
export ROS_MASTER_URI=http://192.168.1.106:11311
export ROS_HOSTNAME=192.168.1.106
```

## 
```
roslaunch racecar teleop.launch
```

## Reactive method
Choose between
```
rosrun f1tenth_simulator reactive_gap_follow.py
rosrun f1tenth_simulator wall_following.py
```

## Mapping
That will localize the car in a map it is creating (SLAM).
```
roslaunch gmapping slam_gmapping_pr2.launch
```

When done you can download it
```
rosrun map_server map_saver -f circuit.pgm
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

```


