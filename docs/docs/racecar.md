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