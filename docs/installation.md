# Vitual linux machine for windows

There are several options to have a linux OS to work with from a windows computer.

I suggest you use WSL in order to simulate a linux VM on a windows computer. You can see [here](https://learn.microsoft.com/fr-fr/windows/wsl/install) for more informations about installing WSL. Currently on the car it's ROS `melodic` installed and ROS `melodic` works with Ubuntu 18.04. You should specify that distribution of unix.

# Installing ROS `melodic`

I suggest you use the official wiki [here](http://wiki.ros.org/melodic/Installation/Ubuntu). Go through each step carefully. It can take some time
You should check that your ROS installation is correct by executing some basic commands such as `roscd`.

At that point you can start using ROS and looking for some ROS basic tutorials such as [these](http://wiki.ros.org/tf/Tutorials). However, you do not have any graphic ressources (you cannot use gazebo or rviz).

# Installing graphics

As you are on windows you need to get the graphics of the WSL and display it. For this you need to download a X-launch. This can be a painful step because it can depends on many parameters. After a lot of different tries, we came up with a solution that functions on all our computers.

I suggest then that you use [vcxsrv](https://sourceforge.net/projects/vcxsrv/). And you need to set some parameters: `disable native opengl`. Otherwise it do not work.

You also need to set up your `.bashrc` file. If you do not know what it is go [here](https://www.digitalocean.com/community/tutorials/bashrc-file-in-linux).
With `nano ~/.bashrc`:

This line should already be here. If not add it.
```sh
source /opt/ros/melodic/setup.bash
```

When you will make your catkin workspace you should add this line
```sh
source ~/catkin_ws/devel/setup.bash
```

To begin with ROS configuration, you can start with
```sh
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost
```

You MUST set this line to cope with X-launch
```sh
export LIBGL_ALWAYS_INDIRECT=0
```

This line is just for logging purpose.
```sh
export XDG_RUNTIME_DIR=~/Xdisplay/
```

At this point you can use gazebo and rviz. For more detailled installation steps you can look at [this video](https://www.youtube.com/watch?v=DW7l9LHdK5c).

# Installing f1tenth simulator

You must be aware that this is the simulatot. It is different from the software installed on the car which is f1tenth_system.

To install f1tenth simulator you should follow official wiki [here](https://f1tenth.readthedocs.io/en/stable/going_forward/simulator/sim_install.html).