We assume that you are on a windows computer. 

# Vitual linux machine for windows

There are several options to have a linux OS to work with from a windows computer.

I suggest that you use WSL in order to simulate a linux VM on a windows computer. You can see [here](https://learn.microsoft.com/fr-fr/windows/wsl/install) for more informations about installing WSL. Currently on the car it's ROS `melodic` installed and ROS `melodic` works with Ubuntu 18.04. You should take that distribution of unix.

# Installing ROS `melodic`

I suggest you use the official wiki [here](http://wiki.ros.org/melodic/Installation/Ubuntu). Go through each step carefully. It can take some time.
You should check that your ROS installation is correct by executing some basic commands such as `roscd`.

At that point you can start using ROS and looking for some ROS basic tutorials such as [these](http://wiki.ros.org/tf/Tutorials). However, you do not have any graphic ressources (you cannot use gazebo or rviz).

# Installing graphics

As you are on windows you need to get the graphics of the WSL and display it. For this you need to download a X-launch. This can be a painful step because it can depends on many parameters. After a lot of different tries, we came up with a solution that works on all our computers.

I suggest then that you use [vcxsrv](https://sourceforge.net/projects/vcxsrv/). And you need to set some parameters: `disable native opengl`. Otherwise it do not work.


# Setting up `.bashrc`

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

If you want to use this roscore from another machine on the network, you need to replace `localhost` by your IP (something like `162.168.1.89`)

You MUST set this line to cope with X-launch
```sh
export LIBGL_ALWAYS_INDIRECT=0
```

This line is just for logging purpose.
```sh
export XDG_RUNTIME_DIR=~/Xdisplay/
```

When you open a new terminal, this bash file will be executed. Because it is the first time you can either run `source ~/.bashrc` or close and open your terminal

At this point you can use gazebo and rviz. For more detailled installation steps you can look at [this video](https://www.youtube.com/watch?v=DW7l9LHdK5c).

# Installing f1tenth simulator

You must be aware that the simulator is different from the software installed on the car which is f1tenth_system.

To install f1tenth simulator you should follow official wiki [here](https://f1tenth.readthedocs.io/en/stable/going_forward/simulator/sim_install.html).

# IDE

I suggest you use VScode as an IDE. It integrates features to code in ROS, python or C++. But also it is very powerful to deal with git and remote connection.
