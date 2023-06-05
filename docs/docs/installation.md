# Introduction

Take the time to go step by step. Otherwise you won't be able to determine which step went wrong. For an example of the installation process on windows, you can look at [this video](https://www.youtube.com/watch?v=DW7l9LHdK5c).

# Setting up a unix OS

If your machine is running on a unix distribution (linux or MacOS for example) you can skip this step.

If your machine is a windows you have several options to set up a unix OS:
- Windows subsystem for linux (WSL) **(recommended)**
- dual boot
- softwares such as VMWare, Hyper-V (Gen2), VirtualBox et Parallels
- accessing a unix OS with ssh

When using a dual boot, you need to restart your computer whenever you want to switch from the unix OS and Windows. This is why we do not recommend this.
Softwares can be slow and costly in terms of runtime. They are not adapted here.
Accessing a unix OS with ssh can be a good option. However, you need to set up a redirection of graphics.

For those reasons we recommend the WSL. 

## Setting up WSL

You can see [here](https://learn.microsoft.com/fr-fr/windows/wsl/install) for informations about installing WSL. Currently on the car it's ROS `melodic` installed and ROS `melodic` works with Ubuntu 18.04. You should take that distribution of unix.

## Installing graphics

As you are on windows you need to get the graphics from WSL and display it. For this you need to download a X-launch. This can be a painful step because the settings depends on many parameters of your computer. After a lot of different tries, we came up with a solution that works on all our computers.

I suggest then that you use [vcxsrv](https://sourceforge.net/projects/vcxsrv/). And you need to set the parameter: `disable native opengl`. Otherwise it do not work.

# Installing ROS `melodic`

I suggest you use the official wiki [here](http://wiki.ros.org/melodic/Installation/Ubuntu). Go through each step carefully. It can take some time.
You should check that your ROS installation is correct by executing some basic commands such as `roscd`.

At that point you can start using ROS and looking for some ROS basic tutorials such as [these](http://wiki.ros.org/tf/Tutorials).

# Setting up `.bashrc`

To be more at ease, you should set up your `.bashrc` file. If you do not know what it is go [here](https://www.digitalocean.com/community/tutorials/bashrc-file-in-linux).

With `nano ~/.bashrc`:

This line should already be here. If not add it.
```sh
source /opt/ros/melodic/setup.bash
```

When you will make your catkin workspace you should add this line (this is explained in ROS basic tutorials)
```sh
source ~/catkin_ws/devel/setup.bash
```

To begin with ROS configuration, you can start with
```sh
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost
```

Later, if you want to use this roscore from another machine on the network, you need to replace `localhost` by your IP (something like `162.168.1.89`)

You MUST set this line to cope with X-launch
```sh
export LIBGL_ALWAYS_INDIRECT=0
```

This line is just for logging purpose.
```sh
export XDG_RUNTIME_DIR=~/Xdisplay/
```

When you open a new terminal, this bash file will be executed. Because it is the first time you can either run `source ~/.bashrc` or close and open your terminal

At this point you can use gazebo and rviz. 

# Installing f1tenth simulator

You must be aware that the simulator is different from the software installed on the car which is f1tenth_system.

To install f1tenth simulator you should follow official wiki [here](https://f1tenth.readthedocs.io/en/stable/going_forward/simulator/sim_install.html).

# IDE

I suggest you use VScode as an IDE. It integrates features to code in ROS, python or C++. But also it is very powerful to deal with git, WSL and remote connection. For those uses, you need to download the corresponding extensions.

# Conclusion

Well done, you can start using ros, the f1tenth simulator and with some effort you can also play with the racecar.