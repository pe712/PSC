# Introduction

This page explains how to run the f1tenth simulation.

## Setting environnement variables

If it is not done, set up your `~/.bashrc` file with those lines:
```sh
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost
```

Do not forget to save the changes by sourcing the file (`source ~/.bashrc`).

## Open a roscore

This is not necessary but heavily recommended. When you make a roslaunch it automatically creates a roscore. But it is easier to have it separated, up and running if you want to relaunch/rerun a part of the project.

```sh
roscore
```

## Start the X-launch

Run vcxsrv (X-launch) that you previously downloaded. Do not forget to set the parameter: `disable native opengl`. You can create a shortcut with this setting.

## Launch the simulator

```sh
roslaunch f1tenth_simulator simulator.launch 
```

## Manual control

You can already control the car by pressing 'k' in terminal to activate the keyboard and then using 'wasd' and 'space' keys. You can go [here](https://f1tenth.readthedocs.io/en/stable/going_forward/simulator/sim_use.html) for more basic informations.

git clone the repo
switcher.py