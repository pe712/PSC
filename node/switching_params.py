#!/usr/bin/env python

from switcher import SIMULATION

class topics:
    """
    Lists every topic used as class parameters and set it to the right value depending on the context (simulation or experimental) 
    """
    LIDARSCAN = '/scan'
    CLOSEST_OBSTACLE = '/closest_obstacle'
    if SIMULATION:
        DRIVE = '/nav'
        ODOMETRY = '/odom'
        SAFETY = '/brake'
        SAFETY_BOOL = "/brake_bool"
    else:
        DRIVE = '/vesc/high_level/ackermann_cmd_mux/input/nav_0'
        ODOMETRY = '/vesc/odom'
        SAFETY = '/vesc/low_level/ackermann_cmd_mux/input/safety'
        SAFETY_BOOL = "/vesc/low_level/ackermann_cmd_mux/active"

