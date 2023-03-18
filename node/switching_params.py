#!/usr/bin/env python

from switcher import SIMULATION

class topics:
    LIDARSCAN = '/scan'

    if SIMULATION:
        DRIVE = '/nav'
        ODOMETRY = '/odom'
        SAFETY = '/brake'
        SAFETY_BOOL = "/brake_bool"
    else:
        DRIVE = '/vesc/high_level/ackermann_cmd_mux/input/nav_0'
        ODOMETRY = '/vesc/odom'
        SAFETY = '/vesc/low_level/ackermann_cmd_mux/input/safety'
        SAFETY_BOOL = ""

