"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

Lab 4B - LIDAR Wall Following
"""

########################################################################################
# Imports
########################################################################################

import sys
import cv2 as cv
import numpy as np

sys.path.insert(0, "../../library")
import racecar_core
import racecar_utils as rc_utils

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

# Add any global variables here

RIGHT_WINDOW = 45
LEFT_WINDOW = 675
FRONT_WINDOW = (-10, 10)

########################################################################################
# Functions
########################################################################################


def start():
    """
    This function is run once every time the start button is pressed
    """
    # Have the car begin at a stop
    rc.drive.stop()
    rc.drive.set_max_speed(0.25)
    # Print start message
    print(">> Lab 4B - LIDAR Wall Following")


def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    # rt = rc.controller.get_trigger(rc.controller.Trigger.RIGHT)
    # lt = rc.controller.get_trigger(rc.controller.Trigger.LEFT)
    speed = 0

    # TODO: Follow the wall to the right of the car without hitting anything.

    scan = rc.lidar.get_samples()
    right_dist = rc_utils.get_lidar_average_distance(scan, RIGHT_WINDOW, 10)
    left_dist = rc_utils.get_lidar_average_distance(scan, LEFT_WINDOW, 10)
    _, forward_dist = rc_utils.get_lidar_closest_point(scan, FRONT_WINDOW)
    speed = rc_utils.remap_range(forward_dist, 60, 130, 0.1, 1)
    #if forward_dist < 45:
    #    speed = -1

    print(left_dist, right_dist, right_dist - left_dist)

    angle = rc_utils.remap_range(right_dist - left_dist, -70, 70, -1, 1)
    angle = rc_utils.clamp(angle, -1, 1)
    speed = rc_utils.clamp(speed, -1, 1)
    #angle = 0
    speed = 1
    rc.drive.set_speed_angle(speed, angle)
    rc.display.show_lidar(scan)
    pass


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()
