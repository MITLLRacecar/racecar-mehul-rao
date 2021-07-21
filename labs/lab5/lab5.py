"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

Lab 5 - AR Markers
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
from enum import Enum

########################################################################################
# Global variables
########################################################################################
class Mode(Enum):
    wall_following = 0
    line_following = 1


rc = racecar_core.create_racecar()

# Add any global variables here
RIGHT_WINDOW = 45
LEFT_WINDOW = 675
FRONT_WINDOW = (-10, 10)
turn_priority = None

MIN_CONTOUR_AREA = 30

# A crop window for the floor directly in front of the car
SECOND_CROP_FLOOR = ((200, 0), (350, rc.camera.get_width()))
CROP_FLOOR = ((350, 0), (rc.camera.get_height(), rc.camera.get_width()))

# Colors, stored as a pair (hsv_min, hsv_max)
BLUE = ((90, 120, 120), (120, 255, 255))  # The HSV range for the color blue
# TODO (challenge 1): add HSV ranges for other colors
GREEN = ((60, 50, 50), (80, 255, 255))
RED = ((0, 50, 50), (0, 255, 255)) # original value was (0, 100, 100)
color_list = [GREEN, BLUE, RED]

contour_center = None  # The (pixel row, pixel column) of contour
contour_area = 0  # The area of contour
curr_mode = Mode.wall_following

########################################################################################
# Functions
########################################################################################
def update_contour():
    """
    Finds contours in the current color image and uses them to update contour_center
    and contour_area
    """
    global contour_center
    global contour_area
    global color_list

    image = rc.camera.get_color_image()

    if image is None:
        contour_center = None
        contour_area = 0
    else:
        # TODO (challenge 1): Search for multiple tape colors with a priority order
        # Priority: Red, Green, Blue
        # (currently we only search for blue)
        # Crop the image to the floor directly in front of the car

        image = rc_utils.crop(image, CROP_FLOOR[0], CROP_FLOOR[1])

        for color in color_list:
            # Find all of the blue contours
            contours = rc_utils.find_contours(image, color[0], color[1])
            # Select the largest contour
            contour = rc_utils.get_largest_contour(contours, MIN_CONTOUR_AREA)

            if contour is not None:
                # Calculate contour information
                contour_center = rc_utils.get_contour_center(contour)
                contour_area = rc_utils.get_contour_area(contour)

                # Draw contour onto the image
                rc_utils.draw_contour(image, contour)
                rc_utils.draw_circle(image, contour_center)
                break

            else:
                contour_center = None
                contour_area = 0

def start():
    """
    This function is run once every time the start button is pressed
    """
    # Have the car begin at a stop
    rc.drive.stop()
    rc.drive.set_max_speed(0.25)

    # Print start message
    print(">> Lab 5 - AR Markers")



def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    global turn_priority
    global color_list
    global curr_mode

    potential_colors = [
    ((90, 50, 50), (120, 255, 255), "blue"),
    ((40, 50, 50), (80, 255, 255), "green"),
    ((170, 50, 50), (10, 255, 255), "red")
    ]

    speed = 0
    angle = 0
    color_image = rc.camera.get_color_image()
    markers = rc_utils.get_ar_markers(color_image)
    scan = rc.lidar.get_samples()
    right_dist = rc_utils.get_lidar_average_distance(scan, RIGHT_WINDOW, 10)
    left_dist = rc_utils.get_lidar_average_distance(scan, LEFT_WINDOW, 10)
    _, forward_dist = rc_utils.get_lidar_closest_point(scan, FRONT_WINDOW)
    #speed = rc_utils.remap_range(forward_dist, 60, 130, 0, 1)
    if curr_mode == Mode.wall_following:
        angle = rc_utils.remap_range(right_dist - left_dist, -70, 70, -1, 1)
        if forward_dist < 60:
            if turn_priority == "Left":
                angle = -1
                speed = rc_utils.remap_range(forward_dist, 40, 80, 0.3, 1)
                #speed = 1
                print("LEFT")
            elif turn_priority == "Right":
                angle = 1
                speed = rc_utils.remap_range(forward_dist, 40, 80, 0.3, 1)
                #speed = 1
                print("RIGHT")
        print(left_dist, right_dist, right_dist - left_dist)

    if curr_mode == Mode.line_following:
        update_contour()
        # Choose an angle based on contour_center
        # If we could not find a contour, keep the previous angle
        if contour_center is not None:
            angle = rc_utils.remap_range(contour_center[1], 0, rc.camera.get_width(), -1, 1)

    angle = rc_utils.clamp(angle, -1, 1)
    speed = rc_utils.clamp(speed, 0, 1)
    #angle = 0
    speed = 1
    rc.drive.set_speed_angle(speed, angle)
    rc.display.show_lidar(scan)
    # TODO: Turn left if we see a marker with ID 0 and right for ID 1
    if markers is not None:
        for i in markers:
            i.detect_colors(color_image, potential_colors)
            if i.get_id() == 0:
                turn_priority = "Left"
                curr_mode = Mode.wall_following
            elif i.get_id() == 1:
                turn_priority = "Right"
                curr_mode = Mode.wall_following
            elif i.get_id() == 199:
                if i.get_orientation().value == 1:
                    turn_priority = "Left"
                    curr_mode = Mode.wall_following
                elif i.get_orientation().value == 3:
                    turn_priority = "Right"
                    curr_mode = Mode.wall_following
            elif i.get_id() == 2:
                if i.get_color() == "red":
                    color_list = [RED, GREEN, BLUE]
                    curr_mode = Mode.line_following
                    print("RED")
                elif i.get_color() == "blue":
                    color_list = [BLUE, GREEN, RED]
                    curr_mode = Mode.line_following
                    print("BLUE")


    # TODO: If we see a marker with ID 2, follow the color line which matches the color
    # border surrounding the marker (either blue or red). If neither color is found but
    # we see a green line, follow that instead.


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()
