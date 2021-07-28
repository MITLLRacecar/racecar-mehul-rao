"""
Copyright MIT and Harvey Mudd College
MIT License
Fall 2020

Final Challenge - Grand Prix
"""

########################################################################################
# Imports
########################################################################################

import sys
# import cv2 as cv
# import numpy as np

sys.path.insert(0, "../../library")
import racecar_core
import racecar_utils as rc_utils
from enum import IntEnum

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

# Add any global variables here
speed = 0
angle = 0

RED = ((160, 0, 0), (179, 255, 255)) # NOTE: THIS IS ON THE OTHER SIDE OF THE HUE WHEEL!
BLUE = ((90, 120, 120), (120, 255, 255))

can_switch = False
counter = 0
willTransfer = False

########################################################################################
# Line Following Variables
########################################################################################
MIN_CONTOUR_AREA = 30
SECOND_CROP_FLOOR = ((200, 0), (350, rc.camera.get_width()))
CROP_FLOOR = ((350, 0), (rc.camera.get_height(), rc.camera.get_width()))
GREEN = ((60, 50, 50), (80, 255, 255))
contour_center = None
contour_area = 0

########################################################################################
# Wall Following Variables
########################################################################################
RIGHT_WINDOW = 45
LEFT_WINDOW = 675
FRONT_WINDOW = (-10, 10)
wallDone = False #False

########################################################################################
# Ar Following Variables
########################################################################################
# turn_priority = None
coneVisible = None
coneApproaching = None
waypointCenter = (0,0)
coneCenter = None

########################################################################################
# Ledge Following Variables
########################################################################################
ORANGE = ((10, 100, 100), (20, 255, 255))
PURPLE = ((90, 120, 120), (120, 255, 255)) ##Temp values, these are for blue

########################################################################################
# Elevator Variables
########################################################################################
elevator_color = None

########################################################################################
# Plank Variables
########################################################################################

########################################################################################
# Main State Machine
########################################################################################
class State(IntEnum):
    line_following = 0
    wall_following = 1
    ledge_following = 2
    ar_following = 3
    elevator = 4
    cone_slalom = 5
    train = 6
    orange_planks = 7
    jump = 8
    transfer = 9

cur_state = State.line_following

########################################################################################
# Cone Slalom State Machine
########################################################################################
class Mode(IntEnum):
    searching = 0
    red = 1
    blue = 2
    linear = 3
class Color(IntEnum):
    RED = 0
    BLUE = 1
    BOTH = 2

cur_mode = Mode.searching
color_priority = Color.RED

########################################################################################
# AR Following State Machine
########################################################################################
class AR_State(IntEnum) :
    approaching = 0 # Go around red cone
    passing = 1 # Go around blue cone
    searching = 2 # Manual control until we re-aquire

class Turn(IntEnum) :
    left = 0
    right = 1
    
state_ar = AR_State.approaching

########################################################################################
# Helper Functions
########################################################################################
def update_contour_line(image):
    """
    Finds contours in the current color image and uses them to update contour_center
    and contour_area
    """
    global contour_center, contour_area, GREEN

    if image is None:
        contour_center = None
        contour_area = 0
    else:
        image = rc_utils.crop(image, CROP_FLOOR[0], CROP_FLOOR[1])

        contours = rc_utils.find_contours(image, GREEN[0], GREEN[1])
        contour = rc_utils.get_largest_contour(contours, MIN_CONTOUR_AREA)

        if contour is not None:
            # Calculate contour information
            contour_center = rc_utils.get_contour_center(contour)
            contour_area = rc_utils.get_contour_area(contour)

            # Draw contour onto the image
            rc_utils.draw_contour(image, contour)
            rc_utils.draw_circle(image, contour_center)
        else:
            contour_center = None
            contour_area = 0

def update_contour_plank(image):
    """
    Finds contours in the current color image and uses them to update contour_center
    and contour_area
    """
    global contour_center, contour_area, ORANGE

    if image is None:
        contour_center = None
        contour_area = 0
    else:

        contours = rc_utils.find_contours(image, ORANGE[0], ORANGE[1])
        contour = rc_utils.get_largest_contour(contours, MIN_CONTOUR_AREA)

        if contour is not None:
            # Calculate contour information
            contour_center = rc_utils.get_contour_center(contour)
            contour_area = rc_utils.get_contour_area(contour)

            # Draw contour onto the image
            rc_utils.draw_contour(image, contour)
            rc_utils.draw_circle(image, contour_center)
        else:
            contour_center = None
            contour_area = 0

def update_contour_cones(color_image):
    """
    Finds contoours for the blue and red cone using color image
    """

    MIN_CONTOUR_AREA = 800

    if color_image is None:
        contour_center = None
        contour_area = 0

    else:
        contours_R = rc_utils.find_contours(color_image, RED[0], RED[1])
        contour_R = rc_utils.get_largest_contour(contours_R, MIN_CONTOUR_AREA)
        contours_B = rc_utils.find_contours(color_image, BLUE[0], BLUE[1])
        contour_B = rc_utils.get_largest_contour(contours_B, MIN_CONTOUR_AREA)

        if contour_R is not None and contour_B is not None:
            contour_area_R = rc_utils.get_contour_area(contour_R)
            contour_area_B = rc_utils.get_contour_area(contour_B)

            if abs(contour_area_R - contour_area_B) < 700:
                print("wall follower", abs(contour_area_R - contour_area_B))
                return None, Color.BOTH

            elif contour_area_R > contour_area_B:
                return contour_R, Color.RED

            else:
                return contour_B, Color.BLUE

        elif contour_R is None and contour_B is not None:
            return contour_B, Color.BLUE

        elif contour_B is None and contour_R is not None: 
            return contour_R, Color.RED

        else:
            return None, None

########################################################################################
# Main Functions
########################################################################################

def start():
    """
    This function is run once every time the start button is pressed
    """
    # Have the car begin at a stop
    global speed, angle
    

    # Initialize variables
    speed = 0
    angle = 0

    # Set initial driving speed and angle
    rc.drive.set_speed_angle(speed, angle)

    # Print start message
    print(">> Final Challenge - Grand Prix")


def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    global speed, angle, cur_state, contour_center, contour_area, counter, willTransfer
    global wallDone, turn_priority, elevator_color, state_ar

    color_image = rc.camera.get_color_image()
    depth_image = rc.camera.get_depth_image()
    markers = rc_utils.get_ar_markers(color_image)
    scan = rc.lidar.get_samples()
    right_dist = rc_utils.get_lidar_average_distance(scan, RIGHT_WINDOW, 10)
    left_dist = rc_utils.get_lidar_average_distance(scan, LEFT_WINDOW, 10)
    _, forward_dist = rc_utils.get_lidar_closest_point(scan, FRONT_WINDOW)

    potential_colors = [
    ((90, 50, 50), (120, 255, 255), "blue"),
    ((40, 50, 50), (80, 255, 255), "green"),
    ((170, 50, 50), (10, 255, 255), "red"),
    ((10, 100, 100), (20, 255, 255), "orange"),
    ((90, 120, 120), (120, 255, 255), "purple")
    ]

    if markers is not None:
        for i in markers:
            i.detect_colors(color_image, (potential_colors)) 
            row, col = i.get_corners()[0]
            depth = depth_image[row][col]

            #print("ID: " + str(i.get_id()))

            if i.get_id() == 0 and depth < 100:
                cur_state = State.wall_following
                willTransfer = True
                if wallDone:
                    cur_state = State.ar_following
            elif i.get_id() == 1:
                if i.get_color() == "purple":
                    print("It's purple!")
                elif i.get_color() == "orange":
                    print("It's orange!")
                cur_state = State.ledge_following
            elif i.get_id() == 199:
                willTransfer = True
                cur_state = State.ar_following
                if i.get_orientation().value == 1:
                    turn_priority = "Left"
                elif i.get_orientation().value == 3:
                    turn_priority = "Right"
            elif i.get_id() == 3:
                cur_state = State.elevator
                elevator_color = i.get_color()
            elif i.get_id() == 4:
                cur_state = State.cone_slalom
            elif i.get_id() == 5:
                cur_state = State.train
            elif i.get_id() == 6 and depth < 100:
                willTransfer = True
                cur_state = State.orange_planks
            elif i.get_id() == 8:
                cur_state = State.jump
        
    
    if cur_state == State.line_following:
        print("line_following")
        rc.drive.set_max_speed(0.7)
        update_contour_line(color_image)
        if contour_center is not None:
            angle = rc_utils.remap_range(contour_center[1], 0, rc.camera.get_width(), -1, 1)
            speed = rc_utils.remap_range(abs(angle), 0, 1, 1, 0.2)

    elif cur_state == State.wall_following:
        print("wall_following")
        rc.drive.set_max_speed(0.4)
        if not markers:
            speed = 0
    
            # print(left_dist, right_dist, right_dist - left_dist)

            angle = rc_utils.remap_range(right_dist - left_dist, -70, 70, -1, 1)
            angle = rc_utils.clamp(angle, -1, 1)

            speed = 1
            speed = rc_utils.clamp(speed, 0, 1)

            rc.drive.set_speed_angle(speed, angle)
            update_contour_line(color_image)

        if contour_center is not None and contour_area > 7000:
            print(contour_area)
            print("State Switched")
            wallDone = True
            cur_state = State.line_following

    elif cur_state == State.ledge_following:
        cropped_image = rc_utils.crop(color_image, (0, 0), (color_image, 10))
        rc.drive.set_max_speed(0.3)
        speed = 0
        angle = 0
        print("ledge_following")


    elif cur_state == State.ar_following:
        print("ar_following")
        speed = 0
        angle = 0
        rc.drive.set_max_speed(0.3)

        findCone()
        calculateWaypoint(depth_image)

        if state_ar == state_ar.approaching :
            approachCone()
        elif state_ar == state_ar.passing :
            passCone()
        elif state_ar == state_ar.searching:
            search()
            # TODO: Implement autonomous searching

        # angle = rc_utils.remap_range(right_dist - left_dist, -70, 70, -1, 1)

        # speed = 1
        # speed = rc_utils.clamp(speed, 0, 1)
        # angle = rc_utils.clamp(angle, -1, 1)

        # if forward_dist < 100:
        #     if turn_priority == "Left":
        #         angle = -1
        #         speed = 0.5
        #     elif turn_priority == "Right":
        #         angle = 1
        #         speed = 0.5

        # print(left_dist, right_dist, right_dist - left_dist)

    elif cur_state == State.elevator:
        print("elevator")
        angle = 0
    elif cur_state == State.cone_slalom:
        print("cone_slalom")

        speed = 0.0
        angle = 0.0
        distance = 5000
        speed_multiplier = 1
        distance_param = 200

        # Get camera sizes
        camera_height = (rc.camera.get_height() // 10) * 10
        camera_width = (rc.camera.get_width() // 10) * 10
        top_left_inclusive = (0, rc.camera.get_width() - camera_width)
        bottom_right_exclusive = ((camera_height, camera_width))

        # Cropping both images
        rc_utils.crop(color_image, top_left_inclusive, bottom_right_exclusive)
        rc_utils.crop(depth_image, top_left_inclusive, bottom_right_exclusive)

        # Getting contours from update_contours()
        contour, color = update_contour_cones(color_image)

        # Copy the image
        color_image_display = np.copy(color_image)

        if contour is not None:
            # Getting contour center
            contour_center = rc_utils.get_contour_center(contour)

            # Drawing a color image with the contour
            rc_utils.draw_contour(color_image_display, contour)
            rc_utils.draw_circle(color_image_display, contour_center)

            # Calculate distance
            distance = rc_utils.get_pixel_average_distance(depth_image, contour_center)
            last_distance = distance

        else:
            cur_mode = Mode.searching

        # Setting the current state
        if color == Color.RED:
            cur_mode = Mode.red
            color_priority = Color.RED
        elif color == Color.BLUE:
            cur_mode = Mode.blue
            color_priority = Color.BLUE
        elif color == Color.BOTH:
            cur_mode = Mode.linear
        else:
            cur_mode = Mode.searching

        # Check current mode and implement the respective actions
        if cur_mode == Mode.red and (distance < distance_param):
            # TODO: Red Cone Logic -> drive right to avoid
            angle = rc_utils.remap_range(contour_center[1], 0, camera_width, 0.3, 1)
            angle *= rc_utils.remap_range(last_distance, 200, 50, 0, 2)
            print("RED, ANGLE:", angle)
            counter = 0
        elif cur_mode == Mode.blue and (distance < distance_param):
            # TODO: Blue Cone Logic -> drive left to avoid
            angle = rc_utils.remap_range(contour_center[1], 0, camera_width, -1, -0.3)
            angle *= rc_utils.remap_range(last_distance, 50, 200, 2, 0)
            counter = 0
        elif (cur_mode == Mode.blue or cur_mode == Mode.red) and distance >= distance_param:
            if cur_mode == Mode.blue:
                angle = rc_utils.remap_range(contour_center[1], 0, camera_width, -1, 1)
            elif cur_mode == Mode.red:
                angle = rc_utils.remap_range(contour_center[1], 0, camera_width, -1, 1)
            print("waiting")
            counter = 0
        elif cur_mode == Mode.linear:
            print("got here")
            angle = 0
            counter = 0
        else:
            if color_priority == Color.RED:
                angle = rc_utils.remap_range(last_distance, 0, 100, -0.3, -0.65) # drive left to return
            else:
                angle = rc_utils.remap_range(last_distance, 0, 100, 0.3, 0.65) # drive right to return

        # Clamping functions
        angle = rc_utils.clamp(angle, -1, 1)
        speed = rc_utils.remap_range(last_distance, 60, 150, 0.1, 0.98)
        speed = rc_utils.clamp(speed, 0.1, 1)

        # Displaying the color camera that was drawn on

    elif cur_state == State.train:
        print("train")

    elif cur_state == State.orange_planks:
        print("orange_planks")
        rc.drive.set_max_speed(0.25)
        update_contour_plank(color_image)
        camera_width = rc.camera.get_width()
        distance_to_plank = rc_utils.get_pixel_average_distance(depth_image, contour_center)
        if contour_center[1] > (camera_width // 2) and distance_to_plank < 100:
            angle = rc_utils.remap_range(contour_center[1], camera_width // 2, camera_width, 1, 0.1)
        else:
            angle = rc_utils.remap_range(contour_center[1], 0, camera_width // 2, -0.1, -1)

    elif cur_state == State.jump:
        print("jump")

    if willTransfer:
        print("transfering")
        speed = 1
        angle = 0
        if contour_center is None:
            if counter > 0.05:
                counter = 0
                willTransfer = False
            else:
                counter += rc.get_delta_time()
    rc.display.show_color_image(color_image)
    rc.drive.set_speed_angle(speed, angle)

def angleController_ar():
    global waypointCenter
    kP = 0.8 
    angle = 0
    error = waypointCenter[1] - rc.camera.get_width() / 2
    angle =  kP * error / (rc.camera.get_width() / 2)
    return rc_utils.clamp(angle, -1, 1)

########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()

########################################################################################
# AR TAG IDS
########################################################################################
"""
Wall Following: ID: 0, Orientation: Up, Color: None

Ledge Following: ID: 1, Orientation: Up, Color: Orange

AR Following: ID: 0, Orientation: Up, Color: None
    Turning Left: ID: 199, Orientation: Left, Color: None
    Turning Right: ID: 199, Orientation: Right, Color: None

Elevator:
    Go: ID: 3, Orientation: Up, Color: Blue
    Warning: ID: 3, Orientation: Up, Color: Orange
    Stop: ID: 3, Orientation: Up, Color: Red

Cone Slalom: ID:4, Orientation : Up, Color: None

Train: ID: 5, Orientation: Up, Color: None

Orange Planks: ID 6: , Orientation: Up, Color: None

Jump: ID: 8, Orientation: Up, Color: None

"""