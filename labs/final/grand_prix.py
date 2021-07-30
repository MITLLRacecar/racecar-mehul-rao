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
import cv2 as cv
import numpy as np

sys.path.insert(0, "../../library")
import racecar_core
import racecar_utils as rc_utils
from enum import IntEnum
from nptyping import NDArray
import abc
import physics

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

# Add any global variables here
speed = 0
angle = 0

RED = ((160, 0, 0), (179, 255, 255)) # NOTE: THIS IS ON THE OTHER SIDE OF THE HUE WHEEL!
BLUE = ((90, 120, 120), (120, 255, 255))
arTagColor = None
can_switch = False
counter = 0
willTransfer = False
willTransferJump = False
willTransferTrain = False
last_angle = 0
rampDetect = True
willTransferLedge = False

col = 0

########################################################################################
# Line Following Variables
########################################################################################
MIN_CONTOUR_AREA = 30
SECOND_CROP_FLOOR = ((300, 0), (500, rc.camera.get_width()))
CROP_FLOOR = ((350, 0), (rc.camera.get_height(), rc.camera.get_width()))
GREEN = ((60, 50, 50), (80, 255, 255))
contour_center = None
contour_area = 0

########################################################################################
# Wall Following Variables
########################################################################################
RIGHT_WINDOW = 45
LEFT_WINDOW = 675
STRAIGHT_RIGHT_WINDOW = 170
STRAIGHT_LEFT_WINDOW = 550
FRONT_WINDOW = (-10, 10)
wallDone = False #False

########################################################################################
# Ar Following Variables
########################################################################################
# turn_priority = None
turn_priority = None
pillarVisible = None
pillarApproaching = None
waypointCenter = (0,0)
pillarCenter = None
MAX_SPEED_AR = 0.75
counter_ar = 0
distanceToPillar = 0
MIN_CONTOUR_AREA_AR = 100
shift_ar = 0
newPillarCenter = (0,0)
maxspeed = False

########################################################################################
# Ledge Following Variables
########################################################################################
ORANGE = ((10, 100, 100), (20, 255, 255))
PURPLE = ((125, 120, 120), (145, 255, 255))

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
    line_following = 0 # fine
    wall_following = 1 # fine
    ledge_following = 2 # work
    ar_following = 3 # definitely never going to happen
    elevator = 4 # fine
    cone_slalom = 5 # never going to happen
    train = 6 # kinda
    orange_planks = 7 # never going to happen
    jump = 8 # work
    transfer = 9 # wtf

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
color_priority = Color.BLUE
last_distance = 0

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
def angleController_ar():
    global waypointCenter
    kP = 1 # 0.8
    angle = 0
    error = waypointCenter[1] - rc.camera.get_width() / 2
    angle =  kP * error / (rc.camera.get_width() / 2)
    return rc_utils.clamp(angle, -1, 1)
    
def angleController_train():
    global col
    kP = 0.6 # 0.8
    error = col - rc.camera.get_width() / 2
    angle =  kP * error / (rc.camera.get_width() / 2)
    return rc_utils.clamp(angle, -1, 1)

# def calculateWaypoint(depthImage):
#     global state_ar, pillarApproaching, pillarVisible, pillarCenter, waypointCenter, MAX_SPEED_AR, counter_ar, distanceToPillar, MIN_CONTOUR_AREA_AR
#     global shift_ar, newPillarCenter

#     if pillarCenter is not None: 
#         distanceToPillar = depthImage[newPillarCenter[0]][newPillarCenter[1]]
#         tan73 = 3.25 
#         k = 5000
#         x = (k /distanceToPillar) * tan73

#         pointDirection = 1
#         if pillarVisible == Turn.left : 
#             pointDirection = -1

#         # Clamp to prevent viewport overflow and integer casting overflow
#         waypointCenter = (rc_utils.clamp(newPillarCenter[0], 0, rc.camera.get_height() - 1), rc_utils.clamp(int(rc_utils.clamp(newPillarCenter[1] + (pointDirection * x), 0, sys.maxsize)), 0, rc.camera.get_width() - 1))
#         # print("Found " + str(pillarVisible)[5:] + " waypoint at: " + str(waypointCenter))
#     else: 
#         # print("Could not find waypoint")

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

def update_contour_ledge(image):
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
                # print("wall follower", abs(contour_area_R - contour_area_B))
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
    rc.drive.set_max_speed(0.5)

    # Print start message
    # print(">> Final Challenge - Grand Prix")


def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    global speed, angle, cur_state, contour_center, contour_area, counter, willTransfer, color_priority, last_distance, willTransferJump, willTransferTrain, last_angle, maxspeed, rampDetect, willTransferLedge
    global wallDone, turn_priority, elevator_color, arTagColor
    global state_ar, pillarApproaching, pillarVisible, pillarCenter, waypointCenter, MAX_SPEED_AR, counter_ar, distanceToPillar, MIN_CONTOUR_AREA_AR
    global shift_ar, newPillarCenter, col

    color_image = rc.camera.get_color_image()
    depth_image = rc.camera.get_depth_image()
    top_left_inclusive = (0, 0)
    bottom_right_exclusive = ((rc.camera.get_height() // 10) * 9, rc.camera.get_width())
    cropped_depth_image = rc_utils.crop(depth_image, top_left_inclusive, bottom_right_exclusive)
    cropped_depth_height = (rc.camera.get_height() // 10) * 9
    markers = rc_utils.get_ar_markers(color_image)
    scan = rc.lidar.get_samples()
    right_dist = rc_utils.get_lidar_average_distance(scan, RIGHT_WINDOW, 10)
    left_dist = rc_utils.get_lidar_average_distance(scan, LEFT_WINDOW, 10)
    straight_right_dist = rc_utils.get_lidar_average_distance(scan, STRAIGHT_RIGHT_WINDOW, 10)
    straight_left_dist = rc_utils.get_lidar_average_distance(scan, STRAIGHT_LEFT_WINDOW, 10)
    _, forward_dist = rc_utils.get_lidar_closest_point(scan, FRONT_WINDOW)

    potential_colors = [
    ((90, 50, 50), (120, 255, 255), "blue"),
    ((40, 50, 50), (80, 255, 255), "green"),
    ((170, 50, 50), (10, 255, 255), "red"),
    ((10, 100, 100), (20, 255, 255), "orange"),
    ((125, 120, 120), (145, 255, 255), "purple")
    ]

    if markers is not None:
        for i in markers:
            i.detect_colors(color_image, (potential_colors)) 
            row_1 = i.get_corners()[0][0]
            row_2 = i.get_corners()[2][0]
            col_1 = i.get_corners()[0][1]
            col_2 = i.get_corners()[1][1]
            row = (row_1 + row_2) // 2
            col = (col_1 + col_2) // 2
            # print(i.get_corners())
            depth = rc_utils.get_pixel_average_distance(depth_image, (row, col))
            if depth == 0: depth = 1000
            # print("depth is", depth)

            #print("ID: " + str(i.get_id()))

            if i.get_id() == 0 and depth < 100:
                cur_state = State.wall_following
                willTransfer = True
                if wallDone:
                    cur_state = State.ar_following # ar_following
                rampDetect = True
            elif i.get_id() == 1 and depth < 100 and depth > 60:
                # print(i.get_color())
                if i.get_color() == "purple":
                    arTagColor = "purple"
                elif i.get_color() == "orange":
                    arTagColor = "orange"
                cur_state = State.ledge_following
                rampDetect = True
            elif i.get_id() == 199:
                cur_state = State.line_following #ar_following
                rc.drive.set_max_speed(0.62)
                if i.get_orientation().value == 1:
                    turn_priority = "Left"
                elif i.get_orientation().value == 3:
                    turn_priority = "Right"
                rampDetect = False
            elif i.get_id() == 3 and depth < 450: # used to be 320
                cur_state = State.elevator
                elevator_color = i.get_color()
                rampDetect = True
            elif i.get_id() == 4 and depth < 300:
                cur_state = State.line_following #cone_slalom
                rc.drive.set_max_speed(0.35)
                rampDetect = True
            elif i.get_id() == 5:
                willTransferTrain = False # True
                rc.drive.set_max_speed(0.7)
                cur_state = State.line_following # Traim
                rampDetect = True
            elif i.get_id() == 6 and depth < 100:
                willTransfer = False
                cur_state = State.line_following #orange_planks
                rc.drive.set_max_speed(0.5)
                rampDetect = True
            elif i.get_id() == 8 and depth < 100:
                willTransferJump = False
                cur_state = State.line_following #jump
                rc.drive.set_max_speed(0.7)
                rampDetect = True


    if cur_state == State.line_following:
        # print("line_following")
        update_contour_line(color_image)
        if contour_center is not None:
            y, x = rc_utils.get_closest_pixel(depth_image)
            distance = depth_image[y, x]
            further_y = rc_utils.clamp(y - 40, 0, cropped_depth_height)
            further_distance = depth_image[further_y, x]
            angle = rc_utils.remap_range(contour_center[1], 0, rc.camera.get_width(), -1, 1)
            if rampDetect:
                if further_distance - distance < 5:
                    # print("ramp detected")
                    speed = 0.1
                else:
                    speed = rc_utils.remap_range(abs(angle), 0, 1, 1, 0.5)
            else:
                speed = rc_utils.remap_range(abs(angle), 0, 1, 1, 0.5)

    elif cur_state == State.wall_following:
        # print("wall_following")
        rc.drive.set_max_speed(0.4)
        if not markers:
            speed = 0
            # print(left_dist, right_dist, right_dist - left_dist)

            angle = rc_utils.remap_range(right_dist - left_dist, -70, 70, -1, 1)
            angle = rc_utils.clamp(angle, -1, 1)

            speed = 1
            speed = rc_utils.clamp(speed, 0, 1)

            update_contour_line(color_image)

        if contour_center is not None and contour_area > 6500:
            # print("contour area", contour_area)
            # print("State Switched")
            cur_state = State.line_following
            rc.drive.set_max_speed(0.25)

    elif cur_state == State.ledge_following:
        wallDone = True
        rc.drive.set_max_speed(0.6)
        cropped_image = rc_utils.crop(color_image, CROP_FLOOR[0], CROP_FLOOR[1])
        width = rc.camera.get_width()//2
        orange = rc_utils.get_largest_contour(rc_utils.find_contours(cropped_image,ORANGE[0],ORANGE[1]), 60)
        purple = rc_utils.get_largest_contour(rc_utils.find_contours(cropped_image,PURPLE[0],PURPLE[1]), 60)
        #print(left_orange, right_orange, left_purple, right_purple, "\n")
        #print("\n")
        #print("\n")
        if orange is None and purple is None:
            update_contour_line(color_image)
            if contour_center is not None and contour_area > 7000:
                cur_state = State.line_following
                rc.drive.set_max_speed(0.5)
                speed = 0.7
                angle = 0
                willTransferLedge = True
            # print("setting last angle:", last_angle)
            #angle = last_angle
        else:
            if orange is None:
                orange_area = 0
            else:
                orange_area = rc_utils.get_contour_area(orange)
            if purple is None:
                purple_area = 0
            else:
                purple_area = rc_utils.get_contour_area(purple)
            color = "orange"
            current_contour = orange
            if purple_area >= orange_area:
                # print("Color Is Purple!")
                color = "purple"
                current_contour = purple
                    # if right_purple is not None and left_purple is not None and right_orange is not None and left_orange is not None:
            #     if (rc_utils.get_contour_area(right_purple) >= rc_utils.get_contour_area(right_orange)
            #     and rc_utils.get_contour_area(left_purple) >= rc_utils.get_contour_area(left_orange)):
            #         color = "purple"
            #         current_right = right_purple
            #         current_left = left_purple
            #     else:
            #         color = "orange"
            # print("color:", color, "arColor", arTagColor)
            if arTagColor == color:
                # print("speed")
                speed = 0.9
            else:
                # print("slow")
                speed = 0.2
            loc = rc_utils.get_contour_center(current_contour)[1]
            if (loc < width):
                loc += 350
            elif (loc > width):
                loc -= 500
            # print("loc:", loc, "width:", width)
            angle = ((loc - width) / (width)) * 2
            angle = rc_utils.clamp(angle, -1, 1)
            # print(angle)
            last_angle = angle
            #print("ledge_following")
        
    elif cur_state == State.ar_following:
        # print("ar_following")
        
        """
        if turn_priority == "Left":
            print("Left")
            pillarVisible = Turn.left
            shift_ar = -100
        elif turn_priority == "Right":
            print("Right")
            pillarVisible = Turn.right
            shift_ar = 100
        else:
            pillarVisible = None
        
        
        pillarContours = rc_utils.find_contours(color_image, ORANGE[0], ORANGE[1])
        newPillarContours = pillarContours
        # newPillarContours = []
        # for c in pillarContours:
        #     if rc_utils.get_contour_area(c) < 40000:
        #         newPillarContours.append(c)
        largestPillarContour = rc_utils.get_largest_contour(newPillarContours, MIN_CONTOUR_AREA_AR)
        if largestPillarContour is not None:
            pillarCA = rc_utils.get_contour_area(largestPillarContour)
            print("pillarCA", pillarCA)
        pillarCenter = rc_utils.get_contour_center(largestPillarContour)
        if pillarCenter is not None:
            newPillarCenter = (pillarCenter[0], rc_utils.clamp(pillarCenter[1] + shift_ar, 0, rc.camera.get_width()-1))
        print("newPillarCenter[1]: ", newPillarCenter[1])
        
        if largestPillarContour is not None:
            rc_utils.draw_contour(color_image, largestPillarContour)
        if pillarCenter is not None:
            rc_utils.draw_circle(color_image, (newPillarCenter[0], newPillarCenter[1]))
        
        print("right dist: ", right_dist)
        print("left_dist: ", left_dist)
        calculateWaypoint(depth_image)

        if state_ar == AR_State.approaching :
            print("approaching")
            angle = angleController_ar()
            speed = 0.5

            print("pillarApproaching: ", pillarApproaching)
            print("pillarVisible: ", pillarVisible)

            if counter_ar == 0 :
                pillarApproaching = pillarVisible
                counter_ar += rc.get_delta_time()

            if turn_priority == "Left" and right_dist < 75 or pillarCenter is None:
                state_ar = AR_State.passing
                counter_ar = 0
            elif turn_priority == "Right" and left_dist < 75 or pillarCenter is None:
                state_ar = AR_State.passing
                counter_ar = 0

        elif state_ar == AR_State.passing :
            print("passing")
            # turningSpeed = 1.0
            # coastingTime = 0.35
            # maxTurningTime = 2.0
            # turnAngle =  0.5 #0.8
            # turnInAngle = 0.5 #0.2

            counter_ar += rc.get_delta_time()
            print(counter_ar)
            speed = 0.5
            # if counter_ar < coastingTime :
            #     speed = MAX_SPEED_AR
            #     angle = turnInAngle if pillarApproaching == Turn.left else -turnInAngle
            # elif (counter_ar < maxTurningTime + coastingTime and pillarApproaching == pillarVisible) or pillarCenter is None:
            #     speed = MAX_SPEED_AR * turningSpeed
            #     angle = turnAngle if pillarApproaching == Turn.left else -turnAngle
            # else :
            #     if pillarApproaching != pillarVisible: # and counter < afterTurnTime + coastingTime + maxTurningTime
            #         state_ar = AR_State.approaching
            #         counter_ar = 0
            if turn_priority == "Left":
                angle = 0.4
            elif turn_priority == "Right":
                angle = -0.4
            else:
                angle = 0
            # if  pillarApproaching != pillarVisible: # and counter < afterTurnTime + coastingTime + maxTurningTime
            if turn_priority == "Left" and right_dist > 75:
                state_ar = AR_State.approaching
                counter_ar = 0
            elif turn_priority == "Right" and left_dist > 75:
                state_ar = AR_State.approaching
                counter_ar = 0
        
            

        ####### Mehul Lab 5
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
        """

    elif cur_state == State.elevator:
        rc.drive.set_max_speed(0.5)
        # print("elevator color:", elevator_color, "forward dist:", forward_dist)
        # print("elevator")
        speed = 0
        angle = 0
        if elevator_color == "red":
            angle = 0
            speed = 0
        elif elevator_color == "orange":
            if forward_dist < 100 and forward_dist > 60:
                speed = 0.1
                angle = 0
        elif elevator_color == "blue":
            if forward_dist < 60:
                speed = 0
                angle = 0
            else:
                speed = 0.5
                angle = 0
        
    
    elif cur_state == State.cone_slalom:
        # print("cone_slalom")
        """
        distance = 5000
        speed_multiplier = 0.5
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
        """

    elif cur_state == State.train:
        print("train")
        '''
        rc.drive.set_max_speed(1)
        angle = rc_utils.remap_range(RIGHT_WINDOW - LEFT_WINDOW, -70, 70, -1, 1)
        print(RIGHT_WINDOW - LEFT_WINDOW)
        angle = rc_utils.clamp(angle, -1, 1)
        print("angle ", angle)

        if forward_dist < 220 and forward_dist > 170 :
            speed = 0.02
            print("approaching")
        elif forward_dist > 220:
            speed = 1
            print("wheee")
        elif forward_dist < 170:
            speed = 0
            print("slowing")
        '''
        rc.drive.set_max_speed(0.75)
        #angle = rc_utils.remap_range(straight_left_dist - straight_right_dist, -100, 100, -1, 1)
        #print(straight_left_dist - straight_right_dist)
        #angle = rc_utils.clamp(angle, -1, 1)
        '''
        if (eDist < 300 or wDist < 300):
            pre_angle = (eDist - wDist) * constant
            print ("turning engaged")
        else:
            pre_angle = 0
        '''
        # angle = rc_utils.clamp(pre_angle, -1.0, 1.0)
        N_WINDOW = (-15, 15)
        nDist = rc_utils.get_lidar_closest_point(scan, N_WINDOW)[1]
        accel = rc.physics.get_linear_acceleration()[2]
        if nDist < 180 and nDist > 40:
            if accel > -0.6:
                pre_speed = (1 / nDist) * (- 6)
                # print("approaching - slowing")
            else:
                pre_speed = (1 / nDist) * (- 10)
                # print ("approaching - braking")
            speed = rc_utils.clamp(pre_speed, -1.0, 1.0)
        elif nDist > 180:
            speed = 1
            # print("wheee")
        elif nDist < 40:
            speed = 0
            # print("slowing")
        update_contour_line(color_image)
        if contour_center is not None and contour_area > 7500:
            cur_state = State.line_following
            rc.drive.set_max_speed(0.7)

    elif cur_state == State.orange_planks:
        # print("orange_planks")
        """
        rc.drive.set_max_speed(0.25)
        update_contour_plank(color_image)
        camera_width = rc.camera.get_width()
        distance_to_plank = rc_utils.get_pixel_average_distance(depth_image, contour_center)
        if contour_center[1] > (camera_width // 2) and distance_to_plank < 100:
            angle = rc_utils.remap_range(contour_center[1], camera_width // 2, camera_width, 1, 0.1)
        else:
            angle = rc_utils.remap_range(contour_center[1], 0, camera_width // 2, -0.1, -1)
        angle = rc_utils.clamp(angle, -1, 1)
        """

    elif cur_state == State.jump:
        rc.drive.set_max_speed(0.75)
        cropped_image = rc_utils.crop(color_image, SECOND_CROP_FLOOR[0], SECOND_CROP_FLOOR[1])
        blue = rc_utils.get_largest_contour(rc_utils.find_contours(cropped_image,BLUE[0],BLUE[1]), 30)
        #print(left_orange, right_orange, left_purple, right_purple, "\n")
        #print("\n")
        #print("\n")
        if blue is None:
            update_contour_line(color_image)
            if contour_center is not None and contour_area > 10000:
                cur_state = State.line_following
                rc.drive.set_max_speed(0.5)
                    # if right_purple is not None and left_purple is not None and right_orange is not None and left_orange is not None:
            #     if (rc_utils.get_contour_area(right_purple) >= rc_utils.get_contour_area(right_orange)
            #     and rc_utils.get_contour_area(left_purple) >= rc_utils.get_contour_area(left_orange)):
            #         color = "purple"
            #         current_right = right_purple
            #         current_left = left_purple
            #     else:
            #         color = "orange"
        else:
            color = "blue"
            current_contour = blue
            loc = rc_utils.get_contour_center(current_contour)[1]
            if (loc < rc.camera.get_width()/2):
                loc += 350
            elif (loc > rc.camera.get_width()/2):
                loc -= 250
            angle = 0.5 * ((loc - rc.camera.get_width()//2) / (rc.camera.get_width()//2)) * 2
            angle = rc_utils.clamp(angle, -1, 1)
        y, x = rc_utils.get_closest_pixel(depth_image)
        distance = depth_image[y, x]
        further_y = rc_utils.clamp(y - 60, 0, cropped_depth_height)
        further_distance = depth_image[further_y, x]
        # print("diff", further_distance - distance)
        speed = rc_utils.remap_range(abs(angle), 0, 1, 0.4, 0.1)
        # print("distance:", further_distance)
        if further_distance - distance < 10 or further_distance > 50:
            # print("ramp detected")
            speed = rc_utils.remap_range(abs(angle), 0, 1, 1, 1)
            angle = 0

        '''
        print("jump")
        rc.drive.set_max_speed(0.75)
        print(counter)
        if counter < 3:
            speed = 1
            angle = 0
        elif counter > 3 and counter < 5:
            speed = 0.5
            angle = 0.1
        elif counter > 5 and counter < 6.5:
            speed = 0.3
            angle = 0.2
        else:
            speed = 1
            angle = 0
            update_contour_line(color_image)
            if contour_center is not None and contour_area > 7000:
                cur_state = State.line_following
                counter = 0
                rc.drive.set_max_speed(0.5)
        counter += rc.get_delta_time()
        '''

    if willTransfer:
        # print("transferring")
        speed = 1
        angle = 0
        if counter > 0.4:
            counter = 0
            willTransfer = False
        else:
            counter += rc.get_delta_time()
            
    if willTransferLedge:
        # print("transferring")
        speed = 1
        angle = 0
        if counter > 0.1:
            counter = 0
            willTransferLedge = False
        else:
            counter += rc.get_delta_time()
            
    if willTransferJump:
        # print("transferring to jump")
        speed = 0.6
        angle = 0
        if counter > 2.5:
            counter = 0
            willTransferJump = False
        else:
            counter += rc.get_delta_time()
    
    if willTransferTrain:
        # print("transferring to train")
        speed = 0.2
        angle = angleController_train()
        if counter > 0.3:
            counter = 0
            willTransferTrain = False
        else:
            counter += rc.get_delta_time()
    #rc.display.show_color_image(color_image)
    rc.drive.set_speed_angle(speed, angle)

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