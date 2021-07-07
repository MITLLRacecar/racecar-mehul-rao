"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

Lab 1 - Driving in Shapes
"""

########################################################################################
# Imports
########################################################################################

import sys

sys.path.insert(1, "../../library")
import racecar_core
import racecar_utils as rc_utils

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

# Put any global variables here
counter = 0
end_count = 0
isA = False
isB = False
isTurningB = False
isX = False
isTurningX = False
isY = False
square_count = 0
figure_turn_count = 0

########################################################################################
# Functions
########################################################################################


def start():
    """
    This function is run once every time the start button is pressed
    """
    # Begin at a full stop
    rc.drive.stop()

    # Print start message
    # TODO (main challenge): add a line explaining what the Y button does
    print(
        ">> Lab 1 - Driving in Shapes\n"
        "\n"
        "Controls:\n"
        "    Right trigger = accelerate forward\n"
        "    Left trigger = accelerate backward\n"
        "    Left joystick = turn front wheels\n"
        "    A button = drive in a circle\n"
        "    B button = drive in a square\n"
        "    X button = drive in a figure eight\n"
    )

def update():
    global counter
    global isA
    global isB
    global isX
    global isY
    global end_count
    global square_count
    global isTurningB
    global figure_turn_count
    global isTurningX

    (x, y) = rc.controller.get_joystick(rc.controller.Joystick.LEFT)
    speed_forward = rc.controller.get_trigger(rc.controller.Trigger.RIGHT)
    speed_backward = rc.controller.get_trigger(rc.controller.Trigger.LEFT)
    speed = speed_forward - speed_backward
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """

    # Implement acceleration and steering
    if rc.controller.is_down(rc.controller.Button.RB):
        rc.drive.set_speed_angle(speed, x)

    if rc.controller.was_pressed(rc.controller.Button.LB):
        rc.drive.stop()
        end_count = 0
        isA = False
        isB = False
        isTurningB = False
        isX = False
        isTurningX = False
        isY = False
        square_count = 0
        figure_turn_count = 0

    # TODO (main challenge): Drive in a circle
    if rc.controller.was_pressed(rc.controller.Button.A):
        print("Driving in a circle...")
        rc.drive.set_speed_angle(1, 0.5)
        isA = True
        end_count = counter + 9.6

    if isA:
        print(end_count - counter)
        if counter > end_count:
            print("stopping A")
            rc.drive.stop()
            isA = False
            end_count = 0

    # TODO (main challenge): Drive in a square when the B button is pressed
    if rc.controller.was_pressed(rc.controller.Button.B):
        print("Driving in a square...")
        isB = True
        rc.drive.set_speed_angle(1, 0)
        end_count = counter + 2
        square_count = 1

    if isB:
        if square_count == 5:
            print("stopping b")
            rc.drive.stop()
            isB = False
        if isTurningB:
            if counter > end_count:
                print("stop turn b")
                rc.drive.set_speed_angle(1, 0)
                isTurningB = False
                end_count = counter + 2
                square_count += 1
        elif counter > end_count:
            print("turning b")
            rc.drive.set_speed_angle(0.2, 1)
            isTurningB = True
            end_count = counter + 2.8

    # TODO (main challenge): Drive in a figure eight when the X button is pressed
    if rc.controller.was_pressed(rc.controller.Button.X):
        print("Driving in a figure 8...")
        rc.drive.set_speed_angle(1, 0.5)
        isX = True
        figure_turn_count = 1
        end_count = counter + 9.6

    if isX:
        if counter > end_count:
            rc.drive.stop()
            end_count = 0
            if figure_turn_count < 2:
                rc.drive.set_speed_angle(1, -0.5)
                end_count = counter + 9.6
                figure_turn_count += 1
            else:
                print("got to else")
                rc.drive.stop()
                isX = False


    # TODO (main challenge): Drive in a shape of your choice when the Y button
    # TODO is pressed
    if rc.controller.was_pressed(rc.controller.Button.Y):
        print("Driving in a shape...")

    counter += rc.get_delta_time()

########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update)
    rc.go()