"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

Lab 3B - Depth Camera Cone Parking
"""

########################################################################################
# Imports
########################################################################################

import sys
import cv2 as cv
import numpy as np
from typing import Any, Tuple, List, Optional
from nptyping import NDArray

sys.path.insert(0, "../../library")
import racecar_core
import racecar_utils as rc_utils

########################################################################################
# Global variables
########################################################################################
rc = racecar_core.create_racecar()

# >> Constants
# The smallest contour we will recognize as a valid contour
MIN_CONTOUR_AREA = 30

# The HSV range for the color orange, stored as (hsv_min, hsv_max)
ORANGE = ((10, 100, 100), (20, 255, 255))

# >> Variables
speed = 0.0  # The current speed of the car
angle = 0.0  # The current angle of the car's wheels
contour_center = None  # The (pixel row, pixel column) of contour
contour_area = 0  # The area of contour

# Add any global variables here

isParked = False

########################################################################################
# Functions
########################################################################################
def get_closest_pixel(
    depth_image: NDArray[(Any, Any), np.float32],
    kernel_size: int = 5
) -> Tuple[int, int]:
    """
    Finds the closest pixel in a depth image.

    Args:
        depth_image: The depth image to process.
        kernel_size: The size of the area to average around each pixel.

    Returns:
        The (row, column) of the pixel which is closest to the car.

    Warning:
        kernel_size be positive and odd.
        It is highly recommended that you crop off the bottom of the image, or else
        this function will likely return the ground directly in front of the car.

    Note:
        The larger the kernel_size, the more that the depth of each pixel is averaged
        with the distances of the surrounding pixels.  This helps reduce noise at the
        cost of reduced accuracy.
    """
    # Shift 0.0 values to 10,000 so they are not considered for the closest pixel
    depth_image = (depth_image - 0.01) % 10000

    kernel_size = 11
    blurred_image = cv.GaussianBlur(depth_image, (kernel_size, kernel_size), 0)
    minVal, maxVal, minLoc, maxLoc = cv.minMaxLoc(blurred_image)

    return np.flip(minLoc)

def remap_range(
    val: float,
    old_min: float,
    old_max: float,
    new_min: float,
    new_max: float,
) -> float:
    """
    Remaps a value from one range to another range.

    Args:
        val: A number form the old range to be rescaled.
        old_min: The inclusive 'lower' bound of the old range.
        old_max: The inclusive 'upper' bound of the old range.
        new_min: The inclusive 'lower' bound of the new range.
        new_max: The inclusive 'upper' bound of the new range.

    Note:
        min need not be less than max; flipping the direction will cause the sign of
        the mapping to flip.  val does not have to be between old_min and old_max.
    """
    '''
    old_range = old_max - old_min
    new_range = new_max - new_min
    return (((val - old_min) * new_range) / old_range) + new_min
    '''
    ratio = (val - old_min) / (old_max - old_min)
    new_range = new_max - new_min
    return (ratio * new_range) + new_min

def update_contour():
    """
    Finds contours in the current color image and uses them to update contour_center
    and contour_area
    """
    global contour_center
    global contour_area

    image = rc.camera.get_color_image()

    if image is None:
        contour_center = None
        contour_area = 0
    else:
        # Find all of the orange contours
        contours = rc_utils.find_contours(image, ORANGE[0], ORANGE[1])

        # Select the largest contour
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

def start():
    """
    This function is run once every time the start button is pressed
    """
    # Have the car begin at a stop
    rc.drive.stop()

    # Print start message
    print(">> Lab 3B - Depth Camera Cone Parking")

def crop(
    image: NDArray[(Any, ...), Any],
    top_left_inclusive: Tuple[float, float],
    bottom_right_exclusive: Tuple[float, float]
) -> NDArray[(Any, ...), Any]:
    """
    Crops an image to a rectangle based on the specified pixel points.

    Args:
        image: The color or depth image to crop.
        top_left_inclusive: The (row, column) of the top left pixel of the crop rectangle.
        bottom_right_exclusive: The (row, column) of the pixel one past the bottom right corner of the crop rectangle.

    Returns:
        A cropped version of the image.

    Note:
        The top_left_inclusive pixel is included in the crop rectangle, but the
        bottom_right_exclusive pixel is not.

        If bottom_right_exclusive exceeds the bottom or right edge of the image, the
        full image is included along that axis.
    """
    # Extract the minimum and maximum pixel rows and columns from the parameters
    r_min, c_min = top_left_inclusive
    r_max, c_max = bottom_right_exclusive

    # Shorten the array to the specified row and column ranges
    return image[r_min:r_max, c_min:c_max]

def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    # TODO: Park the car 30 cm away from the closest orange cone.
    global speed
    global angle
    global isParked

    # Search for contours in the current color image
    update_contour()

    # TODO: Park the car 30 cm away from the closest orange cone
    try:
        angle = remap_range(contour_center[1], 0, rc.camera.get_width(), -1, 1)
    except:
        print("Angle Error: setting angle to 0.3")
        angle = 0.3

    depth_image = rc.camera.get_depth_image()
    depth_image = (depth_image - 0.01) % 10000
    kernel_size = 11
    blurred_image = cv.GaussianBlur(depth_image, (kernel_size, kernel_size), 0)
    top_left_inclusive = (0, 0)
    bottom_right_exclusive = ((rc.camera.get_height() // 3) * 2, rc.camera.get_width())
    cropped_image = crop(blurred_image, top_left_inclusive, bottom_right_exclusive)
    rc.display.show_depth_image(cropped_image)
    y, x = get_closest_pixel(cropped_image)
    distance = cropped_image[y][x]
    print(distance)

    if distance < 29:
        angle *= -1
        speed = remap_range(distance, 0, 30, -1, 0)
        speed = np.clip(speed, -1, 1)
        isParked = False
        print("Backing up")
    elif distance < 33.3:
        rc.drive.stop()
        speed = 0
        angle = 0
        isParked = True
        print("parked")
    else:
        if not isParked:
            try:
                speed = remap_range(distance, 30, 500, 0, 1)
                speed = np.clip(speed, -1, 1)
            except:
                print("Speed Error: setting speed to 0.2")
                speed = 0.2
    print(speed)

    rc.drive.set_speed_angle(speed, angle)

    # Print the current speed and angle when the A button is held down
    if rc.controller.is_down(rc.controller.Button.Y):
        isParked = False
        print("not parke")
    if rc.controller.is_down(rc.controller.Button.A):
        print("Speed:", speed, "Angle:", angle)

    # Print the center and area of the largest contour when B is held down
    if rc.controller.is_down(rc.controller.Button.B):
        if contour_center is None:
            print("No contour found")
        else:
            print("Center:", contour_center, "Area:", contour_area)
    pass


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()
