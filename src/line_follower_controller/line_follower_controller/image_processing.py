import math
import cv2
import os
import numpy as np
import sys

# Convert HEX color to RGB
hex_color_of_marker = '#cde900'
marker_color = tuple(int(hex_color_of_marker[i:i+2], 16) for i in (1, 3, 5))

sys.path.insert(0, os.path.abspath(
    os.path.join(os.path.dirname(__file__), '..')))


green = (0, 255, 0)
blue = (255, 0, 0)
red = (0, 0, 255)
yellow = (255, 255, 0)

contour_line_width = 2
contour_area_line_width = 2
expected_path_line_width = 2

threshold = 80

def adjust_linear_velocity(angle_rad, min_speed, max_speed):
    """
    Adjust the linear velocity based on the given angle in radians.

    Parameters:
    - angle_rad (float): The angle in radians (expected to be between -π/2 and π/2).
    - min_speed (float): The minimum linear velocity.
    - max_speed (float): The maximum linear velocity.

    Returns:
    - float: The adjusted linear velocity.
    """
    # Ensure the angle is within the expected range
    if not (-math.pi/2 <= angle_rad <= math.pi/2):
        raise ValueError("The angle must be between -π/2 and π/2 radians.")
    
    # Calculate the absolute normalized angle to scale the speed reduction
    normalized_angle = abs(angle_rad) / (math.pi / 2)

    # Calculate the linear velocity (inverse scaling)
    linear_velocity = max_speed - (max_speed - min_speed) * normalized_angle

    return linear_velocity


def map_value(value, in_min, in_max, out_min, out_max):
    # Normalize input value
    normalized_value = (value - in_min) / (in_max - in_min)
    
    # Scale and shift to output range
    mapped_value = out_min + normalized_value * (out_max - out_min)
    
    return mapped_value


def scale_error(roi_width):
    out_max = math.radians(45)
    out_min = -out_max
    def _scale_error(error):
        in_max = (roi_width / 2)
        in_min = -in_max
        if error < in_min or error > in_max: 
            return float(0)
        return float(map_value(error, in_min, in_max, out_min, out_max))
 
    return _scale_error
 

def draw_expected_path_dots(image, have_contour):
    image_height, image_width = image.shape[:2]

    color = yellow if have_contour else red

    buffer = 20
    x_offset = image_width // 2

    y_1 = (image_height // 3 * 2) + buffer
    y_2 = (image_height // 3 * 2) + (image_height // 3) // 2
    y_3 = image_height - buffer

    # Define the gap between dots
    gap = 10

    # Draw dotted line
    for y in range(y_1, y_3 + 1, gap * 2):
        cv2.line(image, (x_offset - 70, y), (x_offset - 70,
                 y + gap), color, expected_path_line_width)
        cv2.line(image, (x_offset, y), (x_offset, y + gap),
                 color, expected_path_line_width)
        cv2.line(image, (x_offset + 70, y), (x_offset + 70,
                 y + gap), color, expected_path_line_width)
    
    return image


def get_roi_portion(image):
    image_height, _image_width = image.shape[:2]

    roi = image[(3 * image_height) // 4:, :]

    roi_height, roi_width = roi.shape[:2]

    # Calculate the width of the one-third part from the left and right sides
    one_third_width_of_roi = roi_width // 3

    roi_y = image.shape[0] - roi.shape[0]
    roi_x = one_third_width_of_roi

    # Crop the left and right one-third parts from the ROI
    roi = roi[:, one_third_width_of_roi:2 * one_third_width_of_roi]

    return roi, roi_x, roi_y

def draw_contour_and_corners(image, contour):
    green = (0, 255, 0)  # Define colors
    blue = (255, 0, 0)
    contour_area_line_width = 2

    x, y, w, h = cv2.boundingRect(contour)

    # Split contour into three equal parts along the y-axis
    sub_contours = []
    part_height = h // 3
    for i in range(3):
        start_y = y + i * part_height
        end_y = start_y + part_height
        sub_contour = contour.copy()
        sub_contour = sub_contour[(sub_contour[:, :, 1] >= start_y) & (sub_contour[:, :, 1] <= end_y)]
        sub_contours.append(sub_contour)

    # Draw contours around each sub contour with some space between them
    center_points = []
    for i, sub_contour in enumerate(sub_contours):
        x_sub, y_sub, w_sub, h_sub = cv2.boundingRect(sub_contour)
        cv2.rectangle(image, (x_sub, y_sub), (x_sub + w_sub, y_sub + h_sub), blue, contour_area_line_width)

        # Find center point of each sub contour
        center_x = x_sub + w_sub // 2
        center_y = y_sub + h_sub // 2
        center_points.append((center_x, center_y))
        # Draw a dot at the center of each sub contour
        cv2.circle(image, (center_x, center_y), 5, blue, -1)

    # Draw lines joining the centers of the sub contours
    for i in range(2):
        cv2.line(image, center_points[i], center_points[i+1], green, 2, cv2.LINE_AA)

    return image


def calculate_error(image, contour, addErrorTextOnFrame=False, scale_fn=None, error_text='Error'):
    if contour is None:
        return image

    x, y, w, h = cv2.boundingRect(contour)
    
    # Split contour into three equal parts along the y-axis
    part_height = h // 3
    
    # Calculate the bounding rectangle for the uppermost part
    start_y = y
    end_y = start_y + part_height
    upper_contour = contour[(contour[:, :, 1] >= start_y) & (contour[:, :, 1] < end_y)]
    if len(upper_contour) == 0:
        return image

    x_sub, y_sub, w_sub, h_sub = cv2.boundingRect(upper_contour)

    # Find center point of the uppermost part
    center_x = x_sub + w_sub // 2

    expected_x_coordinate = image.shape[1] // 2

    error = expected_x_coordinate - center_x

    if scale_fn is not None:
        error = scale_fn(error)

    if addErrorTextOnFrame:
        cv2.putText(image, f"{error_text}: {error}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

    return error, image


def process_image(image):

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    lower_bound = np.array([0, 0, 150])  # Lower bound for gray color in HSV
    upper_bound = np.array([180, 20, 170])  # Upper bound for gray color in HSV


    binary_image = cv2.inRange(hsv, lower_bound, upper_bound)

    # Define the structuring element (kernel)
    kernel = np.ones((5, 5), np.uint8)  # 5x5 square kernel

    # # Apply erosion
    eroded_image = cv2.erode(binary_image, kernel, iterations=5)

    # # Apply dilation
    dilated_image = cv2.dilate(eroded_image, kernel, iterations=5)

    contours, _ = cv2.findContours(
        dilated_image.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    max_contour = None

    if contours:
        # Define a function to calculate the area of a contour
        def contour_area(contour):
            x, y, w, h = cv2.boundingRect(contour)
            return w * h

        # Find the contour with the largest area
        max_contour = max(contours, key=contour_area)
    else:
        print("No contours found")
    return binary_image, max_contour


def scale_contour_from_roi_to_frame(contour, roi_x, roi_y):
    for point in contour:
        point[0][0] += roi_x
        point[0][1] += roi_y

    return contour

def process_image_and_draw_contour(image):
    roi, roi_x, roi_y = get_roi_portion(image)
    contour = process_image(roi)
    if contour is not None:
        contour = scale_contour_from_roi_to_frame(contour)
        processed_image = draw_contour_and_corners(processed_image, contour)
    # processed_image = draw_expected_path_dots(
    #     processed_image, contour is not None)
    return processed_image, roi, contour, roi_x, roi_y
