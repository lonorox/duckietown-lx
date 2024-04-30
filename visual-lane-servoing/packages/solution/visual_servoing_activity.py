from typing import Tuple

import numpy as np
import cv2


def get_steer_matrix_left_lane_markings(shape: Tuple[int, int]) -> np.ndarray:
    """
    Args:
        shape:              The shape of the steer matrix.m-8:

    Return:
        steer_matrix_left:  The steering (angular rate) matrix for Braitenberg-like control
                            using the masked left lane markings (numpy.ndarray)
    """

    # TODO: implement your own solution here
    center_x = shape[1] // 2  # Calculate image center

    steer_matrix_left = np.zeros(shape=shape, dtype="float32")

    for y in range(shape[0]):
        for x in range(shape[1]):
            distance_from_center = center_x - x
            weight = max(0.0, 1 - abs(distance_from_center) / (shape[1] / 4)) 
            weight *= -1.0
            steer_matrix_left[y, x] = weight
   
    return steer_matrix_left


def get_steer_matrix_right_lane_markings(shape: Tuple[int, int]) -> np.ndarray:
    """
    Args:
        shape:               The shape of the steer matrix.

    Return:
        steer_matrix_right:  The steering (angular rate) matrix for Braitenberg-like control
                             using the masked right lane markings (numpy.ndarray)
    """

    # TODO: implement your own solution here
    center_x = shape[1] // 2  # Calculate image center

    steer_matrix_right = np.zeros(shape=shape, dtype="float32")

    for y in range(shape[0]):
        for x in range(shape[1]):
            distance_from_center = x - center_x
            weight = max(0.0, 1 - abs(distance_from_center) / (shape[1] / 4)) 
            steer_matrix_right[y, x] = weight
    return steer_matrix_right

def detect_lane_markings(image: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """
    Args:
        image: An image from the robot's camera in the BGR color space (numpy.ndarray)
    Return:
        mask_left_edge:   Masked image for the dashed-yellow line (numpy.ndarray)
        mask_right_edge:  Masked image for the solid-white line (numpy.ndarray)
    """
    h, w, _ = image.shape

    # TODO: implement your own solution here
    imgrgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

    # Convert the image to HSV for any color-based filtering
    imghsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    img = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    sigma = 1.5 # CHANGE ME

    # Smooth the image using a Gaussian kernel
    img_gaussian_filter = cv2.GaussianBlur(img,(0,0), sigma)

    # threshold = 100 # CHANGE ME
    # Convolve the image with the Sobel operator (filter) to compute the numerical derivatives in the x and y directions
    sobelx = cv2.Sobel(img_gaussian_filter,cv2.CV_64F,1,0)
    sobely = cv2.Sobel(img_gaussian_filter,cv2.CV_64F,0,1)

    # Compute the magnitude of the gradients
    Gmag = np.sqrt(sobelx*sobelx + sobely*sobely)

    mask_mag = (Gmag > 65)


    white_lower_hsv = np.array([0, 0, 122])         
    white_upper_hsv = np.array([172, 65, 255])   
    yellow_lower_hsv = np.array([14, 95, 0])        # CHANGE ME
    yellow_upper_hsv = np.array([104, 255, 255])  # CHANGE ME


    mask_white = cv2.inRange(imghsv, white_lower_hsv, white_upper_hsv)
    mask_yellow = cv2.inRange(imghsv, yellow_lower_hsv, yellow_upper_hsv)

    width = img.shape[1]
    mask_left = np.ones(sobelx.shape)
    mask_left[:,int(np.floor(width/2)):width + 1] = 0
    mask_right = np.ones(sobelx.shape)
    mask_right[:,0:int(np.floor(width/2))] = 0
    mask_sobelx_pos = (sobelx > 0)
    mask_sobelx_neg = (sobelx < 0)
    mask_sobely_pos = (sobely > 0)
    mask_sobely_neg = (sobely < 0)
    mask_left_edge = mask_left * mask_mag * mask_sobelx_neg * mask_sobely_neg * mask_yellow
    mask_right_edge =  mask_right * mask_mag * mask_sobelx_pos * mask_sobely_neg * mask_white
 

    return mask_left_edge, mask_right_edge
