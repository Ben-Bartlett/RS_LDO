'''
In essence we have created a very robust image segmentor for the red rope
We get all "Red pixels" within a tolerence range in LAB colour space
We segment based on this, the reason we didnt stop here is the edge would not 
detect well with just LAB such that the region is underrepresntative of the rope
We segment the image with a Canny edge detector with a very low threshold such that it gives very good edges 
but is noisy. 
We bitwise AND these together such that the result is RED EDGES
We then apply morphological closing with a large kernel to close the edeges, encompassing the rope 
Gives quite a robust detection of the rope even with somewhat fast movement for the frame rate
This is somewhat senisitive to lighting variations though such that
preferably we have uniform lighting and an uncluttered background, without red objects. (not too bad limitations)
'''

import pyrealsense2 as rs
import numpy as np
import cv2

# Global variables for selected color in Lab space
# Used another program to probe the image to get this value
selected_color_lab = np.array([74, 176, 163])
tolerance = 40  # Adjust this tolerance value as needed

# Initialize Canny edge thresholds
canny_threshold1 = 0
canny_threshold2 = 25

# Define kernel for morphological operations
kernel_size = 5
kernel = np.ones((kernel_size, kernel_size), np.uint8)

def initialize_camera():
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 30)
    pipeline.start(config)
    return pipeline

def filter_red_colors(frame):
    global selected_color_lab, tolerance, kernel

    # Convert frame to Lab
    lab_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2Lab)

    # Define range around selected color in Lab space
    lower_red = np.array([selected_color_lab[0] - tolerance, selected_color_lab[1] - 20, selected_color_lab[2] - 20])
    upper_red = np.array([selected_color_lab[0] + tolerance, selected_color_lab[1] + 20, selected_color_lab[2] + 20])

    # Create mask for red colors in Lab space
    red_mask = cv2.inRange(lab_frame, lower_red, upper_red)

    return red_mask

def edge_segmentation(frame):
    global canny_threshold1, canny_threshold2

    # Convert frame to grayscale
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Apply Gaussian blurring to reduce noise
    blurred_frame = cv2.GaussianBlur(gray_frame, (5, 5), 0)

    # Apply Canny edge detection
    edges = cv2.Canny(blurred_frame, canny_threshold1, canny_threshold2)
    
    return edges

def enhance_mask_with_edges(red_mask, edges):
    global kernel

    # Dilate the red mask until it reaches the edges
    dilated_red_mask = cv2.dilate(red_mask, kernel, iterations=1)

    # Combine the dilated red mask with the edges using bitwise AND
    enhanced_mask = cv2.bitwise_and(dilated_red_mask, edges)

    # Apply morphological closing to refine the mask
    closing_kernel_size = 37
    closing_kernel = np.ones((closing_kernel_size, closing_kernel_size), np.uint8)
    enhanced_mask = cv2.morphologyEx(enhanced_mask, cv2.MORPH_CLOSE, closing_kernel)

    return enhanced_mask


def overlay_mask_on_image(image, mask):
    # Create a copy of the original image
    overlay = image.copy()

    # Generate a binary mask for the object
    object_mask = cv2.merge((mask, mask, mask))

    # Set the unmasked area to black in the original image
    overlay_masked = cv2.bitwise_and(overlay, object_mask)

    return overlay_masked


def run_application():
    # Initialize camera
    pipeline = initialize_camera()

    # Create windows
    cv2.namedWindow('Overlay Mask')

    while True:
        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()

        if not color_frame:
            continue

        # Convert to numpy array
        frame = np.asanyarray(color_frame.get_data())

        # Perform segmentation
        red_mask = filter_red_colors(frame)
        edge_mask = edge_segmentation(frame)
        enhanced_mask = enhance_mask_with_edges(red_mask, edge_mask)

        # Overlay combined mask on the original image
        overlay_image = overlay_mask_on_image(frame, enhanced_mask)

        # Display the results in separate windows
        cv2.imshow('Overlay Mask', overlay_image)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q') or key == 27:
            break

    cv2.destroyAllWindows()
    pipeline.stop()

if __name__ == "__main__":
    run_application()

