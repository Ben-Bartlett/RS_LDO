import pyrealsense2 as rs
import numpy as np
import cv2
from skimage.filters import threshold_multiotsu

# Global variables for selected color in Lab space
selected_color_lab = np.array([81, 181, 170])
tolerance = 40  # Adjust this tolerance value as needed

# Initialize Canny edge thresholds
canny_threshold1 = 50
canny_threshold2 = 150

def initialize_camera():
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 30)
    pipeline.start(config)
    return pipeline

def filter_red_colors(frame):
    global selected_color_lab, tolerance

    # Convert frame to Lab
    lab_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2Lab)

    # Define range around selected color in Lab space
    lower_red = np.array([selected_color_lab[0] - tolerance, selected_color_lab[1] - 20, selected_color_lab[2] - 20])
    upper_red = np.array([selected_color_lab[0] + tolerance, selected_color_lab[1] + 20, selected_color_lab[2] + 20])

    # Create mask for red colors in Lab space
    red_mask = cv2.inRange(lab_frame, lower_red, upper_red)

    # Apply morphological operations to clean up the mask
    kernel = np.ones((5, 5), np.uint8)
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel)

    return red_mask

def threshold_segmentation(frame):
    # Convert frame to grayscale
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Apply simple thresholding
    #_, thresh_mask = cv2.threshold(gray_frame, 100, 255, cv2.THRESH_BINARY)
    blur = cv2.GaussianBlur(gray_frame,(5,5),0)
    _,thresh_mask = cv2.threshold(blur,0,255,cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)

    return thresh_mask

def multiotsu_threshold_segmentation(frame, num_regions=4): # increasing this too much will crash python
    # Convert frame to grayscale
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Applying multi-Otsu thresholding
    thresholds = threshold_multiotsu(gray_frame, classes=num_regions)

    # Using the threshold values, we generate the regions
    regions = np.digitize(gray_frame, bins=thresholds)

    # Inverse the segmentation
    inverse_regions = np.ones_like(gray_frame) * 255 - regions.astype(np.uint8) * (255 // num_regions)

    return regions.astype(np.uint8) * (255 // num_regions), inverse_regions


def edge_segmentation(frame):
    # Convert frame to grayscale
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Apply Canny edge detection
    edges = cv2.Canny(gray_frame, 50, 150)

    return edges

def run_application():
    # Initialize camera
    pipeline = initialize_camera()

    while True:
        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()

        if not color_frame:
            continue

        # Convert to numpy array
        frame = np.asanyarray(color_frame.get_data())

        # Perform different segmentations
        red_mask = filter_red_colors(frame)
        thresh_mask = threshold_segmentation(frame)
        multiotsu_mask, inverse_multiotsu_mask = multiotsu_threshold_segmentation(frame, num_regions=3)
        edge_mask = edge_segmentation(frame)

        # Display the results in separate windows
        cv2.imshow('Original Frame', frame)
        cv2.imshow('Red Mask (Lab)', red_mask)
        cv2.imshow('Threshold Mask', thresh_mask)
        cv2.imshow('Multi-Otsu Threshold Mask', inverse_multiotsu_mask)
        cv2.imshow('Edge Mask (Canny)', edge_mask)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q') or key == 27:
            break

    cv2.destroyAllWindows()
    pipeline.stop()

if __name__ == "__main__":
    run_application()
