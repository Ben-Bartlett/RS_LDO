import pyrealsense2 as rs
import numpy as np
import cv2

# Global variables for selected color in Lab space
selected_color_lab = np.array([74, 176, 163])
tolerance = 40  # Adjust this tolerance value as needed

# Initialize Canny edge thresholds
canny_threshold1 = 70
canny_threshold2 = 90

def initialize_camera():
    pipeline = rs.pipeline()
    config = rs.config()
    print("adding bagfile stream")
    rs.config.enable_device_from_file(config, "bags/bag1.bag")  # Replace with your bag file path
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)
    return pipeline

def select_red_color(event, x, y, flags, param):
    global selected_color_lab
    if event == cv2.EVENT_LBUTTONDOWN:
        frame, = param  # Unpack the frame from param tuple
        selected_color = frame[y, x]
        selected_color_lab = cv2.cvtColor(np.uint8([[selected_color]]), cv2.COLOR_BGR2Lab)[0][0]
        # Save selected color in Lab space to a text file
        with open('selected_color_lab.txt', 'w') as f:
            f.write(','.join(map(str, selected_color_lab)))

def filter_red_colors(lab_frame, selected_color_lab, tolerance):
    # Define range around selected color in Lab space
    lower_red = np.array([selected_color_lab[0] - tolerance, selected_color_lab[1] - 20, selected_color_lab[2] - 20])
    upper_red = np.array([selected_color_lab[0] + tolerance, selected_color_lab[1] + 20, selected_color_lab[2] + 20])

    # Create mask for red colors in Lab space
    red_mask = cv2.inRange(lab_frame, lower_red, upper_red)

    # Apply morphological operations to clean up the mask
    kernel = np.ones((5, 5), np.uint8)
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel)

    return red_mask


def edge_segmentation(frame):
    global canny_threshold1, canny_threshold2

    # Convert frame to grayscale
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Apply Gaussian blurring to reduce noise
    blurred_frame = cv2.GaussianBlur(gray_frame, (5, 5), 0)

    # Apply Canny edge detection
    edges = cv2.Canny(blurred_frame, canny_threshold1, canny_threshold2)
    
    thickening_kernel_size = 3
    thickening_kernel = np.ones((thickening_kernel_size, thickening_kernel_size), np.uint8)

    # Dilate to make the edges slightly thicker
    dilated_edges = cv2.dilate(edges, thickening_kernel, iterations=1)


    return dilated_edges

def overlay_mask_on_image(image, mask):
    # Create a copy of the original image
    overlay = image.copy()

    # Generate a binary mask for the object
    object_mask = cv2.merge((mask, mask, mask))

    # Add the object mask to the overlay
    overlay_masked = cv2.addWeighted(overlay, 1, object_mask, 0.5, 0)

    return overlay_masked

def on_trackbar(val):
    global canny_threshold1, canny_threshold2
    canny_threshold1 = cv2.getTrackbarPos('Min Threshold', 'Edge Mask (Canny)')
    canny_threshold2 = cv2.getTrackbarPos('Max Threshold', 'Edge Mask (Canny)')

def run_application():
    # Initialize camera
    pipeline = initialize_camera()

    # Create windows
    cv2.namedWindow('Original Frame')
    cv2.namedWindow('Edge Mask (Canny)')

    # Create trackbars for Canny edge detection
    cv2.createTrackbar('Min Threshold', 'Edge Mask (Canny)', canny_threshold1, 255, on_trackbar)
    cv2.createTrackbar('Max Threshold', 'Edge Mask (Canny)', canny_threshold2, 255, on_trackbar)

    while True:
        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()

        if not color_frame:
            continue

        # Convert to numpy array
        frame = np.asanyarray(color_frame.get_data())

        cv2.imshow('Select Red Color', frame)
        
        # Set mouse callback with parameter tuple (frame,)
        cv2.setMouseCallback('Select Red Color', select_red_color, (frame,))

      

        # Convert to numpy array
        frame = np.asanyarray(color_frame.get_data())

        # Convert frame to Lab
        lab_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2Lab)

        # Filter red colors in Lab space
        red_mask = filter_red_colors(lab_frame, selected_color_lab, tolerance)

        # Apply the mask to the frame
        filtered_frame = cv2.bitwise_and(frame, frame, mask=red_mask)

        # Display the filtered frame
        cv2.imshow('Pinky Red Rope Mask', filtered_frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q') or key == 27:
            break

        # Perform segmentation
        edge_mask = edge_segmentation(frame)


        # Display the results in separate windows
        cv2.imshow('Original Frame', frame)
        cv2.imshow('Edge Mask (Canny)', edge_mask)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q') or key == 27:
            break

    cv2.destroyAllWindows()
    pipeline.stop()

if __name__ == "__main__":
    run_application()
