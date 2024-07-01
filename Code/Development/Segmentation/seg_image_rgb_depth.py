import pyrealsense2 as rs
import numpy as np
import cv2

# Global variables for selected color in Lab space
selected_color_lab = np.array([74, 176, 163])
tolerance = 40  # Adjust this tolerance value as needed

# Initialize Canny edge thresholds
canny_threshold1 = 10
canny_threshold2 = 50

# Define kernel for morphological operations
kernel_size = 5
kernel = np.ones((kernel_size, kernel_size), np.uint8)

def initialize_camera():
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 30)  # Enable depth stream
    pipeline.start(config)
    align = rs.align(rs.stream.color)
    return pipeline, align

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

def edge_segmentation(frame):
    global canny_threshold1, canny_threshold2

    # Convert frame to grayscale
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Apply Gaussian blurring to reduce noise
    blurred_frame = cv2.GaussianBlur(gray_frame, (5, 5), 0)

    # Apply Canny edge detection
    edges = cv2.Canny(blurred_frame, canny_threshold1, canny_threshold2)
    
    return edges

def overlay_mask_on_image(image, mask):
    # Create a copy of the original image
    overlay = image.copy()

    # Generate a binary mask for the object
    object_mask = cv2.merge((mask, mask, mask))

    # Set the unmasked area to black in the original image
    overlay_masked = cv2.bitwise_and(overlay, object_mask)

    return overlay_masked

def run_application():
    # Initialize camera and align
    pipeline, align = initialize_camera()

    while True:
        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()

        if not color_frame or not depth_frame:
            continue

        # Convert to numpy array
        frame = np.asanyarray(color_frame.get_data())

        # Perform segmentation
        red_mask = filter_red_colors(frame)
        edge_mask = edge_segmentation(frame)
        enhanced_mask = enhance_mask_with_edges(red_mask, edge_mask)

        # Overlay combined mask on the original image
        overlay_image = overlay_mask_on_image(frame, enhanced_mask)

        # Overlay depth information on the masked area
        depth_image = np.asanyarray(depth_frame.get_data())
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        depth_overlay = cv2.bitwise_and(depth_colormap, depth_colormap, mask=enhanced_mask)

        # Combine the masked RGB image and depth overlay horizontally
        combined_image = np.hstack((overlay_image, depth_overlay))

        # Display the combined image
        cv2.imshow('Combined RGB and Depth Overlay', combined_image)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q') or key == 27:
            break

    cv2.destroyAllWindows()
    pipeline.stop()

if __name__ == "__main__":
    run_application()
