import pyrealsense2 as rs
import numpy as np
import cv2

# Global variables for selected color and tolerance
selected_color_lab = None
tolerance = 60  # Adjust this tolerance value as needed

def select_red_color(event, x, y, flags, param):
    global selected_color_lab
    if event == cv2.EVENT_LBUTTONDOWN:
        frame, = param  # Unpack the frame from param tuple
        selected_color = frame[y, x]
        selected_color_lab = cv2.cvtColor(np.uint8([[selected_color]]), cv2.COLOR_BGR2Lab)[0][0]
        # Save selected color in Lab space to a text file
        with open('selected_color_lab.txt', 'w') as f:
            f.write(','.join(map(str, selected_color_lab)))

def initialize_camera():
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 30)
    pipeline.start(config)
    return pipeline

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

def load_selected_color():
    global selected_color_lab
    try:
        with open('selected_color_lab.txt', 'r') as f:
            data = f.readline().strip()
            selected_color_lab = list(map(int, data.split(',')))
            print(f"Loaded selected color: {selected_color_lab}")
    except FileNotFoundError:
        print("No saved color found. Please select a color by clicking on 'Select Red Color' window and press 'p'.")

def run_application():
    global selected_color_lab

    # Initialize camera
    pipeline = initialize_camera()

    cv2.namedWindow('Select Red Color')

    # Load selected color from file
    load_selected_color()

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

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q') or key == 27:
            break
        elif key == ord('p') and selected_color_lab is not None:
            while True:
                # Wait for a coherent pair of frames: depth and color
                frames = pipeline.wait_for_frames()
                color_frame = frames.get_color_frame()

                if not color_frame:
                    continue

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

    cv2.destroyAllWindows()
    pipeline.stop()

if __name__ == "__main__":
    run_application()

