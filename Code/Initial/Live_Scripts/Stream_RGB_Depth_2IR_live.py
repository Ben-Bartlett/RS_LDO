import pyrealsense2 as rs
import numpy as np
import cv2

def main():
    try:
        # Initialize context and check for devices
        ctx = rs.context()
        # Get a snapshot of currently connected devices
        devices = ctx.query_devices()

        if len(devices) == 0:
            print("No device detected. Is it plugged in?")
        else:
            for i, d in enumerate(devices):
                print(f"Device :{i + 1}")
                for attr in rs.camera_info.__members__.items():
                    try:
                        info = d.get_info(attr[1])
                        print(f"{attr[0].replace('_', ' ').title()} : {info}")
                    except RuntimeError:
                        continue


        


        # Start pipeline
        pipeline = rs.pipeline()
        config = rs.config()

        # Enable streams (Color, Depth, Infrared)
        serial_number = devices[0].get_info(rs.camera_info.serial_number)
        config.enable_device(serial_number)
        config.enable_stream(rs.stream.color, width=1280, height=720, format=rs.format.rgb8, framerate=30)
        config.enable_stream(rs.stream.depth, width=848, height=480, format=rs.format.z16, framerate=30)
        config.enable_stream(rs.stream.infrared, stream_index=1, width=848, height=480, format=rs.format.y8, framerate=30)
        config.enable_stream(rs.stream.infrared, stream_index=2, width=848, height=480, format=rs.format.y8, framerate=30)

        # Start pipeline with configuration
        profile = pipeline.start(config)

        # Get depth sensor scale
        depth_sensor = profile.get_device().first_depth_sensor()
        scale = depth_sensor.get_depth_scale()
        print(f"Depth scale: {scale}")

        # Get video stream intrinsics and extrinsics
        depth_stream = profile.get_stream(rs.stream.depth)
        color_stream = profile.get_stream(rs.stream.color)
        e = depth_stream.get_extrinsics_to(color_stream)
        print(f"Extrinsics: {e}")

        # Get field of view
        depth_stream_profile = depth_stream.as_video_stream_profile()
        intrinsics = depth_stream_profile.get_intrinsics()
        fov = rs.rs2_fov(intrinsics)
        print(f"Field of view (degrees): {fov}")

        # Control laser and emitter
        if depth_sensor.supports(rs.option.emitter_enabled):
            depth_sensor.set_option(rs.option.emitter_enabled, 1)

        if depth_sensor.supports(rs.option.laser_power):
            range = depth_sensor.get_option_range(rs.option.laser_power)
            #depth_sensor.set_option(rs.option.laser_power, range.max)  # Set max power
            depth_sensor.set_option(rs.option.laser_power, range.default)  # Disable laser

        try:
            while True:
                # Wait for a coherent pair of frames: depth and color
                frames = pipeline.wait_for_frames(timeout_ms=10000)  # Increased timeout to 10 seconds
                depth_frame = frames.get_depth_frame()
                color_frame = frames.get_color_frame()
                infrared_frame1 = frames.get_infrared_frame(1)
                infrared_frame2 = frames.get_infrared_frame(2)

                if not depth_frame or not color_frame or not infrared_frame1 or not infrared_frame2:
                    continue

                # Convert images to numpy arrays
                depth_image = np.asanyarray(depth_frame.get_data())
                color_image = np.asanyarray(color_frame.get_data())
                infrared_image1 = np.asanyarray(infrared_frame1.get_data())
                infrared_image2 = np.asanyarray(infrared_frame2.get_data())

                # Convert RGB to BGR for OpenCV
                color_image_bgr = cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR)

                # Apply colormap on depth image
                depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

                # Display color image in its own window
                cv2.imshow('Color Stream', color_image_bgr)

                # Display depth image in its own window
                cv2.imshow('Depth Stream', depth_colormap)

                # Display infrared streams in their own windows
                cv2.imshow('Infrared Stream 1', infrared_image1)
                cv2.imshow('Infrared Stream 2', infrared_image2)

                # Wait for a key press to exit
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        finally:
            pipeline.stop()

    except Exception as e:
        print(f"An error occurred: {e}")

if __name__ == "__main__":
    main()
