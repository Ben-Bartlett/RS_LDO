import pyrealsense2 as rs
import numpy as np
import cv2
import open3d as o3d
from enum import IntEnum
from datetime import datetime
from os.path import abspath
import sys

sys.path.append(abspath(__file__))

quit = False

class Preset(IntEnum):
    Custom = 0
    Default = 1
    Hand = 2
    HighAccuracy = 3
    HighDensity = 4
    MediumDensity = 5


def get_intrinsic_matrix(frame):
    intrinsics = frame.profile.as_video_stream_profile().intrinsics
    return o3d.camera.PinholeCameraIntrinsic(640, 480, intrinsics.fx,
                                             intrinsics.fy, intrinsics.ppx,
                                             intrinsics.ppy)


def initialize_realsense_pipeline():
    # Create a pipeline
    pipeline = rs.pipeline()

    # Create a config and configure the pipeline to stream color and depth
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Start streaming
    profile = pipeline.start(config)
    depth_sensor = profile.get_device().first_depth_sensor()

    # Using preset Default for recording
    depth_sensor.set_option(rs.option.visual_preset, Preset.Default)

    return pipeline, depth_sensor, profile


def enable_advanced_mode(device):
    advanced_mode = rs.rs400_advanced_mode(device)

    # Enable advanced mode if not already enabled
    if not advanced_mode.is_enabled():
        advanced_mode.toggle_advanced_mode(True)

    return advanced_mode


def set_depth_table(advanced_mode, depth_clamp_min=0, depth_clamp_max=10000):
    # Retrieve and modify the depth table control settings
    depth_table_control = advanced_mode.get_depth_table()
    depth_table_control.depthClampMin = depth_clamp_min
    depth_table_control.depthClampMax = depth_clamp_max

    # Set the updated depth table control settings back to the camera
    advanced_mode.set_depth_table(depth_table_control)

    print("depthClampMin has been set to:", depth_table_control.depthClampMin)
    print("depthClampMax has been set to:", depth_table_control.depthClampMax)


def main():
    global quit

    # Initialize the RealSense pipeline
    pipeline, depth_sensor, profile = initialize_realsense_pipeline()

    # Initialize advanced mode for the device
    context = rs.context()
    device = context.query_devices()[0]
    advanced_mode = enable_advanced_mode(device)

    # Set depth clamp values
    set_depth_table(advanced_mode)

    # Create an align object to align depth frames to color frames
    align_to = rs.stream.color
    align = rs.align(align_to)

    # Initialize Open3D visualizer
    vis = o3d.visualization.VisualizerWithKeyCallback()
    vis.create_window(width=600,height=400)

    pcd = o3d.geometry.PointCloud()
    flip_transform = [[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]]

    
    # q should quit but it isnt so this slightly poor work around to stop the loop with
    # the listener for a custom key is completed rather than adding keyboard or some other library
    # press h for a full list of the keyboard commands that are meant to be implemented
    def key_callback(vis):
        global quit
        print("q")
        quit = True


    #Q using GLFW_KEYs
    vis.register_key_callback(81, key_callback)

    # Streaming loop
    frame_count = 0
    
    try:
        while quit == False:
            dt0 = datetime.now()

            # Get frameset of color and depth
            frames = pipeline.wait_for_frames()

            # Align the depth frame to color frame
            aligned_frames = align.process(frames)

            # Get aligned frames
            aligned_depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            intrinsic = get_intrinsic_matrix(color_frame)

            # Validate that both frames are valid
            if not aligned_depth_frame or not color_frame:
                continue

            depth_image = o3d.geometry.Image(np.array(aligned_depth_frame.get_data()))
            color_temp = np.asarray(color_frame.get_data())
            color_temp = cv2.cvtColor(color_temp, cv2.COLOR_RGB2BGR)
            color_image = o3d.geometry.Image(color_temp)

            rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
                color_image,
                depth_image,
                depth_scale=1.0 / depth_sensor.get_depth_scale(),
                convert_rgb_to_intensity=False
            )
            temp = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, intrinsic)
            temp.transform(flip_transform)
            pcd.points = temp.points
            pcd.colors = temp.colors

            if frame_count == 0:
                vis.add_geometry(pcd)

            vis.update_geometry(pcd)
            vis.poll_events()
            vis.update_renderer()

            process_time = datetime.now() - dt0
            print("\rFPS: " + str(1 / process_time.total_seconds()), end='')
            frame_count += 1



    finally:
        pipeline.stop()
        vis.destroy_window()


if __name__ == "__main__":
    main()
