import pyrealsense2 as rs
import numpy as np
import cv2
import open3d as o3d

def main():
    # Create a pipeline
    pipeline = rs.pipeline()

    # Create a config and configure the pipeline to stream
    config = rs.config()

    # Get device product line for setting a supporting resolution 
    # (not strictly neccessary as we know what camera we have, we arent explicitly dealing for mulitple cameras at once either)
    pipeline_wrapper = rs.pipeline_wrapper(pipeline) #<pyrealsense2.pyrealsense2.pipeline_wrapper object at 0x7f854143cdf8>
    pipeline_profile = config.resolve(pipeline_wrapper) #<pyrealsense2.pyrealsense2.pipeline_profile object at 0x7f84b8e80b90>
    device = pipeline_profile.get_device() #<pyrealsense2.device: Intel RealSense D435 (S/N: 147122071917  FW: 05.16.00.01  on USB3.2)>
    device_product_line = str(device.get_info(rs.camera_info.product_line)) #D400
    

    found_rgb = False
    for s in device.sensors:
        #<pyrealsense2.sensor: "Stereo Module">
        #<pyrealsense2.sensor: "RGB Camera">
        if s.get_info(rs.camera_info.name) == 'RGB Camera':
            found_rgb = True
            print("connecting to camera")
            break
    if not found_rgb:
        print("This program requires Depth camera with Color sensor")
        exit(0)

    # different resolutions of color and depth streams after having checked it is possible with the camera 
    # not set to the maximum here but will suffice, need to be the same for ease of alignement
    # maximums shown here  
    #config.enable_stream(rs.stream.color, width=1280, height=720, format=rs.format.rgb8, framerate=30)
    #config.enable_stream(rs.stream.depth, width=848, height=480, format=rs.format.z16, framerate=30)
    #config.enable_stream(rs.stream.infrared, stream_index=1, width=848, height=480, format=rs.format.y8, framerate=30)
    #config.enable_stream(rs.stream.infrared, stream_index=2, width=848, height=480, format=rs.format.y8, framerate=30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)


    # Start streaming
    profile = pipeline.start(config)

    # Create an align object
    align_to = rs.stream.color
    align = rs.align(align_to)

    # Create a pointcloud object
    pc = rs.pointcloud()
    # Create a frame object
    points = rs.points()

    # Create an Open3D visualizer
    #vis = o3d.visualization.Visualizer()
    #vis.create_window()

    # Create an empty point cloud
    #pcd = o3d.geometry.PointCloud()
    #vis.add_geometry(pcd)

    # Set the initial view parameters
    #view_ctl = vis.get_view_control()
    #view_ctl.set_front([0.71077, -0.27436, -0.647718])
    #view_ctl.set_lookat([0.07954, 0.014111, 0.21693])
    #view_ctl.set_up([-0.19595, -0.9615816, 0.192254])
    #view_ctl.set_zoom(0.6499)


    try:
        while True:
            # Wait for a coherent pair of frames: depth and color
            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)

            # Get aligned frames
            aligned_depth_frame = aligned_frames.get_depth_frame()  # Aligned depth frame is a 640x480 depth image
            color_frame = aligned_frames.get_color_frame()

            # Validate that both frames are valid
            if not aligned_depth_frame or not color_frame:
                continue

            depth_image = np.asanyarray(aligned_depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())

            # Convert RGB to BGR for OpenCV
            color_image_bgr = cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR)

            # Map the depth frame to the color frame
            pc.map_to(color_frame)
            points = pc.calculate(aligned_depth_frame)

            # Convert the points to numpy arrays
            v, t = points.get_vertices(), points.get_texture_coordinates()
            verts = np.asanyarray(v).view(np.float32).reshape(-1, 3)  # xyz
            texcoords = np.asanyarray(t).view(np.float32).reshape(-1, 2)  # uv
            
            
            # Create an Open3D point cloud
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(verts)
            pcd.colors = o3d.utility.Vector3dVector(color_image_bgr.reshape(-1, 3) / 255.0)

            # Visualize the point cloud
            # when open press ctrl-c to copy json of the configuration of the space, including the view
            o3d.visualization.draw_geometries([pcd], zoom=0.6499,front=[0.71077,-0.27436,-0.647718], lookat=[0.07954,0.014111,0.21693], up=[-0.19595,-0.9615816,0.192254])

            # Update the visualizer
            #vis.update_geometry(pcd)
            #vis.poll_events()
            #vis.update_renderer()
            #print("vis")
            
            
    finally:
        pipeline.stop()

if __name__ == "__main__":
    main()
