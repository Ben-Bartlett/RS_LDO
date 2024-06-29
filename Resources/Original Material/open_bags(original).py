import pyrealsense2 as rs
# import matplotlib.pyplot as plt
import numpy as np
import cv2 as cv


### FOR BAG ------------------
# Setup:
pipe = rs.pipeline()
cfg = rs.config()
cfg.enable_device_from_file("cable.bag")
profile = pipe.start(cfg)

#---------------------------------

# Getting the depth sensor's depth scale (see rs-align example for explanation)
depth_scale = profile.get_device().first_depth_sensor().get_depth_scale() # Get data scale from the device and convert to meters

#to align depth to color frame
align_to = rs.stream.color
align = rs.align(align_to)

for i in range(100): ##initialisation
    frames = pipe.wait_for_frames()

## get caemra intrinsics
aligned_frames = align.process(frames)
intrinsics = aligned_frames.get_depth_frame().profile.as_video_stream_profile().intrinsics

try: 
    while True:
        frames = pipe.wait_for_frames()                 #gets surrent frames
        aligned_frames = align.process(frames)
        
        color_frame = aligned_frames.get_color_frame()  #align color and depth
        depth_frame = aligned_frames.get_depth_frame()

        cv.imshow('Color frame', np.asanyarray(color_frame.get_data()))
        cv.imshow("Depth frame", np.asanyarray(rs.colorizer().colorize(depth_frame).get_data()))
        key = cv.waitKey(1)

except KeyboardInterrupt:
    pass

cv.destroyAllWindows()
pipe.stop()
