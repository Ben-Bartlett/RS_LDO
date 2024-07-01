## Other

This folder is short with 2 files that don't really fit elsewhere, but are somewhat worth including.

- advanced\_mode\_set\_params.py allows you to set advanced parameters. As you may notice it is relatively easy to set default paramters looking in the realsense-viewer. But the advanced parameters need a bit more work. You also will need to explore the repo to find the headers and what not as the python documentation and python autocomplete in things like vscode are poor due to an underpopulated stub file due to it just being a wrapper. So for example the means to set the clamping distance was found via [this](https://github.com/search?q=repo%3AIntelRealSense%2Flibrealsense%20depthclampmax&type=code) search in the realsense repo. 

- test\_recursive\_print.py just prints 3 random numbers over and over in the same command prompt lines. This was the basis of the optical flow output printing and the ascii art.

## Segmentation

There are a number of files in here with work completed at various stages on RGB image segmentation. Below it is sorted in a timeline of sorts. This material is here mostly to show what was tried, and to allow for a sample of code to pull from if needed. 

__This was completed such that the point cloud could be segmentated based on the detection of the cable in the RGB image.__

- attempt_depth+red.py is an early attempt at segmenting the image based on distance and the colour RED as the rope was RED and leads us to a potetnially simple and robust detection system. As a tracking system is based on good detections over time. This file includes:
  - HSV colour filtering for mask creation
  - A poor attempt at combining Depth and COlour Masking information
  - Shows a clipped RGB stream based on a narrow near and far clipping distance of 1 to 1.2m
  - An attempt at setting device paramters dynamically through the command prompt. 

- image\_segmentation\_test.py has a number of functionalities tested, including:
  - Graysclae Thresholding
  - Multi Otsu Thresholding
  - LAB colour space, RED thresholding
  - Canny Edge Detection

- red\_filter\_live.py and red\_filter_\bag.py show the same fuctionality for a bag file and live camera feed. As such this is a code piece of code to reference to determine how to switch between bag and live streams if needed. Another example is shown in the next bullet point. This file incudes:
  - Selecting a pixel with mouse click to save an LAB colour to a txt file.
  - Pressing p moves to the live image to show it segmented based on the selected colour

__To Note:__ this functionality is available in the __/Code/Final__ folder in [seg\_tuning\_bag.py](https://github.com/Ben-Bartlett/RS_LDO/blob/main/Code/Final/seg_tuning_bag.py) and __should__ be utilized before running any RED based segmentation as it should use a value relevant to the current illumination etc. Most of these files use similar values, but they were chosen based on live images in the testing environment. This file also shows and allows the tuning of the canny thresholds and should also be adjusted for the current test. The sliders were also used for the multi otsu thresholding but anything above 5 would crash the program. This is left as a comment in the image\_segmentation\_test.py file. 

- seg\_image\_rgb.py and seg\_image\_rgb\_depth.py show the "__finalised__" process of segmenting the images to detect the red rope. It is explained in detail at the start of seg\_image\_rgb.py, but, this file includes:
  - LAB segmentation 
  - Noisy Canny edge detection
  - Bitwise AND of both Masks for Red Edge Mask
  - Morphologcal expansion of the red edges to close them together.



## PointCloud

There are a number of files here with respect to pointcloud generation, also mostly organised into a timeline. 

- point\_cloud\_streamer.py is based on, or mostly a copy of the sample file [here](https://github.com/isl-org/Open3D/blob/main/examples/python/reconstruction_system/sensors/realsense_pcd_visualizer.py) on the Open3D repo. This file includes:
  - generating a pointcloud in Open3D with python. But it is slow.

- point\_cloud\_fast.py is my implementation of the same file, without the manual depth clipping. The clipping would be completed on device, by setting the clamping distances. This file includes:
  - just the streaming of the cameras into a pointcloud in Open3D.

- point\_cloud\_rope.py is some version of the file being made to show the point cloud of the rope, but this is a poor name choice in this case really. This file includes:
  - some issue with the Open3D environment, leading to static frames.
  - q closes the window and reopens it with the latest frame. 
  - to quit the program, ctrl+c in the terminal and then q in the Open3D window.
This realtime streaming was later solved. 

- pc\_segmented.py is our __"final"__ implementation of the segmentation to the pointcloud generation. The Red Edge Mask is used to filter the pointcloud, by setting all points outside the mask to 0. The points that are within the mask (which should only include points belonging to the rope) don't always have the correct depth values. As such we also use a depth filter to remove those points. This file includes:
  - RGB image segmentation
  - Point cloud segmentaion based on RGB mask and depth values of points (> 0.4m)
  - i.e a point cloud of "just" the rope. (more or less)


## Cable_Generation

- 3d\_cylinder\_open3d.py produces a 3d cylinder in Open3D. Self explanatory but is the basis of creating a virtual cable in the Open3D environment as this was chosen due to the link with the Realsense sensors and code base. Other choices such as Unity and ROS are also availble.

- load\_pcd.py loads a pcd into an Open3D environment. __The PCD files can be produced by looking at the code in [bag\_pc\_optical\_flow.py](https://github.com/Ben-Bartlett/RS_LDO/blob/main/Code/Development/Cable_Generation/bag_pc_optical_flow.py) as this is the only file with that functionality left in due to the numerous interations of coding and testing.__

- pcd\_and\_twin.py combines both of the above programs, opening a pcd and generating the straight cylinder. It does not offer much else but was the first step to match a cylinder to the pointcloud data. 

- pcd\_splined.py, sorts the points, decimated the pcd pointcloud, finds "average" points and fits a spline to the points with splprep from scipy. The q callback is explcilty added as I was having problems with the Open3D built in keyboard callbacks found [here](https://www.open3d.org/docs/0.12.0/tutorial/visualization/visualization.html)

- pcd_matched.py creates a cylinder between every pair of points on the spline by:
  - following the process above for steps 1-3
  1. downsampling the point cloud
  2. sorting the points left to right (may be opposite)
  3. finding average points along the area 
  4. as the cylinders are created with a center point and expanding in the + and - direction and having x points to represent the "circular" shape we:
   - create a vector from a spline point to the next
   - find the midpoint
   - create a vector from the origin to the midpoint (as we assume this point to be the "edge" of the cylinder closest the camera from a perspective point of view)
   - create a cylinder along the original direction of the point to point vector in both directions from the midpoint. 
   - we then add this to the scene. 
   - we repeat for all spline points. (this however upsets the Open3D viewer not allowing us to rotate the view at all)

  - an example of setting the view is also visible. __To get the current view in any Open3D window, press ctrl+c and paste the result somewhere__ [doc](https://www.open3d.org/docs/0.12.0/tutorial/visualization/visualization.html)

- pc\_seg\_fit\_bag.py runs the above in a loop on a bag file rather than just on a single pcd

- bag\_pc\_optical\_flow.py was the "final" version of the code demonstrated at the second last meeting and first official lab access day. This code is an optimisation of the last. This file includes:
  - The same functionality as above, from a bag file, segmentating a pointcloud based on RGB mask and depth values, decimating, averaging and creating a spline.
  - But it creates 1 mesh object of cylinders added together instead of many individual cylinders added and removed from the scene on each iteration. This allows the view to be rotated and the frame rate to remain at around 30fps for this visualisation with around 100 spline points. 
  - It is however a bit noisy and the bag file used was not the "best" example. 


  
