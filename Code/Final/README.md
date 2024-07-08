## seg\_tuning\_bag.py

This file should be run first. It allows you to adjust the parameters for the LAB colour segmentation and to adjust Canny thresholds. 

To be honest for speed and simplification, it may be best to remove all Canny Detection and just used LAB red colour segementation. 


## final.py

This file is the current final draft of the project.

It includes:
1. Options for Live or Bag playback
  1. Bag is the default option and enabled now
  2. The bag chooser expects RS_LDO/Data to have the bag files
  3. It is robust enough that it does not depend on the folder you run the script in (e.g root or this folder)
  4. The fps is in the bagfiles names in my case (before hz) and is automatically pulled and used
  5. If live the uder is prompted to select 30 or 60 fps

2. Timing functions
  1. One for the frame rate
  2. One for the loop rate
  3. One for the decorated functions time
   1. Put @timer before any function you want to add to this counter
   2. The idea was to see how fast or slow "my functions" were. 
   3. No @timers are currently added

3. Memory Profiling
  1. This can be enabled by uncommenting the @profile before main()

4. Functionalised Code
  1. The main is broken down to core steps such that the functions can be edited 
  2. This is 50/50 as to whether or not it makes the code more readable. 

5. Processing Steps
  1. RGB
  2. Depth
  3. PointCloud Creation
  4. PointCloud Processing

6. Toggle on/off vis
  1. With two flags at the top you can enable cv2 and o3d vis. 
  2. Within o3d vis you can toggle all parts of the vis
   1. m for main pcd
   2. d for decimated pcd (it doesnt look massively decimated with the settings used)
   3. a for shifted average points GREEN
   4. p for spline BLUE
   5. c for cylinder mesh or virtual rope




__NOTES & TO DO:__

1. Pull out paramters for ease of tuning and testing
  1. e.g bin size, spline points, voxel_size, kernel size, etc
2. Simplify the RGB masking
  2. While more robust to noise, people etc, it has its own flaws
3. See if the PCA algorithm is robust to multiple changes in direction
4. Try to develop a CUDA based approach to threshold the image
5. GPU vector functions are definitly possible on the spline generation and point shifting etc
6. A validation of the spline needs to take place, to see how close of an estimate it is, and tune the paramters
7. Use a different library to Open3d for the pointcloud generation and manipulation as it is not really designed to be a realtime library and the ask is 60Hz


