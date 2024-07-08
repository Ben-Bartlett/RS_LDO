## seg\_tuning\_bag.py

This file should be run first. It allows you to adjust the parameters for the LAB colour segmentation and to adjust Canny thresholds. 

To be honest for speed and simplification, it may be best to remove all Canny Detection and just used LAB red colour segementation. 


## final.py

This file is the current final draft of the project.

It includes:
1. Options for Live or Bag playback
 - Bag is the default option and enabled now
 - The bag chooser expects RS_LDO/Data to have the bag files
 - It is robust enough that it does not depend on the folder you run the script in (e.g root or this folder)
 - The fps is in the bagfiles names in my case (before hz) and is automatically pulled and used
 - If live the user is prompted to select 30 or 60 fps

2. Timing functions
 - One for the frame rate
 - One for the loop rate
 - One for the decorated functions time
 - Put @timer before any function you want to add to this counter
 - The idea was to see how fast or slow "my functions" were.
 - No @timers are currently added

3. Memory Profiling
 - This can be enabled by uncommenting the @profile before main()

4. Functionalised Code
 - The main is broken down to core steps such that the functions can be edited
 - This is 50/50 as to whether or not it makes the code more readable. 

5. Processing Steps
 - RGB
 - Depth
 - PointCloud Creation
 - PointCloud Processing

6. Toggle on/off vis
 - With two flags at the top you can enable cv2 and o3d vis.
 - Within o3d vis you can toggle all parts of the vis
   - m for main pcd
   - d for decimated pcd (it doesnt look massively decimated with the settings used)
   - a for shifted average points GREEN
   - p for spline BLUE
   - c for cylinder mesh or virtual rope




__NOTES & TO DO:__

1. Pull out paramters for ease of tuning and testing
  - e.g bin size, spline points, voxel_size, kernel size, etc
2. Simplify the RGB masking
  - While more robust to noise, people etc, it has its own flaws
4. See if the PCA algorithm is robust to multiple changes in direction
5. Try to develop a CUDA based approach to threshold the image
6. GPU vector functions are definitly possible on the spline generation and point shifting etc
7. A validation of the spline needs to take place, to see how close of an estimate it is, and tune the paramters
8. Use a different library to Open3d for the pointcloud generation and manipulation as it is not really designed to be a realtime library and the ask is 60Hz


