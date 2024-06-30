## Bag_Scripts
These files are mostly following the examples found in the realsense repo [here](https://github.com/IntelRealSense/librealsense/tree/master/wrappers/python/examples) and adapting the code to work with the bad files originally made available to me. 

- First the bag file viewer given with the bag files found [here]() was adjusted to create Bag\_Viewer.py, which have the ability to dynamically select a bag file from the subfolder __/bag_files__. Download these first or add some here to utilize the scripts without editing the code. 
-  There is a version of the Tutorial 1 example (ASCII\_ART.py) generating the ASCII art in terminal with print statements. This was very poor originally and somewhat pointless to work on, but given the work completed on recursively displaying strings in the terminal in the final scripts, it has been updated to display recursively in the terminal and looks good. But is still pretty pointless
-  Depth\_and\_RGB.py is a slightly improved version of Bag\_Viewer.py with Depth and RGB in one frame based on Tutorial 2.
- Aligning\_Depth\_and\_RGB.py is based on Tutorial 3 and does background removal based on depth information, segmenting the combined stream in a simple manner. 

**To Note:** I have no idea what settings, or in what way, the bag files were recorded for these tests, and as such cannot guarantee all required info is there or the stream resolutions/fps.


## Live_Scripts
The only script here for the moment is the script where the RGB, Depth and 2 IR streams are streamed as this was the first "development" when I recieved the camera itself to work on. 

Run this script with the camera connected to verify everything is installed correctly and the camera is working. 

This script also grabs a number of camera parameters and sets the laser to default somewhat following Tutorial 4. 

**To Note:** You must use a USB 3.0 cable with the camera. I could not get a stable or any connection at times with a 2.0 cable. I at least had one but it was a bit short. USB cable extender as such may not be useable.

  ```bash
  python Live_Scripts/Stream_RGB_Depth_2IR_live.py





Tutorial 5-9 found in the realsense repo [here](https://github.com/IntelRealSense/librealsense/tree/master/wrappers/python/examples) while trialed, were deemed unneccesary to explore in any more detail. The Backend interface could not be enabled for Tutorial 5, Reading of the Bag files had issues using the code provided. Bag file reading functionality was already demonstrated. For any further work ROS would be preferential. The other examples were Jupityer notebook files and were not tested or beyond the scope of the work. 


