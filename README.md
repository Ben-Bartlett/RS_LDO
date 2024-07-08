# Erasmus+ Placement Project

## Project Overview
This project was completed as part of an Erasmus+ placement in Montpellier, France during the early summer of 2024. The objective was to use a RealSense camera to provide real-time feedback for dual robotic arm manipulation of linear deformable objects, specifically focusing on manipulating a rope.

## Table of Contents
1. Introduction
2. Project Structure
3. Installation and Setup
4. Usage
5. Development Stages
   - Initial Setup and Camera Connection
   - Intermediate Development
   - Final Implementation
6. Resources
7. Contributing
8. Contact Information
9. Acknowledgments

## Introduction
This document serves to detail the work completed during the Erasmus+ placement and to provide a resource for future students who may continue the work on the perception task for dual manipulation (plus good practice for me, who does not often compile my code together tidily, to be able to come back and get it working straight away. Without the usual 3 hours of remembering). The manipulation of deformable objects is an interesting area with many applications requiring solutions to be created such as robotic suturing. The main focus here is with respect to perception, specifically creating a realtime input for use within the Model Predictive Control of the manipulators.

## Project Structure
- **/Code**: Contains all the code used in the project.
  - **/Initial**: Code for initial camera connection and setup.
  - **/Development**: Intermediate development and testing code.
  - **/Final**: Final version of the code integrating all components.
- **/Resources**: Contains useful? resources such as links, papers, and presentations of literature review .


## Installation and Setup
### Prerequisites
All developments were completed using Python and the realsense-viewer application within a Ubuntu 18.04 environment. Many other options exist, with a recomendation to utilise ROS in the future due to the basis of the realsense library being somewhat built around ROS and the utilisation of Bag files. I was too rusty with ROS to utilize it during this time. 

#### Installing
- __You will of course need python first.__ I won't cover this installation. The version utilized is __Python 3.6.9__. You can try other versions but the installation of other packages will also most likely need changing to different versions. (remove the specific versions of the libraries in the requirements.txt or install them manually) 

- With any python project, I would recommend a virtual environment, pick a name and a location to put it. I used ~/virtualenvs/ as a location and an environment name of MUS. For more info on choosing specific versions of python check [here](https://www.freecodecamp.org/news/how-to-setup-virtual-environments-in-python/)
  - Example: 
  ```bash
  cd <location>
  pip3 install virtualenv
  python3.6 -m venv <environment_name>
  source <enironment_name>/bin/activate
  # activate the environment
  source ~/virtualenvs/MUS/bin/activate
- With the virtualenv setup we can download the code, clone or fork it.
  ```bash
  git clone https://github.com/Ben-Bartlett/RS_LDO.git
  
- With the code downloaded we can install the requirments.txt (made for __python 3.6__)
  ```bash
  cd RS_LDO
  pip install -r requirements.txt
  
- Ensure you have a RealSense camera. The code here was developed using the __D435__ camera. I would expect it to work with any of the 400 series, and the other series with only minor changes.

or

- Utilise the bag files added to the repository. Download them at this link. I could not make a script to download the data, Fuck Onedrive.

  [https://ulcampus-my.sharepoint.com/:f:/g/personal/17218187_studentmail_ul_ie/EtIYMD9JirNEpHhStkT-lnkBOESLD-65iAI_5KaVxzuHJg](https://ulcampus-my.sharepoint.com/:f:/g/personal/17218187_studentmail_ul_ie/EtIYMD9JirNEpHhStkT-lnkBOESLD-65iAI_5KaVxzuHJg) 

- __Put the folder in the RS_LDO directory as RS_LDO/Data or leave it in ~/Downloads__. Because Onedrive is shit, if you download the entire folder, it will create a Zip file. This is not a problem. But they don't use ZIP standards. As such this will appear corrupted. Run the script called fix_zip.sh to be able to unzip the data
  ```bash
  chmod +x fix_zip.sh

- Lastly install the Realsense SDK following the instructions [here](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md)
  - verify with:
    ```bash
    realsense-viewer

## Usage
### Basic Usage
This information is available in the subdirectories' README.md files but to see the code in action and verify the installation I would recommend starting with the initial code.
  ```bash
  python /Code/Initial/Live_Scripts/Stream_RGB_Depth_2IR_live.py
  ```
This should open the connected cameras live RGB, depth and IR streams. 

## Utilising the Repo 
### First time
I would recommend starting with the Resouces and Reading the Report [here]().
This summarises all information. THe specific information is also in each subdirectory and its README.

For running the code I would recommend starting with the script detailed in basic usage above as a test. 

For understanding the codebase or how to develop any desired functionality, I would recommend going through all of the /Code/Initial/Bag_Scripts. You will need to download the bag files via ..

For understanding the development process that led to the final outputted code and to either verify the processing is correct or make changes, I would recommend going through the code examples in the /Code/Development folder. 

For utilising the main devlopment of the repo, it can be found in the /Code/Final Directory. This code .....

## Resources
Discussed in more detail in the **/Resources** folder [README.md](https://github.com/Ben-Bartlett/RS_LDO/blob/main/Resources/README.md)

## Contributing
1. Fork the repository.
2. Create your feature branch (git checkout -b feature/AmazingFeature).
3. Commit your changes (git commit -m 'Add some AmazingFeature').
4. Push to the branch (git push origin feature/AmazingFeature).
5. Open a Pull Request.


## Contact Information
Ben Bartlett - ben.bartlett@ul.ie


## Acknowledgments
-
-
