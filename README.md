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
8. License
9. Contact Information
10. Acknowledgments

## Introduction
This document serves to detail the work completed during the Erasmus+ placement and to provide a resource for future students who may continue the work on the perception task for dual manipulation. The manipulation of deformable objects is an interesting area with many applications requiring solutions to be created. The main focus here is with respect to perception, specifically creating a realtime input for use within the Model Predictive Control.

## Project Structure
- **/code**: Contains all the code used in the project.
  - **/initial_setup**: Code for initial camera connection and setup.
  - **/development_stages**: Intermediate development and testing code.
  - **/final_implementation**: Final version of the code integrating all components.
- **/resources**: Contains useful resources such as links, papers, and presentations.
- **/documents**: Contains the literature review, project reports, and other documentation.

## Installation and Setup
### Prerequisites
- List any prerequisites (e.g., software, libraries) required to run the project.
  - Example: `sudo apt-get install librealsense2`
- Ensure you have a RealSense camera connected.

### Installing
1. Clone the repository:
   git clone https://github.com/yourusername/repository.git
2. Navigate to the project directory:
   cd repository
3. Install the necessary dependencies:
   pip install -r requirements.txt

## Usage
### Basic Usage
- Instructions on how to run the initial setup code:
  cd code/initial_setup
  python connect_camera.py
- Example commands for running the final implementation:
  cd code/final_implementation
  python main.py

## Development Stages
### Initial Setup and Camera Connection
- Details on the initial setup process, including connecting the RealSense camera.
- Example code snippets and explanations.

### Intermediate Development
- Documentation of the development process, including the various stages of testing and iteration.
- Include snippets of testing code that may be useful.

### Final Implementation
- Step-by-step explanation of the final code.
- How different modules were integrated to achieve the final result.

## Resources
- Links to useful resources:
  - RealSense SDK Documentation: https://www.intelrealsense.com/sdk-2/
  - Research Paper 1: https://linktopaper1.com
  - Research Paper 2: https://linktopaper2.com
- Presentations on the literature review and other relevant topics.

## Contributing
1. Fork the repository.
2. Create your feature branch (git checkout -b feature/AmazingFeature).
3. Commit your changes (git commit -m 'Add some AmazingFeature').
4. Push to the branch (git push origin feature/AmazingFeature).
5. Open a Pull Request.

## License
Distributed under the MIT License. See LICENSE for more information.

## Contact Information
Your Name - @yourTwitter - email@example.com

Project Link: https://github.com/yourusername/repository

## Acknowledgments
- Person or resource for inspiration.
- Another person or resource.
