# Project Setup and Execution Guide

## Prerequisites
Ensure you have the following installed on your system:
- ROS 2 Humble
- Visual Studio Code
- Necessary dependencies (as outlined below)

---

## Steps Before Executing the Project

### 1. Clone the Repository
Clone the project repository from the desired source:
```bash
cd ~
git clone <repository_url>
```

### 2. Navigate to the Workspace
Move into your workspace directory:
```bash
cd {workspace/../../}
```

### 3. Build the Project
Use `colcon` to build the project:
```bash
colcon build
```

### 4. Source the Project
Ensure the project environment is properly set up by sourcing the installation setup file:
```bash
source install/setup.bash
```

### 5. Install Required Dependencies
Install the necessary dependencies for the project:
```bash
sudo apt install ros-humble-usb-cam
sudo apt install ros-humble-gazebo-ros
sudo apt install python3-opencv
```

---

## Steps During Project Execution

### 1. Open Visual Studio Code
Ensure the pre-steps are completed, then open the project in Visual Studio Code. Run the provided code. The camera node will be created automatically.

### 2. Subscribe to the Camera Node
Run the following command to subscribe to the camera node:
```bash
ros2 run camera_subscriber camera_node
```

### 3. Launch the Robot Visualization Environment
Set the TurtleBot3 model and launch the Gazebo simulation environment:
```bash
export TURTLEBOT3_MODEL=burger
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:`ros2 pkg prefix turtlebot3_gazebo`/share/turtlebot3_gazebo/models
ros2 launch turtlebot3_gazebo empty_world.launch.py
```

---

## Notes
- Ensure all steps are followed sequentially for a smooth setup and execution.
- For troubleshooting and further support, refer to the official ROS 2 Humble documentation or the project repository's issues page on the electronic platform of poznan university of technology.

