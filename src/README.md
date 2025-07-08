# EAI-NexT Workspace

## Description
EAI-NexT: Embodied AI Laboratory’s Humanoid Robot Navigation, Execution, and Tools. This workspace is designed to support the development and testing of humanoid robot navigation, execution, and related tools, featuring custom packages and managed dependencies.

## First-Time Usage

To set up the EAI-NexT workspace for the first time, follow these steps:

### Step 1: Clone the Repository
Clone the repository using the following command:
```bash
git clone --recurse-submodules https://github.com/Noah-Lau/eai_next.git 

sudo apt remove ros-humble-navigation2 ros-humble-nav2-*
sudo apt remove ros-humble-slam-toolbox

cd ~/eai_next/src
bash install_dependencies.sh

```

### Step 2: Build the Workspace
Return to the workspace root and build with specific packages:
```bash
cd ..
colcon build --parallel-workers 2 --allow-overriding common_interfaces diagnostic_msgs geometry_msgs launch launch_testing launch_testing_ament_cmake launch_xml launch_yaml nav2_map_server nav_msgs sensor_msgs sensor_msgs_py shape_msgs std_msgs std_srvs trajectory_msgs visualization_msgs
```
