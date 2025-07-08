# eai_next Workspace

This is a ROS 2 workspace for the eai_next project, containing custom packages and dependencies managed via Git submodules and install_dependencies.sh.

## Custom Packages
- `eai_bt_controller`: Custom behavior tree controller.
- `underlay.repos`: Dependency configuration file.
- `install_dependencies.sh`: Script to install additional dependencies.

## Dependencies
- Other packages (e.g., BehaviorTree, BehaviorTree.ROS2, navigation2) are managed as Git submodules or installed via install_dependencies.sh.

## Usage
1. Clone the repository: `git clone --recurse-submodules https://github.com/<your_username>/eai_next.git`
2. Run `install_dependencies.sh` to set up dependencies.
3. Build the workspace: `colcon build`
4. Source the environment: `source install/setup.bash`
