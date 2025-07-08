# Sample Behaviors

Behavior Tree Controller for EAI Robot
# TreeExecutionServer Sample

Documentation on the TreeExecutionServer used in this example can be found [here](../behaviortree_ros2/tree_execution_server.md).

To start the sample Execution Server that load a list of plugins and BehaviorTrees from `yaml` file:
``` bash
ros2 launch eai_bt_controller eai_bt_controller.launch.xml
```

> *NOTE:* For documentation on the `yaml` parameters please see [bt_executor_parameters.md](../behaviortree_ros2/bt_executor_parameters.md).

As the Server starts up it will print out the name of the ROS Action followed by the plugins and BehaviorTrees it was able to load.

To call the Action Server from the command line:
``` bash
ros2 action send_goal /eai_bt_server btcpp_ros2_interfaces/action/ExecuteTree "{target_tree: MainTree}"
```