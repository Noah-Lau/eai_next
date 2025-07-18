#include "plugins/action/nav_to_pose_trigger_action.hpp"
#include "behaviortree_ros2/plugins.hpp"

bool NavigationTriggerAction::setGoal(RosActionNode::Goal& goal)
{
  // auto nav_goal_2d = getInput<geometry_msgs::msg::Pose2D>("nav_goal");
  // if (!nav_goal_2d.has_value()) {
  //   RCLCPP_ERROR(rclcpp::get_logger("your_node_name"), "Error: %s", nav_goal_2d.error().c_str());
  //   }
  geometry_msgs::msg::Pose2D nav_goal_2d;
  nav_goal_2d.x = 1.0;
  nav_goal_2d.y = 1.0;
  nav_goal_2d.theta = 0.0;
  auto nav_goal_3d = pose2DToPoseStamped(nav_goal_2d, "map");

      // 提取 PoseStamped 对象
      std::cout << "Goal Pose:" << std::endl
                << "  Header:" << std::endl
                << "    Timestamp: " << nav_goal_3d.header.stamp.sec << "." << nav_goal_3d.header.stamp.nanosec << std::endl
                << "    Frame ID: " << nav_goal_3d.header.frame_id << std::endl
                << "  Pose:" << std::endl
                << "    Position: (x: " << nav_goal_3d.pose.position.x 
                << ", y: " << nav_goal_3d.pose.position.y 
                << ", z: " << nav_goal_3d.pose.position.z << ")" << std::endl
                << "    Orientation: (x: " << nav_goal_3d.pose.orientation.x 
                << ", y: " << nav_goal_3d.pose.orientation.y 
                << ", z: " << nav_goal_3d.pose.orientation.z 
                << ", w: " << nav_goal_3d.pose.orientation.w << ")" << std::endl;
  goal.pose = nav_goal_3d;
  goal.behavior_tree = "/home/noah/eai_next/src/eai_bt_controller/behavior_trees/navigate_through_poses_w_replanning_and_recovery.xml";

  return true;
}

void NavigationTriggerAction::onHalt()
{
  RCLCPP_INFO(logger(), "NavigationTriggerAction [%s] halted", name().c_str());
  // No additional cleanup needed beyond what's handled by RosActionNode
}

NodeStatus NavigationTriggerAction::onResultReceived(const WrappedResult& wr)
{
  RCLCPP_INFO(logger(), "NavigationTriggerAction [%s] completed with result code: %d",
              name().c_str(), static_cast<int>(wr.code));

  switch (wr.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(logger(), "Navigation to pose succeeded");
      return NodeStatus::SUCCESS;

    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(logger(), "Navigation to pose was aborted");
      return NodeStatus::FAILURE;

    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_WARN(logger(), "Navigation to pose was canceled");
      return NodeStatus::FAILURE;

    default:
      RCLCPP_ERROR(logger(), "Unknown result code received");
      return NodeStatus::FAILURE;
  }
}

NodeStatus NavigationTriggerAction::onFailure(ActionNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "NavigationTriggerAction [%s] failed with error: %s",
               name().c_str(), toStr(error));

  switch (error) {
    case SERVER_UNREACHABLE:
      RCLCPP_ERROR(logger(), "Cannot reach navigation action server");
      break;
    case SEND_GOAL_TIMEOUT:
      RCLCPP_ERROR(logger(), "Timeout while sending navigation goal");
      break;
    case GOAL_REJECTED_BY_SERVER:
      RCLCPP_ERROR(logger(), "Navigation goal rejected by server");
      break;
    case ACTION_ABORTED:
      RCLCPP_ERROR(logger(), "Navigation action aborted");
      break;
    case ACTION_CANCELLED:
      RCLCPP_ERROR(logger(), "Navigation action cancelled");
      break;
    case INVALID_GOAL:
      RCLCPP_ERROR(logger(), "Invalid navigation goal");
      break;
  }
  
  return NodeStatus::FAILURE;
}  // namespace BT
CreateRosNodePlugin(NavigationTriggerAction, "NavigationTriggerAction");
