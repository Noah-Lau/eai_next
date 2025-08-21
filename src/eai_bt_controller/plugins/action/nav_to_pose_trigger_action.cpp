#include "plugins/action/nav_to_pose_trigger_action.hpp"
#include "behaviortree_ros2/plugins.hpp"

bool NavigationTriggerAction::setGoal(RosActionNode::Goal& goal)
{
  // geometry_msgs::msg::PoseStamped nav_goal;
    Expected<geometry_msgs::msg::PoseStamped> nav_goal = getInput<geometry_msgs::msg::PoseStamped>("nav_goal");
    Expected<std::string> nav_tree = getInput<std::string>("nav_tree");
  goal.pose = nav_goal.value();
  goal.behavior_tree = nav_tree.value();

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
