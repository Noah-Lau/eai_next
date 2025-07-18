#include "behaviortree_ros2/bt_action_node.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "../utils/utils_func.hpp"

using namespace BT;

class NavigationTriggerAction : public RosActionNode<nav2_msgs::action::NavigateToPose>
{
public:
  NavigationTriggerAction(const std::string& name, const NodeConfig& conf, 
                          const RosNodeParams& params)
  : RosActionNode<nav2_msgs::action::NavigateToPose>(name, conf,params)
  {}

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        // InputPort<geometry_msgs::msg::Pose2D>("nav_goal")
      });
  }

  bool setGoal(Goal& goal) override;

  void onHalt() override;

  BT::NodeStatus onResultReceived(const WrappedResult& wr) override;

  virtual BT::NodeStatus onFailure(ActionNodeErrorCode error) override;

};