#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <rclcpp/rclcpp.hpp>

geometry_msgs::msg::PoseStamped pose2DToPoseStamped(const geometry_msgs::msg::Pose2D& pose2d, const std::string& frame_id) {
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = frame_id;
    pose_stamped.header.stamp = rclcpp::Clock().now();
    pose_stamped.pose.position.x = pose2d.x;
    pose_stamped.pose.position.y = pose2d.y;
    pose_stamped.pose.position.z = 0.0;
    tf2::Quaternion quaternion;
    quaternion.setRPY(0.0, 0.0, pose2d.theta);
    pose_stamped.pose.orientation.x = quaternion.x();
    pose_stamped.pose.orientation.y = quaternion.y();
    pose_stamped.pose.orientation.z = quaternion.z();
    pose_stamped.pose.orientation.w = quaternion.w();
    return pose_stamped;
}
