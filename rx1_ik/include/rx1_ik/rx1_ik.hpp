#ifndef RX1_IK__RX1_IK_HPP_
#define RX1_IK__RX1_IK_HPP_

#include <memory>
#include <string>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "ik_solver_lib/base/ik_solver_base.hpp"

class Rx1IkNode : public rclcpp::Node
{
public:
  Rx1IkNode(const rclcpp::NodeOptions & options);

  void initializeInteractiveMarker();
  void markerRightCallback(
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr feedback);
  void markerLeftCallback(
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr feedback);
  void rightGripperPoseCallback(const geometry_msgs::msg::Pose msg);
  void leftGripperPoseCallback(const geometry_msgs::msg::Pose msg);
  void update();

private:
  void make6DofMarker(visualization_msgs::msg::InteractiveMarker & int_marker);

  interactive_markers::msg::InteractiveMarkerServer marker_server_;
  visualization_msgs::msg::InteractiveMarker int_marker_r_;
  visualization_msgs::msg::InteractiveMarker int_marker_l_;

  rclcpp::Subscription<geometry_msgs::Pose>::SharedPtr right_gripper_pose_sub_;
  rclcpp::Subscription<geometry_msgs::Pose>::SharedPtr left_gripper_pose_sub_;

  rclcpp::Publisher<sensor_msgs::JointState>::SharedPtr right_joint_state_pub_;
  rclcpp::Publisher<sensor_msgs::JointState>::SharedPtr left_joint_state_pub_;

  std::unique_ptr<pluginlib::ClassLoader<ik_solver_plugin::IKSolverBase>> ik_loader_r_ptr_;
  std::shared_ptr<ik_solver_plugin::IKSolverBase> ik_solver_r_ptr_;

  std::unique_ptr<pluginlib::ClassLoader<ik_solver_plugin::IKSolverBase>> ik_loader_l_ptr_;
  std::shared_ptr<ik_solver_plugin::IKSolverBase> ik_solver_l_ptr_;

  sensor_msgs::msg::JointState right_prev_joint_state_msg_;
  sensor_msgs::msg::JointState left_prev_joint_state_msg_;

  sensor_msgs::msg::JointState right_cur_joint_state_msg_;
  sensor_msgs::msg::JointState left_cur_joint_state_msg_;

  geometry_msgs::msg::TransformStamped world_to_base_tf_stamped_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_br_;

    // Params
  std::string chain_start_, chain_r_end_, chain_l_end_, urdf_param_;
  double max_angle_change_;
  double tracking_timeout_;

    // Ik variables
  double right_last_ik_time_;
  double left_last_ik_time_;

    // util functions

    // Get link's pose in frame
  bool getLinkPose(
    const std::string frame, const std::string link,
    geometry_msgs::msg::PoseStamped & pose);
    // Turn the pose in frame a to frame b
  bool getPoseInNewFrame(
    const geometry_msgs::msg::PoseStamped old_pose,
    const std::string new_frame, geometry_msgs::msg::PoseStamped & new_pose);
    // Turn pose to transform
  geometry_msgs::msg::TransformStamped poseToTransformStamped(
    const geometry_msgs::msg::Pose & pose,
    const std::string & frame_id, const std::string & child_frame_id);
};

#endif // RX1_IK__RX1_IK_HPP_
