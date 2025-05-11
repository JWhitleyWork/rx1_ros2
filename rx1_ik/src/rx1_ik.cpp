#include "rx1_ik/rx1_ik.hpp"

#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <ik_solver_lib/trac_ik/trac_ik_solver.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_kdl/tf2_kdl.hpp>

Rx1IkNode::Rx1IkNode(const rclcpp::NodeOptions & options)
  : Node("rx1_ik", options),
  marker_server_("end_effector_marker", this),
  tf_buffer_(std::make_unique<tf2_ros::Buffer>(this->get_clock())),
  tf_listener_(std::make_unique<tf2_ros::TransformListener>(*tf_buffer_)),
  tf_br_(std::make_unique<tf2_ros::TransformBroadcaster>(*this))
{
  // Retrieve parameters
  double ik_timeout = declare_parameter("ik_timeout", 0.005); // Default timeout
  double eps = declare_parameter("eps", 1e-3); // Default error

  chain_start_ = declare_parameter("chain_start", "head_base_link");
  chain_r_end_ = declare_parameter("chain_r_end", "right_hand_link");
  chain_l_end_ = declare_parameter("chain_l_end", "left_hand_link");
  urdf_param_ = declare_parameter("urdf_param", "/robot_description");
  max_angle_change_ = declare_parameter("max_angle_change", 0.3);
  tracking_timeout_ = declare_parameter("tracking_timeout", 1.0);

  RCLCPP_INFO(this->get_logger(), "eps: %f", eps);

  ik_solver_r_ptr_ = std::make_shared<ik_solver_lib::TracIKSolver>();
  ik_solver_r_ptr_->initialize(shared_from_this(), chain_start_, chain_r_end_, urdf_param_, ik_timeout, eps);

  right_last_ik_time_ = now().seconds() - tracking_timeout_;

  ik_solver_l_ptr_ = std::make_shared<ik_solver_lib::TracIKSolver>();
  ik_solver_l_ptr_->initialize(shared_from_this(), chain_start_, chain_l_end_, urdf_param_, ik_timeout, eps);

  left_last_ik_time_ = now().seconds() - tracking_timeout_;

  RCLCPP_INFO(this->get_logger(), "TracIKSolver plugin loaded and initialized successfully.");

  // Publisher for joint states
  left_joint_state_pub_ =
    this->create_publisher<sensor_msgs::msg::JointState>("left_arm_joint_states", 10);
  right_joint_state_pub_ =
    this->create_publisher<sensor_msgs::msg::JointState>("right_arm_joint_states", 10);

  // Initialize subscribers
  right_gripper_pose_sub_ =
    this->create_subscription<geometry_msgs::msg::Pose>("right_gripper_pose", 10,
    std::bind(&Rx1IkNode::rightGripperPoseCallback, this, std::placeholders::_1));
  left_gripper_pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>("left_gripper_pose",
    10,
    std::bind(&Rx1IkNode::leftGripperPoseCallback, this, std::placeholders::_1));

  // Initialize joint states
  sensor_msgs::msg::JointState right_joint_state_msg;
  right_joint_state_msg.header.stamp = now();
  right_joint_state_msg.name.resize(7);
  right_joint_state_msg.position.resize(7);

  std::vector<std::string> right_joint_name =
  {
    "right_shoul_base2shoul_joint",
    "right_shoul2shoul_rot_joint",
    "right_arm2armrot_joint",
    "right_armrot2elbow_joint",
    "right_forearm2forearmrot_joint",
    "right_forearmrot2forearm_pitch_joint",
    "right_forearm_pitch2forearm_roll_joint"
  };

  for (int i = 0; i < 7; ++i) {
    right_joint_state_msg.position[i] = 0;
    right_joint_state_msg.name[i] = right_joint_name[i];
  }

  right_joint_state_msg.position[3] = -1.57;
  right_joint_state_pub_->publish(right_joint_state_msg);

  right_prev_joint_state_msg_ = right_joint_state_msg;
  right_cur_joint_state_msg_ = right_joint_state_msg;

  sensor_msgs::msg::JointState left_joint_state_msg;
  left_joint_state_msg.header.stamp = now();
  left_joint_state_msg.name.resize(7);
  left_joint_state_msg.position.resize(7);

  std::vector<std::string> left_joint_name =
  {
    "left_shoul_base2shoul_joint",
    "left_shoul2shoul_rot_joint",
    "left_arm2armrot_joint",
    "left_armrot2elbow_joint",
    "left_forearm2forearmrot_joint",
    "left_forearmrot2forearm_pitch_joint",
    "left_forearm_pitch2forearm_roll_joint"
  };

  for (int i = 0; i < 7; ++i) {
    left_joint_state_msg.position[i] = 0;
    left_joint_state_msg.name[i] = left_joint_name[i];
  }

  left_joint_state_msg.position[3] = -1.57;
  left_joint_state_pub_->publish(left_joint_state_msg);
  left_prev_joint_state_msg_ = left_joint_state_msg;
  left_cur_joint_state_msg_ = left_joint_state_msg;

  RCLCPP_INFO(this->get_logger(), "Joints initialized");

  // Initialize floating joint
  world_to_base_tf_stamped_.header.stamp = now();
  world_to_base_tf_stamped_.header.frame_id = "map";
  world_to_base_tf_stamped_.child_frame_id = "base_link";
  world_to_base_tf_stamped_.transform.translation.x = 0.0;
  world_to_base_tf_stamped_.transform.translation.y = 0.0;
  world_to_base_tf_stamped_.transform.translation.z = 0.0;
  world_to_base_tf_stamped_.transform.rotation.x = 0.0;
  world_to_base_tf_stamped_.transform.rotation.y = 0.0;
  world_to_base_tf_stamped_.transform.rotation.z = 0.0;
  world_to_base_tf_stamped_.transform.rotation.w = 1.0;
  tf_br_->sendTransform(world_to_base_tf_stamped_);

  // Initialize the interactive marker
  initializeInteractiveMarker();
}

void Rx1IkNode::make6DofMarker(visualization_msgs::msg::InteractiveMarker & int_marker)
{
  int_marker.scale = 0.15;
  // Create a control for each degree of freedom

  // X-axis control (red)
  visualization_msgs::msg::InteractiveMarkerControl control;
  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  // Y-axis control (green)
  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  // Z-axis control (blue)
  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  // Rotation around X-axis
  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);

  // Rotation around Y-axis
  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);

  // Rotation around Z-axis
  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);
}

void Rx1IkNode::initializeInteractiveMarker()
{
  // Create an interactive marker for the end effector
  //visualization_msgs::InteractiveMarker int_marker;
  int_marker_r_.header.frame_id = chain_start_; //"torso_link";
  int_marker_r_.name = "right_end_effector";
  int_marker_r_.description = "Right End Effector Control";


  int_marker_l_.header.frame_id = chain_start_;
  int_marker_l_.name = "left_end_effector";
  int_marker_l_.description = "Left End Effector Control";

  // Create a 6-DOF control which allows moving and rotating along all axes
  make6DofMarker(int_marker_r_);
  make6DofMarker(int_marker_l_);
  //make6DofMarker(int_marker_b_);

  // Add the interactive marker to our collection &
  // tell the server to call processFeedback() when feedback arrives for it
  marker_server_.insert(int_marker_r_, std::bind(&Rx1IkNode::markerRightCallback, this, std::placeholders::_1));
  marker_server_.insert(int_marker_l_, std::bind(&Rx1IkNode::markerLeftCallback, this, std::placeholders::_1));

  // 'commit' changes and send to all clients
  marker_server_.applyChanges();
}

void Rx1IkNode::markerRightCallback(
  visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr feedback)
{
  if (feedback->event_type == visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE) {
    // Convert the marker pose to a KDL::Frame
    KDL::Frame desired_pose;
    tf2::fromMsg(feedback->pose, desired_pose);

    // Solve IK
    KDL::JntArray result_joint_positions;
    if (ik_solver_r_ptr_->solveIK(desired_pose, result_joint_positions)) {
      bool success = true;

      for (size_t i = 0; i < result_joint_positions.rows(); ++i) {
        if (abs(right_prev_joint_state_msg_.position[i] - result_joint_positions(i)) >
          max_angle_change_)
        {
          success = false;
        }
      }

      if (success) {
        for (size_t i = 0; i < result_joint_positions.rows(); ++i) {
          // simple smoothing
          right_prev_joint_state_msg_.position[i] = right_prev_joint_state_msg_.position[i] * 0.9 +
            result_joint_positions(i) * 0.1;
        }

        RCLCPP_INFO(this->get_logger(), "Succeed finding right arm IK solution");
      } else {
        RCLCPP_INFO(this->get_logger(),
          "Succeed finding right arm IK solution but ditch the result to reduce shake");
      }
    } else {
      RCLCPP_WARN(this->get_logger(), "Failed to find right arm IK solution");
    }
  }
}

void Rx1IkNode::markerLeftCallback(
  visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr feedback)
{
  if (feedback->event_type == visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE) {
    // Convert the marker pose to a KDL::Frame
    KDL::Frame desired_pose;
    tf2::fromMsg(feedback->pose, desired_pose);

    // Solve IK
    KDL::JntArray result_joint_positions;
    if (ik_solver_l_ptr_->solveIK(desired_pose, result_joint_positions)) {
      bool success = true;
      for (size_t i = 0; i < result_joint_positions.rows(); ++i) {
        if (abs(left_prev_joint_state_msg_.position[i] - result_joint_positions(i)) >
          max_angle_change_)
        {
          success = false;
        }
      }

      if (success) {
        for (size_t i = 0; i < result_joint_positions.rows(); ++i) {
          // simple smoothing
          left_prev_joint_state_msg_.position[i] = left_prev_joint_state_msg_.position[i] * 0.9 +
            result_joint_positions(i) * 0.1;
        }

        RCLCPP_INFO(this->get_logger(), "Succeed finding left arm IK solution");
      } else {
        RCLCPP_INFO(this->get_logger(),
          "Succeed finding left arm IK solution but ditch the result to reduce shake");
      }
    } else {
      RCLCPP_WARN(this->get_logger(), "Failed to find left arm IK solution");
    }
  }
}

void Rx1IkNode::rightGripperPoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
{
  // Convert the msg pose to a KDL::Frame
  KDL::Frame desired_pose;
  tf2::fromMsg(msg, desired_pose);

  // Solve IK
  KDL::JntArray result_joint_positions;

  double before_ik_time = now().seconds();
  auto ik_solved = ik_solver_r_ptr_->solveIK(desired_pose, result_joint_positions);
  RCLCPP_INFO(this->get_logger(), "right ik took %f sec", now().seconds() - before_ik_time);

  if (ik_solved) {
    bool success = true;

    // if it's within tracking_timeout, then the angle change should be smaller than max_angle_change
    if ((now().seconds() - right_last_ik_time_) < tracking_timeout_) {
      for (size_t i = 0; i < result_joint_positions.rows(); ++i) {
        if (abs(right_prev_joint_state_msg_.position[i] - result_joint_positions(i)) >
          max_angle_change_)
        {
          success = false;
        }
      }
    }

    if (success) {
      if ((now().seconds() - right_last_ik_time_) < tracking_timeout_) {
        for (size_t i = 0; i < result_joint_positions.rows(); ++i) {
          right_prev_joint_state_msg_.position[i] = result_joint_positions(i);
        }
      } else {
        for (size_t i = 0; i < result_joint_positions.rows(); ++i) {
          right_prev_joint_state_msg_.position[i] = result_joint_positions(i);
        }
      }

      right_last_ik_time_ = now().seconds();
      RCLCPP_INFO(this->get_logger(), "Succeed finding right IK solution");
    } else {
      RCLCPP_INFO(this->get_logger(),
        "Succeed finding right IK solution but ditch the result to reduce shake");
    }
  } else {
    RCLCPP_WARN(this->get_logger(), "Failed to find right IK solution");
  }
}

void Rx1IkNode::leftGripperPoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
{
  // Convert the msg pose to a KDL::Frame
  KDL::Frame desired_pose;
  tf2::fromMsg(msg, desired_pose);

  // Solve IK
  KDL::JntArray result_joint_positions;

  if (ik_solver_l_ptr_->solveIK(desired_pose, result_joint_positions)) {
    bool success = true;

    // if it's within tracking_timeout, then the angle change should be smaller than max_angle_change
    if ((now().seconds() - left_last_ik_time_) < tracking_timeout_) {
      for (size_t i = 0; i < result_joint_positions.rows(); ++i) {
        if (abs(left_prev_joint_state_msg_.position[i] - result_joint_positions(i)) >
          max_angle_change_)
        {
          success = false;
        }
      }
    }

    if (success) {
      if ((now().seconds() - left_last_ik_time_) < tracking_timeout_) {
        for (size_t i = 0; i < result_joint_positions.rows(); ++i) {
          left_prev_joint_state_msg_.position[i] = result_joint_positions(i);
        }
      } else {
        for (size_t i = 0; i < result_joint_positions.rows(); ++i) {
          left_prev_joint_state_msg_.position[i] = result_joint_positions(i);
        }
      }

      left_last_ik_time_ = now().seconds();
      RCLCPP_INFO(this->get_logger(), "Succeed finding left IK solution");
    } else {
      RCLCPP_INFO(this->get_logger(),
        "Succeed finding left IK solution but ditch the result to reduce shake");
    }
  } else {
    RCLCPP_WARN(this->get_logger(), "Failed to find left IK solution");
  }
}

void Rx1IkNode::update()
{
  for (size_t i = 0; i < right_cur_joint_state_msg_.position.size(); i++) {
    // simple smoothing
    right_cur_joint_state_msg_.position[i] = right_cur_joint_state_msg_.position[i] * 0.9 +
      right_prev_joint_state_msg_.position[i] * 0.1;
  }

  right_cur_joint_state_msg_.header.stamp = now();
  right_joint_state_pub_->publish(right_cur_joint_state_msg_);

  for (size_t i = 0; i < left_cur_joint_state_msg_.position.size(); i++) {
    // simple smoothing
    left_cur_joint_state_msg_.position[i] = left_cur_joint_state_msg_.position[i] * 0.9 +
      left_prev_joint_state_msg_.position[i] * 0.1;
  }

  left_cur_joint_state_msg_.header.stamp = now();
  left_joint_state_pub_->publish(left_cur_joint_state_msg_);

  world_to_base_tf_stamped_.header.stamp = now();
  tf_br_->sendTransform(world_to_base_tf_stamped_);
  geometry_msgs::msg::PoseStamped pose;

  if (getLinkPose(chain_start_, chain_r_end_, pose)) {
    int_marker_r_.pose = pose.pose;
    marker_server_.insert(int_marker_r_);
    marker_server_.applyChanges();
    //RCLCPP_INFO(this->get_logger(), "Marker right pose updated");
  }

  if(getLinkPose(chain_start_, chain_l_end_, pose)) {
    int_marker_l_.pose = pose.pose;
    marker_server_.insert(int_marker_l_);
    marker_server_.applyChanges();
    //RCLCPP_INFO(this->get_logger(), "Marker left pose updated");
  }
}

bool Rx1IkNode::getLinkPose(
  const std::string frame, const std::string link,
  geometry_msgs::msg::PoseStamped & pose)
{
  geometry_msgs::msg::TransformStamped transformStamped;

  try {
    transformStamped = tf_buffer_->lookupTransform(frame, link, tf2::TimePointZero,
      std::chrono::seconds(3));
    pose.header.stamp = now();
    pose.header.frame_id = frame;
    pose.pose.position.x = transformStamped.transform.translation.x;
    pose.pose.position.y = transformStamped.transform.translation.y;
    pose.pose.position.z = transformStamped.transform.translation.z;
    pose.pose.orientation = transformStamped.transform.rotation;
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "%s", ex.what());
    return false;
  }

  return true;
}

bool Rx1IkNode::getPoseInNewFrame(
  const geometry_msgs::msg::PoseStamped & old_pose,
  const std::string & new_frame,
  geometry_msgs::msg::PoseStamped & new_pose)
{
  try {
    geometry_msgs::msg::TransformStamped transformStamped;
    transformStamped = tf_buffer_->lookupTransform(old_pose.header.frame_id, new_frame, tf2::TimePointZero,
      std::chrono::seconds(1));

    tf2::doTransform(old_pose, new_pose, transformStamped);
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "getPoseInNewFrame: %s", ex.what());
    return false;
  }

  return true;
}

geometry_msgs::msg::TransformStamped Rx1IkNode::poseToTransformStamped(
  const geometry_msgs::msg::Pose & pose, const std::string & frame_id,
  const std::string & child_frame_id)
{
  geometry_msgs::msg::TransformStamped transformStamped;

  transformStamped.header.stamp = now();
  transformStamped.header.frame_id = frame_id;
  transformStamped.child_frame_id = child_frame_id;

  transformStamped.transform.translation.x = pose.position.x;
  transformStamped.transform.translation.y = pose.position.y;
  transformStamped.transform.translation.z = pose.position.z;
  transformStamped.transform.rotation = pose.orientation;

  return transformStamped;
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(Rx1IkNode)
