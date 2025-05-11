#include "rx1_motor/rx1_motor.hpp"

#include <cassert>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace rx1_motor
{

Rx1MotorNode::Rx1MotorNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("rx1_motor_node", options),
  joint_state_sub_(this->create_subscription<sensor_msgs::msg::JointState>("/command_joint_states",
    10, std::bind(&Rx1MotorNode::jointStateCallback, this, std::placeholders::_1))),
  right_arm_joint_state_sub_(this->create_subscription<sensor_msgs::msg::JointState>(
    "/right_arm_joint_states", 10,
    std::bind(&Rx1MotorNode::rightArmJointStateCallback, this, std::placeholders::_1))),
  left_arm_joint_state_sub_(this->create_subscription<sensor_msgs::msg::JointState>(
    "/left_arm_joint_states", 10,
    std::bind(&Rx1MotorNode::leftArmJointStateCallback, this, std::placeholders::_1))),
  torso_joint_state_sub_(this->create_subscription<sensor_msgs::msg::JointState>(
    "/torso_joint_states", 10,
    std::bind(&Rx1MotorNode::torsoJointStateCallback, this, std::placeholders::_1))),
  head_joint_state_sub_(this->create_subscription<sensor_msgs::msg::JointState>(
    "/head_joint_states", 10,
    std::bind(&Rx1MotorNode::headJointStateCallback, this, std::placeholders::_1))),
  right_gripper_sub_(this->create_subscription<std_msgs::msg::Float32>("/right_gripper", 10,
    std::bind(&Rx1MotorNode::rightGripperCallback, this, std::placeholders::_1))),
  left_gripper_sub_(this->create_subscription<std_msgs::msg::Float32>("/left_gripper", 10,
    std::bind(&Rx1MotorNode::leftGripperCallback, this, std::placeholders::_1)))
{
  servo_port_ = declare_parameter<std::string>("rx1_motor_node/servo_port",
      "/dev/ttyUSB-arduino4.3");

  if (!sts_servo_.begin(1000000, servo_port_.c_str())) {
    RCLCPP_ERROR(get_logger(), "[RX1_MOTOR] Failed initialize sts servo port %s!",
        servo_port_.c_str());
  }

  if (!scs_servo_.begin(1000000, servo_port_.c_str())) {
    RCLCPP_ERROR(get_logger(), "[RX1_MOTOR] Failed initialize scs servo port %s!",
        servo_port_.c_str());
  }

  for(int i = 0; i < right_arm_servo_ids_.size(); i++) {
    uint8_t id = right_arm_servo_ids_[i];
    sts_servo_.WritePosEx(id, 2048, 200, 20);
  }

  for(int i = 0; i < left_arm_servo_ids_.size(); i++) {
    uint8_t id = left_arm_servo_ids_[i];
    sts_servo_.WritePosEx(id, 2048, 200, 20);
  }

  for(int i = 0; i < torso_servo_ids_.size(); i++) {
    uint8_t id = torso_servo_ids_[i];
    sts_servo_.WritePosEx(id, 2048, 200, 20);
  }

  for(int i = 0; i < head_servo_ids_.size(); i++) {
    uint8_t id = head_servo_ids_[i];

    if (i == 1) {
      //sts_servo_.WritePosEx(id, 1600, 200, 20);
      sts_servo_.WritePosEx(id, 2048, 200, 20);
    } else if (i == 0) {
      sts_servo_.WritePosEx(id, 2048, 200, 20);
    } else if (i == 2) {
      sts_servo_.WritePosEx(id, 2000, 200, 20);
    } else {
      scs_servo_.WritePos(id, 512, 0, 100);
    }
  }

  last_spin_time_ = now();
}

Rx1MotorNode::~Rx1MotorNode()
{
  sts_servo_.end();
}

void Rx1MotorNode::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  std::vector<double> joint_positions = msg->position;

  std::vector<double> torso_joint_positions(3);
  std::vector<double> head_joint_positions(5);
  std::vector<double> right_arm_joint_positions(7);
  std::vector<double> left_arm_joint_positions(7);

  for (int i = 0; i < joint_positions.size(); i++) {
    if (i < 3) {
      torso_joint_positions[i] = joint_positions[i];
    } else if (i < 8) {
      head_joint_positions[i - 3] = joint_positions[i];
    } else if (i < 15) {
      right_arm_joint_positions[i - 8] = joint_positions[i];
    } else {
      left_arm_joint_positions[i - 15] = joint_positions[i];
    }
  }

  // Process the joint state information
  auto angles = torsoIk(TORSO_D_, TORSO_L1_, TORSO_H1_, TORSO_H2_, torso_joint_positions[1],
      torso_joint_positions[2]);
  torso_joint_positions[1] = angles[0];
  torso_joint_positions[2] = angles[1];
  std::vector<double> torso_speeds_(torso_servo_ids_.size(), TORSO_SPEED_);
  std::vector<double> torso_accs_(torso_servo_ids_.size(), TORSO_ACC_);
  motorCommand(torso_servo_ids_, torso_servo_dirs_, torso_servo_gears_, torso_joint_positions,
      torso_speeds_, torso_accs_);

  std::vector<double> head_speeds(head_servo_ids_.size(), HEAD_SPEED_);
  std::vector<double> head_accs(head_servo_ids_.size(), HEAD_ACC_);
  headMotorCommand(head_joint_positions, head_speeds, head_accs);

  std::vector<double> arm_speeds(right_arm_servo_ids_.size(), ARM_SPEED_);
  std::vector<double> arm_accs(right_arm_servo_ids_.size(), ARM_ACC_);
  motorCommand(right_arm_servo_ids_, right_arm_servo_dirs_, right_arm_servo_gears_,
      right_arm_joint_positions, arm_speeds, arm_accs);
  motorCommand(left_arm_servo_ids_, left_arm_servo_dirs_, left_arm_servo_gears_,
      left_arm_joint_positions, arm_speeds, arm_accs);
}

void Rx1MotorNode::rightArmJointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  // Access the joint state information
  std::vector<double> joint_positions = msg->position;

  // Process the joint state information
  std::vector<double> arm_speeds(right_arm_servo_ids_.size(), ARM_SPEED_);
  std::vector<double> arm_accs(right_arm_servo_ids_.size(), ARM_ACC_);
  motorCommand(right_arm_servo_ids_, right_arm_servo_dirs_, right_arm_servo_gears_, joint_positions,
      arm_speeds, arm_accs);
}

void Rx1MotorNode::leftArmJointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  // Access the joint state information
  std::vector<double> joint_positions = msg->position;

  // Process the joint state information
  std::vector<double> arm_speeds(left_arm_servo_ids_.size(), ARM_SPEED_);
  std::vector<double> arm_accs(left_arm_servo_ids_.size(), ARM_ACC_);
  motorCommand(left_arm_servo_ids_, left_arm_servo_dirs_, left_arm_servo_gears_, joint_positions,
      arm_speeds, arm_accs);
}

void Rx1MotorNode::torsoJointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  // Access the joint state information
  std::vector<double> joint_positions = msg->position;

  auto angles = torsoIk(TORSO_D_, TORSO_L1_, TORSO_H1_, TORSO_H2_, joint_positions[2],
      joint_positions[1]);

  joint_positions[1] = angles[0];
  joint_positions[2] = angles[1];

  std::vector<double> torso_speeds(torso_servo_ids_.size(), TORSO_SPEED_);
  std::vector<double> torso_accs(torso_servo_ids_.size(), TORSO_ACC_);

  motorCommand(torso_servo_ids_, torso_servo_dirs_, torso_servo_gears_, joint_positions,
      torso_speeds, torso_accs);
}

std::array<double, 2> Rx1MotorNode::torsoIk(
  double d, double L1, double h1, double h2, double roll,
  double pitch)
{
  // obtained from https://github.com/rocketman123456/ros2_ws/blob/master/src/humanoid_motor_control_cpp/src/humanoid_control/kinematic/ankle_ik.cpp
  double cx = cos(pitch);
  double sx = sin(pitch);
  double cy = cos(roll);
  double sy = sin(roll);

  double AL = -L1 * L1 * cy + L1 * d * sx * sy;
  double BL = -L1 * L1 * sy + L1 * h1 - L1 * d * sx * cy;
  double CL = -(L1 * L1 + d * d - d * d * cx - L1 * h1 * sy - d * h1 * sx * cy);

  double LenL = sqrt(AL * AL + BL * BL);

  double AR = -L1 * L1 * cy - L1 * d * sx * sy;
  double BR = -L1 * L1 * sy + L1 * h2 + L1 * d * sx * cy;
  double CR = -(L1 * L1 + d * d - d * d * cx - L1 * h2 * sy + d * h2 * sx * cy);

  double LenR = sqrt(AR * AR + BR * BR);

  if (LenL <= abs(CL) || LenR <= abs(CR)) {
    return {0.0, 0.0};
  } else {
    double tL_1 = asin(CL / LenL) - asin(AL / LenL);
    double tL_2 = asin(CL / LenL) + acos(BL / LenL);

    double tR_1 = asin(CR / LenR) - asin(AR / LenR);
    double tR_2 = asin(CR / LenR) + acos(BR / LenR);

    assert(fabs(tL_1 - tL_2) < 1e-3 && "tL_1 - tL_2 > 1e-3");
    assert(fabs(tR_1 - tR_2) < 1e-3 && "tR_1 - tR_2 > 1e-3");

    return {tL_1, tR_1};
  }
}

void Rx1MotorNode::headJointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  // Access the joint state information
  std::vector<double> joint_positions = msg->position;

  std::vector<double> joint_speeds(head_servo_ids_.size(), HEAD_SPEED_);
  std::vector<double> joint_accs(head_servo_ids_.size(), HEAD_ACC_);

  // Process the joint state information
  headMotorCommand(joint_positions, joint_speeds, joint_accs);
}

void Rx1MotorNode::rightGripperCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
  double grip_ratio = msg->data;

  int length = right_hand_servo_ids_.size();
  uint8_t id;
  int16_t pos;
  uint16_t speed = static_cast<unsigned short>(HAND_SPEED_ * SPEED_CONST_);
  uint8_t acc = static_cast<unsigned char>(HAND_ACC_ * ACC_CONST_);

  for (int i = 0; i < length; i++) {
    id = static_cast<unsigned char>(right_hand_servo_ids_[i]);
    pos = static_cast<short>(right_hand_servo_default_[i] + grip_ratio *
      right_hand_servo_range_[i]);

    // thumb and index fingers are sts servo, others are scs servos
    if (i == 1 || i == 2) {
      sts_servo_.WritePosEx(id, pos, speed, acc);
    } else if (i == 3) {
      scs_servo_.WritePos(id, pos, 0, 400); // id, pos, time, speed
    } else if (i == 4) {
      scs_servo_.WritePos(id, pos, 0, 300); // id, pos, time, speed
    } else if (i == 5) {
      scs_servo_.WritePos(id, pos, 0, 200); // id, pos, time, speed
    }
  }

  // Thumb yaw
  scs_servo_.WritePos(right_hand_servo_ids_[0], 200, 0, 400); // id, pos, time, speed
}

void Rx1MotorNode::leftGripperCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
  double grip_ratio = msg->data;

  int length = left_hand_servo_ids_.size();
  uint8_t id;
  int16_t pos;
  uint16_t speed = static_cast<unsigned short>(HAND_SPEED_ * SPEED_CONST_);
  uint8_t acc = static_cast<unsigned char>(HAND_ACC_ * ACC_CONST_);

  for (int i = 0; i < length; i++) {
    id = static_cast<unsigned char>(left_hand_servo_ids_[i]);
    pos = static_cast<short>(left_hand_servo_default_[i] + grip_ratio * left_hand_servo_range_[i]);

    // thumb and index fingers are sts servo, others are scs servos
    if (i == 1 || i == 2) {
      sts_servo_.WritePosEx(id, pos, speed, acc);
    } else if (i == 3) {
      scs_servo_.WritePos(id, pos, 0, 400); // id, pos, time, speed
    } else if (i == 4) {
      scs_servo_.WritePos(id, pos, 0, 300); // id, pos, time, speed
    } else if (i == 5) {
      scs_servo_.WritePos(id, pos, 0, 200); // id, pos, time, speed
    }
  }

  // Thumb yaw
  scs_servo_.WritePos(left_hand_servo_ids_[0], 512, 0, 400); // id, pos, time, speed
}

void Rx1MotorNode::headMotorCommand(
  const std::vector<double> & joint_positions,
  const std::vector<double> & joint_speeds,
  const std::vector<double> & joint_accs
)
{
  uint8_t id;
  int16_t pos;
  uint16_t speed;
  uint8_t acc;

  for (int i = 0; i < joint_positions.size(); i++) {
    id = static_cast<unsigned char>(head_servo_ids_[i]);
    speed = static_cast<unsigned short>(joint_speeds[i] * SPEED_CONST_);
    acc = static_cast<unsigned char>(joint_accs[i] * ACC_CONST_);

    //neck
    if (i == 0 || i == 1 || i == 2) {
      pos = static_cast<short>(joint_positions[i] / 3.14 * 2048 * head_servo_dirs_[i] *
        head_servo_gears_[i] + 2048);
      sts_servo_.WritePosEx(id, pos, speed, acc);
    } else {  // ear
      pos = static_cast<short>(joint_positions[i] / 3.14 * 512 * head_servo_dirs_[i] *
        head_servo_gears_[i] + 512);
      scs_servo_.WritePos(id, pos, 0, 0); // id, pos, time, speed
    }
  }
}

template<size_t N>
void Rx1MotorNode::motorCommand(
  const std::array<int, N> & joint_ids,
  const std::array<int, N> & joint_dirs,
  const std::array<int, N> & joint_gears,
  const std::vector<double> & joint_angles,
  const std::vector<double> & joint_speeds,
  const std::vector<double> & joint_accs
)
{
  int length = joint_ids.size();
  auto ids = std::make_unique<uint8_t[]>(length);
  auto pos = std::make_unique<int16_t[]>(length);
  auto speeds = std::make_unique<uint16_t[]>(length);
  auto accs = std::make_unique<uint8_t[]>(length);

  if (ids == nullptr || pos == nullptr || speeds == nullptr || accs == nullptr) {
    RCLCPP_WARN(get_logger(), "[RX1_MOTOR] Motor command memory allocation failed!");
  }

  for (int i = 0; i < length; i++) {
    ids[i] = static_cast<unsigned char>(joint_ids[i]);
    speeds[i] = static_cast<unsigned short>(joint_speeds[i] * joint_gears[i] * SPEED_CONST_);
    accs[i] = static_cast<unsigned char>(joint_accs[i] * joint_gears[i] * ACC_CONST_);
    pos[i] = static_cast<short>(joint_angles[i] / 3.14 * 2048 * joint_dirs[i] * joint_gears[i] +
      2048);

    // temporary change: make forearm faster
    if (i >= 3) {
      speeds[i] = 0;
      accs[i] = accs[i] * 10;
    }
  }

  sts_servo_.SyncWritePosEx(ids.get(), length, pos.get(), speeds.get(), accs.get());
}

} // namespace rx1_motor

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rx1_motor::Rx1MotorNode)
