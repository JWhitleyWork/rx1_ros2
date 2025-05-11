// Copyright 2025 Electrified Autonomy, LLC

#include <array>
#include <cassert>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "rx1_motor_lib/rx1_motor_lib.hpp"

namespace rx1_motor_lib
{

Rx1MotorLib::~Rx1MotorLib()
{
  sts_servo_.end();
}

void Rx1MotorLib::initializeServos(const std::string & servo_port)
{
  if (!sts_servo_.begin(1000000, servo_port.c_str())) {
    throw std::runtime_error("Failed to initialize STS servo");
  }

  if (!scs_servo_.begin(1000000, servo_port.c_str())) {
    throw std::runtime_error("Failed to initialize SCS servo");
  }

  for(size_t i = 0; i < right_arm_servo_ids_.size(); i++) {
    uint8_t id = right_arm_servo_ids_[i];
    sts_servo_.WritePosEx(id, 2048, 200, 20);
  }

  for(size_t i = 0; i < left_arm_servo_ids_.size(); i++) {
    uint8_t id = left_arm_servo_ids_[i];
    sts_servo_.WritePosEx(id, 2048, 200, 20);
  }

  for(size_t i = 0; i < torso_servo_ids_.size(); i++) {
    uint8_t id = torso_servo_ids_[i];
    sts_servo_.WritePosEx(id, 2048, 200, 20);
  }

  for(size_t i = 0; i < head_servo_ids_.size(); i++) {
    uint8_t id = head_servo_ids_[i];

    if (i == 1) {
      sts_servo_.WritePosEx(id, 1600, 200, 20);
    } else if (i == 0 || i == 2) {
      sts_servo_.WritePosEx(id, 2048, 200, 20);
    } else {
      scs_servo_.WritePos(id, 512, 0, 100);
    }
  }
}

void Rx1MotorLib::headMotorCommand(
  const std::vector<double> & joint_positions,
  const std::vector<double> & joint_speeds,
  const std::vector<double> & joint_accs)
{
  uint8_t id;
  int16_t pos;
  uint16_t speed;
  uint8_t acc;

  for (size_t i = 0; i < joint_positions.size(); i++) {
    id = static_cast<uint8_t>(head_servo_ids_[i]);
    speed = static_cast<uint16_t>(joint_speeds[i] * SPEED_CONST_);
    acc = static_cast<uint8_t>(joint_accs[i] * ACC_CONST_);

    if (i == 0 || i == 1 || i == 2) {  // neck
      pos = static_cast<int16_t>(joint_positions[i] / 3.14 * 2048 * head_servo_dirs_[i] *
        head_servo_gears_[i] + 2048);
      sts_servo_.WritePosEx(id, pos, speed, acc);
    } else {  // ear
      pos = static_cast<int16_t>(joint_positions[i] / 3.14 * 512 * head_servo_dirs_[i] *
        head_servo_gears_[i] + 512);
      scs_servo_.WritePos(id, pos, 0, 0);  // id, pos, time, speed
    }
  }
}

void Rx1MotorLib::leftArmMotorCommand(
  const std::vector<double> & joint_positions,
  const std::vector<double> & joint_speeds,
  const std::vector<double> & joint_accs)
{
  motorCommand(left_arm_servo_ids_, left_arm_servo_dirs_, left_arm_servo_gears_, joint_positions,
      joint_speeds, joint_accs);
}

void Rx1MotorLib::rightArmMotorCommand(
  const std::vector<double> & joint_positions,
  const std::vector<double> & joint_speeds,
  const std::vector<double> & joint_accs)
{
  motorCommand(right_arm_servo_ids_, right_arm_servo_dirs_, right_arm_servo_gears_, joint_positions,
      joint_speeds, joint_accs);
}

void Rx1MotorLib::torsoMotorCommand(
  const std::vector<double> & joint_positions,
  const std::vector<double> & joint_speeds,
  const std::vector<double> & joint_accs)
{
  auto joint_pos = joint_positions;
  auto angles = torsoIk(TORSO_D_, TORSO_L1_, TORSO_H1_, TORSO_H2_, joint_positions[2],
      joint_positions[1]);

  joint_pos[1] = angles[0];
  joint_pos[2] = angles[1];

  motorCommand(torso_servo_ids_, torso_servo_dirs_, torso_servo_gears_, joint_pos, joint_speeds,
      joint_accs);
}

void Rx1MotorLib::leftGripperMotorCommand(const double grip_ratio)
{
  int length = left_hand_servo_ids_.size();
  uint8_t id;
  int16_t pos;
  uint16_t speed = static_cast<uint16_t>(HAND_SPEED_ * SPEED_CONST_);
  uint8_t acc = static_cast<uint8_t>(HAND_ACC_ * ACC_CONST_);

  for (int i = 0; i < length; i++) {
    id = static_cast<uint8_t>(left_hand_servo_ids_[i]);
    pos = static_cast<int16_t>(left_hand_servo_default_[i] + grip_ratio *
      left_hand_servo_range_[i]);
    if (i == 1 || i == 2) {   // thumb and index fingers are sts servo, others are scs servos
      sts_servo_.WritePosEx(id, pos, speed, acc);
    } else if (i == 3) {
      scs_servo_.WritePos(id, pos, 0, 400);       // id, pos, time, speed
    } else if (i == 4) {
      scs_servo_.WritePos(id, pos, 0, 300);       // id, pos, time, speed
    } else if (i == 5) {
      scs_servo_.WritePos(id, pos, 0, 200);       // id, pos, time, speed
    }
  }

    // Thumb yaw
  scs_servo_.WritePos(left_hand_servo_ids_[0], 512, 0, 400);   // id, pos, time, speed
}

void Rx1MotorLib::rightGripperMotorCommand(const double grip_ratio)
{
  int length = right_hand_servo_ids_.size();
  uint8_t id;
  int16_t pos;
  uint16_t speed = static_cast<uint16_t>(HAND_SPEED_ * SPEED_CONST_);
  uint8_t acc = static_cast<uint8_t>(HAND_ACC_ * ACC_CONST_);

  for (int i = 0; i < length; i++) {
    id = static_cast<uint8_t>(right_hand_servo_ids_[i]);
    pos = static_cast<int16_t>(right_hand_servo_default_[i] + grip_ratio *
      right_hand_servo_range_[i]);

    if (i == 1 || i == 2) {   // thumb and index fingers are sts servo, others are scs servos
      sts_servo_.WritePosEx(id, pos, speed, acc);
    } else if (i == 3) {
      scs_servo_.WritePos(id, pos, 0, 400);       // id, pos, time, speed
    } else if (i == 4) {
      scs_servo_.WritePos(id, pos, 0, 300);       // id, pos, time, speed
    } else if (i == 5) {
      scs_servo_.WritePos(id, pos, 0, 200);       // id, pos, time, speed
    }
  }

    // Thumb yaw
  scs_servo_.WritePos(right_hand_servo_ids_[0], 200, 0, 400);   // id, pos, time, speed
}

std::array<double, 2> Rx1MotorLib::torsoIk(
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

template<size_t N>
void Rx1MotorLib::motorCommand(
  const std::array<int, N> & joint_ids,
  const std::array<int, N> & joint_dirs,
  const std::array<int, N> & joint_gears,
  const std::vector<double> & joint_angles,
  const std::vector<double> & joint_speeds,
  const std::vector<double> & joint_accs)
{
  int length = joint_ids.size();
  std::vector<int16_t> pos(length);
  std::vector<uint16_t> speeds(length);
  std::vector<uint8_t> accs(length);
  std::vector<uint8_t> ids(length);

  for (int i = 0; i < length; i++) {
    accs[i] = static_cast<uint8_t>(joint_accs[i] * joint_gears[i] * ACC_CONST_);
    ids[i] = static_cast<uint8_t>(joint_ids[i]);
    pos[i] = static_cast<int16_t>(joint_angles[i] / 3.14 * 2048 * joint_dirs[i] * joint_gears[i] +
      2048);
    speeds[i] = static_cast<uint16_t>(joint_speeds[i] * joint_gears[i] * SPEED_CONST_);

    // temporary change: make forearm faster
    if (i >= 3) {
      speeds[i] = 0;
      accs[i] = accs[i] * 10;
    }
  }

  sts_servo_.SyncWritePosEx(ids.data(), length, pos.data(), speeds.data(), accs.data());
}

}  // namespace rx1_motor_lib
