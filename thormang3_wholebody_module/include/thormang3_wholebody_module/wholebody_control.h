
#ifndef THORMANG3_WHOLEBODY_MODULE_WHOLEBODY_CONTROL_
#define THORMANG3_WHOLEBODY_MODULE_WHOLEBODY_CONTROL_

#pragma once

#include <math.h>
#include <stdint.h>
#include <string>
#include <vector>
#include <map>

#include <geometry_msgs/Pose.h>

#include "thormang3_kinematics_dynamics/kinematics_dynamics.h"
#include "robotis_math/robotis_math.h"

class WholebodyControl
{
public:
  WholebodyControl(std::string control_group,
                   double init_time, double fin_time,
                   std::vector<double_t> init_joint_pos, std::vector<double_t> init_joint_vel, std::vector<double_t> init_joint_accel,
                   geometry_msgs::Pose goal_task_pose);
  virtual ~WholebodyControl();

  void initialize(std::vector<double_t> init_body_pos, std::vector<double_t> init_body_rot);
  void update(std::vector<double_t> init_body_pos, std::vector<double_t> init_body_rot,
              std::vector<double_t> init_task_pos, std::vector<double_t> init_task_vel, std::vector<double_t> init_task_accel);
  void finalize();

  bool set(double time);

  std::vector<double_t> getJointPosition(double time);
  std::vector<double_t> getJointVelocity(double time);
  std::vector<double_t> getJointAcceleration(double time);

  std::vector<double_t> getTaskPosition(double time);
  std::vector<double_t> getTaskVelocity(double time);
  std::vector<double_t> getTaskAcceleration(double time);
  std::vector<double_t> getOrientation(double time);

  void getGroupPose(std::string name, geometry_msgs::Pose *msg);

private:
  thormang3::KinematicsDynamics *robot_;
  robotis_framework::MinimumJerk *task_trajectory_;

  std::string control_group_;
  int end_link_;
  double init_time_, fin_time_;

  std::vector<double_t> init_joint_pos_, init_joint_vel_, init_joint_accel_;

  std::vector<double_t> init_task_pos_, init_task_vel_, init_task_accel_;
  std::vector<double_t> desired_task_pos_, desired_task_vel_, desired_task_accel_;
  std::vector<double_t> goal_task_pos_, goal_task_vel_, goal_task_accel_;

  Eigen::Quaterniond init_task_quaternion_, desired_task_quaternion_, goal_task_quaternion_;

  Eigen::MatrixXd init_body_pos_, init_body_vel_, init_body_accel_;
  Eigen::MatrixXd init_lleg_pos_, init_lleg_vel_, init_lleg_accel_;
  Eigen::MatrixXd init_rleg_pos_, init_rleg_vel_, init_rleg_accel_;

  Eigen::MatrixXd init_body_rot_, init_lleg_rot_, init_rleg_rot_;
};

#endif
