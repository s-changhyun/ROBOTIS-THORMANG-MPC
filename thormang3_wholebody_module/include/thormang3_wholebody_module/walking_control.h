
#ifndef THORMANG3_WHOLEBODY_MODULE_WALKING_CONTROL_
#define THORMANG3_WHOLEBODY_MODULE_WALKING_CONTROL_

#pragma once

#include <math.h>
#include <stdint.h>
#include <string>
#include <vector>
#include <map>

#include <geometry_msgs/Pose2D.h>

#include "thormang3_wholebody_module_msgs/FootStepCommand.h"
#include "thormang3_wholebody_module_msgs/FootStepArray.h"
#include "thormang3_wholebody_module_msgs/PreviewResponse.h"

#include "thormang3_kinematics_dynamics/kinematics_dynamics.h"
#include "robotis_math/robotis_math.h"

enum WALKING_LEG {
  LEFT_LEG = 0,
  RIGHT_LEG = 1,
  LEG_COUNT = 2
};

enum WALKING_PHASE {
  DSP = 0, // Double Support Phase
  SSP = 1, // Single Support Phase
  PHASE_COUNT = 2
};

class WalkingControl
{
public:
  WalkingControl(double control_cycle,
                 double dsp_ratio, double lipm_height, double foot_height_max, double zmp_offset_x, double zmp_offset_y,
                 std::vector<double_t> x_lipm, std::vector<double_t> y_lipm,
                 std::vector<double_t> init_joint_pos, std::vector<double_t> init_joint_vel, std::vector<double_t> init_joint_accel);
  virtual ~WalkingControl();

  void initialize(thormang3_wholebody_module_msgs::FootStepCommand foot_step_command,
                  std::vector<double_t> init_body_pos, std::vector<double_t> init_body_rot);
  void update(int walking_leg, int walking_phase,
              thormang3_wholebody_module_msgs::FootStepCommand foot_step_command,
              std::vector<double_t> init_body_pos, std::vector<double_t> init_body_rot,
              std::vector<double_t> init_left_foot_pos, std::vector<double_t> init_left_foot_vel, std::vector<double_t> init_left_foot_accel,
              std::vector<double_t> init_right_foot_pos, std::vector<double_t> init_right_foot_vel, std::vector<double_t> init_right_foot_accel);
  void next();
  void finalize();
  bool set(double time, int step);

  double getLipmHeight();

  void calcFootStepParam();
  void updateFootStepParam();
  void calcFootTrajectory(int step);
  void calcFootStepPose(double time,  int step);
  void calcRefZMP(int step);
  void calcPreviewParam(thormang3_wholebody_module_msgs::PreviewResponse msg);
  void calcPreviewControl(double time, int step);

  double calcRefZMPx(int step);
  double calcRefZMPy(int step);

  std::vector<double_t> getJointPosition(int step, double time);
  std::vector<double_t> getJointVelocity(int step, double time);
  std::vector<double_t> getJointAcceleration(int step, double time);

  void getWalkingPosition(std::vector<double_t> &left_foot_pos,
                          std::vector<double_t> &right_foot_pos,
                          std::vector<double_t> &body_pos);
  void getWalkingVelocity(std::vector<double_t> &left_foot_vel,
                          std::vector<double_t> &right_foot_vel,
                          std::vector<double_t> &body_vel);
  void getWalkingAccleration(std::vector<double_t> &left_foot_accel,
                             std::vector<double_t> &right_foot_accel,
                             std::vector<double_t> &body_accel);
  void getWalkingOrientation(std::vector<double_t> &left_foot_quat,
                             std::vector<double_t> &right_foot_quat,
                             std::vector<double_t> &body_quat);
  void getLIPM(std::vector<double_t> &x_lipm, std::vector<double_t> &y_lipm);
  void getWalkingState(int &walking_leg, int &walking_phase);

protected:
  thormang3::KinematicsDynamics *robot_;

//  robotis_framework::MinimumJerk *body_trajectory_;
  robotis_framework::MinimumJerkViaPoint *right_foot_trajectory_;
  robotis_framework::MinimumJerkViaPoint *left_foot_trajectory_;

  double init_time_, fin_time_;
  double control_cycle_;

  int walking_leg_;
  int walking_phase_;

  // Foot Trajectory
  double foot_size_x_;
  double foot_size_y_;
  double foot_origin_shift_x_;
  double foot_origin_shift_y_;

  double dsp_ratio_;
  double foot_trajectory_max_z_;

  int foot_step_size_;
  thormang3_wholebody_module_msgs::FootStepCommand foot_step_command_;
  thormang3_wholebody_module_msgs::FootStepArray foot_step_param_;
  thormang3_wholebody_module_msgs::PreviewResponse preview_response_;

  bool walking_update_;
  double foot_x_diff_;
  double foot_y_diff_;
  double foot_z_diff_;

  // Preview Control
  double preview_time_;
  int preview_size_;
  double lipm_height_;
  double sum_of_zmp_x_, sum_of_zmp_y_ ;
  double sum_of_cx_, sum_of_cy_ ;
  Eigen::MatrixXd A_, b_, c_;
  Eigen::MatrixXd k_x_;
  double k_s_;
  Eigen::MatrixXd f_;
  Eigen::MatrixXd u_x_, u_y_;
  Eigen::MatrixXd x_lipm_, y_lipm_;

  Eigen::MatrixXd K_, P_;

  double ref_zmp_x_, ref_zmp_y_;
  double preview_sum_zmp_x_, preview_sum_zmp_y_;
  double offset_zmp_x_, offset_zmp_y_;

  // Pose Information
  double init_body_yaw_angle_;
  std::vector<double_t> init_joint_pos_, init_joint_vel_, init_joint_accel_;

  std::vector<double_t> init_body_pos_, init_body_vel_, init_body_accel_;
  std::vector<double_t> desired_body_pos_, desired_body_vel_, desired_body_accel_;
  std::vector<double_t> goal_body_pos_, goal_body_vel_, goal_body_accel_;
  Eigen::Quaterniond    init_body_quaternion_, desired_body_quaternion_, goal_body_quaternion_;

  std::vector<double_t> init_left_foot_pos_, init_left_foot_vel_, init_left_foot_accel_;
  std::vector<double_t> desired_left_foot_pos_, desired_left_foot_vel_, desired_left_foot_accel_;
  std::vector<double_t> goal_left_foot_pos_, goal_left_foot_vel_, goal_left_foot_accel_;
  Eigen::Quaterniond    init_left_foot_quaternion_, desired_left_foot_quaternion_, goal_left_foot_quaternion_;

  std::vector<double_t> init_right_foot_pos_, init_right_foot_vel_, init_right_foot_accel_;
  std::vector<double_t> desired_right_foot_pos_, desired_right_foot_vel_, desired_right_foot_accel_;
  std::vector<double_t> goal_right_foot_pos_, goal_right_foot_vel_, goal_right_foot_accel_;
  Eigen::Quaterniond    init_right_foot_quaternion_, desired_right_foot_quaternion_, goal_right_foot_quaternion_;

  // Joint Information
  std::vector<double_t> desired_joint_pos_, desired_joint_pos_past_;
  std::vector<double_t> desired_joint_vel_, desired_joint_vel_past_;
  std::vector<double_t> desired_joint_accel_;

};

#endif
