
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
                 std::vector<double_t> x_lipm, std::vector<double_t> y_lipm);
  virtual ~WalkingControl();

  void initialize(thormang3_wholebody_module_msgs::FootStepCommand foot_step_command,
                  std::vector<double_t> init_body_pos, std::vector<double_t> init_body_Q,
                  std::vector<double_t> init_r_foot_pos, std::vector<double_t> init_r_foot_Q,
                  std::vector<double_t> init_l_foot_pos, std::vector<double_t> init_l_foot_Q);
  void next();
  void finalize();
  bool set(double time, int step);

  double getLipmHeight();

  void calcFootStepParam();
  void calcFootTrajectory(int step);
  void calcFootStepPose(double time,  int step);
  void calcRefZMP(int step);
  void calcPreviewParam(thormang3_wholebody_module_msgs::PreviewResponse msg);
  void calcPreviewControl(double time, int step);

  void calcGoalFootPose();

  double calcRefZMPx(int step);
  double calcRefZMPy(int step);

  void getWalkingPosition(std::vector<double_t> &l_foot_pos,
                          std::vector<double_t> &r_foot_pos,
                          std::vector<double_t> &body_pos);
  void getWalkingVelocity(std::vector<double_t> &l_foot_vel,
                          std::vector<double_t> &r_foot_vel,
                          std::vector<double_t> &body_vel);
  void getWalkingAccleration(std::vector<double_t> &l_foot_accel,
                             std::vector<double_t> &r_foot_accel,
                             std::vector<double_t> &body_accel);
  void getWalkingOrientation(std::vector<double_t> &l_foot_Q,
                             std::vector<double_t> &r_foot_Q,
                             std::vector<double_t> &body_Q);
  void getLIPM(std::vector<double_t> &x_lipm, std::vector<double_t> &y_lipm);
  void getWalkingState(int &walking_leg, int &walking_phase);

protected:
  thormang3::KinematicsDynamics *robot_;

  robotis_framework::MinimumJerk *body_trajectory_;
  robotis_framework::MinimumJerkViaPoint *r_foot_tra_;
  robotis_framework::MinimumJerkViaPoint *l_foot_tra_;

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
  double foot_tra_max_z_;

  int foot_step_size_;
  thormang3_wholebody_module_msgs::FootStepCommand foot_step_command_;
  thormang3_wholebody_module_msgs::FootStepArray foot_step_param_;
  thormang3_wholebody_module_msgs::PreviewResponse preview_response_;

  // Preview Control
  int preview_size_;
  double preview_time_;
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
  double zmp_offset_x_, zmp_offset_y_;

  Eigen::MatrixXd goal_r_foot_pos_buffer_, goal_l_foot_pos_buffer_;
  Eigen::MatrixXd ref_zmp_buffer_;

  // Pose Information
  double init_body_yaw_angle_;

  std::vector<double_t> init_body_pos_, init_body_vel_, init_body_accel_;
  std::vector<double_t> des_body_pos_, des_body_vel_, des_body_accel_;
  std::vector<double_t> goal_body_pos_, goal_body_vel_, goal_body_accel_;
  Eigen::Quaterniond    init_body_Q_, des_body_Q_, goal_body_Q_;

  std::vector<double_t> init_l_foot_pos_, init_l_foot_vel_, init_l_foot_accel_;
  std::vector<double_t> des_l_foot_pos_, des_l_foot_vel_, des_l_foot_accel_;
  std::vector<double_t> goal_l_foot_pos_, goal_l_foot_vel_, goal_l_foot_accel_;
  Eigen::Quaterniond    init_l_foot_Q_, des_l_foot_Q_, goal_l_foot_Q_;

  std::vector<double_t> init_r_foot_pos_, init_r_foot_vel_, init_r_foot_accel_;
  std::vector<double_t> des_r_foot_pos_, des_r_foot_vel_, des_r_foot_accel_;
  std::vector<double_t> goal_r_foot_pos_, goal_r_foot_vel_, goal_r_foot_accel_;
  Eigen::Quaterniond    init_r_foot_Q_, des_r_foot_Q_, goal_r_foot_Q_;
};

#endif
