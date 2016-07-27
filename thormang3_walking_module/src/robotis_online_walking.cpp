/*******************************************************************************
 * Copyright (c) 2016, ROBOTIS CO., LTD.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of ROBOTIS nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

/*
 * robotis_onlinel_walking.cpp
 *
 *  Created on: 2016. 6. 10.
 *      Author: Jay Song
 */

#include <iostream>
#include <stdio.h>
#include "thormang3_walking_module/robotis_onlinel_walking.h"


using namespace thormang3;


static const double MMtoM = 0.001;
static const double MStoS = 0.001;

#define NO_STEP_IDX    (-1)
#define G              (9810.0*MMtoM) // mm/s^2(= 9.81m/s^2 *1000mm/1m)
#define TIME_UNIT      (8*MStoS)

static const int IN_WALKING_STARTING = 0;
static const int IN_WALKING = 1;
static const int IN_WALKING_ENDING = 2;

static const int LEFT_FOOT_SWING  = 1;
static const int RIGHT_FOOT_SWING = 2;
static const int STANDING = 3;

enum
{
  BALANCING_PHASE0 = 0,
  BALANCING_PHASE1 = 1,
  BALANCING_PHASE2 = 2,
  BALANCING_PHASE3 = 3,
  BALANCING_PHASE4 = 4,
  BALANCING_PHASE5 = 5,
  BALANCING_PHASE6 = 6,
  BALANCING_PHASE7 = 7,
  BALANCING_PHASE8 = 8,
  BALANCING_PHASE9 = 9
};

enum StepDataStatus
{
  StepDataStatus1 = 1, //
  StepDataStatus2 = 2, //
  StepDataStatus3 = 3, //
  StepDataStatus4 = 4, //
};

RobotisOnlineWalking::RobotisOnlineWalking()
{
  thormang3_kd_ = new KinematicsDynamics(WholeBody);

  present_right_foot_pose_.x = 0.0;    present_right_foot_pose_.y = -0.5*thormang3_kd_->leg_side_offset_m_;
  present_right_foot_pose_.z = -0.630*MMtoM;
  present_right_foot_pose_.roll = 0.0; present_right_foot_pose_.pitch = 0.0; present_right_foot_pose_.yaw = 0.0;

  present_left_foot_pose_.x = 0.0;    present_left_foot_pose_.y = 0.5*thormang3_kd_->leg_side_offset_m_;
  present_left_foot_pose_.z = -0.630*MMtoM;
  present_left_foot_pose_.roll = 0.0; present_left_foot_pose_.pitch = 0.0; present_left_foot_pose_.yaw = 0.0;

  present_body_pose_.x = 0.0;    present_body_pose_.y = 0.0;     present_body_pose_.z = 0.0;
  present_body_pose_.roll = 0.0; present_body_pose_.pitch = 0.0; present_body_pose_.yaw = 0;

  previous_step_right_foot_pose_  = present_right_foot_pose_;
  previous_step_left_foot_pose_   = present_left_foot_pose_;
  previous_step_body_pose_        = present_body_pose_;

  initial_right_foot_pose_ = previous_step_right_foot_pose_;
  initial_left_foot_pose_  = previous_step_left_foot_pose_;
  initial_body_pose_       = previous_step_body_pose_;

//  mat_cob_to_rhip_ = robotis_framework::getTranslation4D(0.0, -0.5*(186.0)*MMtoM, 0.0);
//  mat_rhip_to_cob_ = robotis_framework::getTranslation4D(0.0,  0.5*(186.0)*MMtoM, 0.0);
//  mat_cob_to_lhip_ = robotis_framework::getTranslation4D(0.0,  0.5*(186.0)*MMtoM, 0.0);
//  mat_lhip_to_cob_ = robotis_framework::getTranslation4D(0.0, -0.5*(186.0)*MMtoM, 0.0);

  mat_cob_to_rhip_ = robotis_framework::getTranslation4D(0.0,       thormang3_kd_->thormang3_link_data_[ID_R_LEG_START]->relative_position_.coeff(1, 0), 0.0);
  mat_rhip_to_cob_ = robotis_framework::getTranslation4D(0.0, -1.0*(thormang3_kd_->thormang3_link_data_[ID_R_LEG_START]->relative_position_.coeff(1, 0)), 0.0);
  mat_cob_to_lhip_ = robotis_framework::getTranslation4D(0.0,       thormang3_kd_->thormang3_link_data_[ID_L_LEG_START]->relative_position_.coeff(1, 0), 0.0);
  mat_lhip_to_cob_ = robotis_framework::getTranslation4D(0.0, -1.0*(thormang3_kd_->thormang3_link_data_[ID_L_LEG_START]->relative_position_.coeff(1, 0)), 0.0);

  mat_rfoot_to_rft_ = robotis_framework::getRotation4d(M_PI,0,0);
  mat_lfoot_to_lft_ = robotis_framework::getRotation4d(M_PI,0,0);
  rot_x_pi_3d_ = robotis_framework::getRotationX(M_PI);
  rot_z_pi_3d_ = robotis_framework::getRotationZ(M_PI);


  mat_g_to_cob_ = robotis_framework::getTransformationXYZRPY(present_body_pose_.x, present_body_pose_.y, present_body_pose_.z,
      present_body_pose_.roll, present_body_pose_.pitch, present_body_pose_.yaw);

  mat_cob_to_g_ = robotis_framework::getInverseTransformation(mat_g_to_cob_);

  mat_robot_to_cob_ = robotis_framework::getRotation4d(present_body_pose_.roll, present_body_pose_.pitch, 0);
  mat_cob_to_robot_ = robotis_framework::getInverseTransformation(mat_robot_to_cob_);
  mat_g_to_robot_   = mat_g_to_cob_ * mat_cob_to_robot_;
  mat_robot_to_g_   = robotis_framework::getInverseTransformation(mat_g_to_robot_);

  mat_g_to_rfoot_ = robotis_framework::getTransformationXYZRPY(present_right_foot_pose_.x, present_right_foot_pose_.y, present_right_foot_pose_.z,
      present_right_foot_pose_.roll, present_right_foot_pose_.pitch, present_right_foot_pose_.yaw);
  mat_g_to_lfoot_ = robotis_framework::getTransformationXYZRPY(present_left_foot_pose_.x, present_left_foot_pose_.y, present_left_foot_pose_.z,
      present_left_foot_pose_.roll, present_left_foot_pose_.pitch, present_left_foot_pose_.yaw);

  mat_robot_to_rfoot_ = mat_robot_to_g_ * mat_g_to_rfoot_;
  mat_robot_to_lfoot_ = mat_robot_to_g_ * mat_g_to_lfoot_;

  rhip_to_rfoot_pose_ = getPose3DfromTransformMatrix((mat_rhip_to_cob_ * mat_cob_to_robot_) * mat_robot_to_rfoot_);
  lhip_to_lfoot_pose_ = getPose3DfromTransformMatrix((mat_lhip_to_cob_ * mat_cob_to_robot_) * mat_robot_to_lfoot_);

  thormang3_kd_->calcInverseKinematicsForRightLeg(&r_leg_out_angle_rad_[0], rhip_to_rfoot_pose_.x, rhip_to_rfoot_pose_.y, rhip_to_rfoot_pose_.z, rhip_to_rfoot_pose_.roll, rhip_to_rfoot_pose_.pitch, rhip_to_rfoot_pose_.yaw);
  thormang3_kd_->calcInverseKinematicsForLeftLeg(&l_leg_out_angle_rad_[0], lhip_to_lfoot_pose_.x, lhip_to_lfoot_pose_.y, lhip_to_lfoot_pose_.z, lhip_to_lfoot_pose_.roll, lhip_to_lfoot_pose_.pitch, lhip_to_lfoot_pose_.yaw);


  r_shoulder_dir_ = thormang3_kd_->thormang3_link_data_[ID_R_ARM_START+2*0]->joint_axis_.coeff(1, 0);
  r_elbow_dir_    = thormang3_kd_->thormang3_link_data_[ID_R_ARM_START+2*3]->joint_axis_.coeff(2, 0);
  l_shoulder_dir_ = thormang3_kd_->thormang3_link_data_[ID_L_ARM_START+2*0]->joint_axis_.coeff(1, 0);
  l_elbow_dir_    = thormang3_kd_->thormang3_link_data_[ID_L_ARM_START+2*3]->joint_axis_.coeff(2, 0);


  r_init_shoulder_angle_rad_ = r_init_elbow_angle_rad_ = r_shoulder_out_angle_rad_ = r_elbow_out_angle_rad_ = 0;
  l_init_shoulder_angle_rad_ = l_init_elbow_angle_rad_ = l_shoulder_out_angle_rad_ = l_elbow_out_angle_rad_ = 0;

  goal_waist_yaw_angle_rad_ = 0.0*M_PI;

  reference_step_data_for_addition_.position_data.moving_foot = STANDING;
  reference_step_data_for_addition_.position_data.elbow_swing_gain = 0.1;
  reference_step_data_for_addition_.position_data.shoulder_swing_gain = 0.05;
  reference_step_data_for_addition_.position_data.foot_z_swap = 0.0;
  reference_step_data_for_addition_.position_data.waist_pitch_angle = 0.0;
  reference_step_data_for_addition_.position_data.waist_yaw_angle = goal_waist_yaw_angle_rad_;
  reference_step_data_for_addition_.position_data.body_z_swap = 0.0;
  reference_step_data_for_addition_.position_data.body_pose = previous_step_body_pose_;
  reference_step_data_for_addition_.position_data.right_foot_pose = previous_step_right_foot_pose_;
  reference_step_data_for_addition_.position_data.left_foot_pose = previous_step_left_foot_pose_;
  reference_step_data_for_addition_.time_data.walking_state = IN_WALKING_ENDING;
  reference_step_data_for_addition_.time_data.abs_step_time = 1.6;
  reference_step_data_for_addition_.time_data.dsp_ratio = 0.2;
  reference_step_data_for_addition_.time_data.sigmoid_ratio_x = 1.0;
  reference_step_data_for_addition_.time_data.sigmoid_ratio_y = 1.0;
  reference_step_data_for_addition_.time_data.sigmoid_ratio_z = 1.0;
  reference_step_data_for_addition_.time_data.sigmoid_ratio_roll = 1.0;
  reference_step_data_for_addition_.time_data.sigmoid_ratio_pitch = 1.0;
  reference_step_data_for_addition_.time_data.sigmoid_ratio_yaw = 1.0;
  reference_step_data_for_addition_.time_data.sigmoid_distortion_x = 1.0;
  reference_step_data_for_addition_.time_data.sigmoid_distortion_y = 1.0;
  reference_step_data_for_addition_.time_data.sigmoid_distortion_z = 1.0;
  reference_step_data_for_addition_.time_data.sigmoid_distortion_roll = 1.0;
  reference_step_data_for_addition_.time_data.sigmoid_distortion_pitch = 1.0;
  reference_step_data_for_addition_.time_data.sigmoid_distortion_yaw = 1.0;

  present_waist_yaw_angle_rad_ = goal_waist_yaw_angle_rad_;
  previous_step_waist_yaw_angle_rad_ = goal_waist_yaw_angle_rad_;

  shouler_swing_gain_    = 0.05;
  elbow_swing_gain_     = 0.1;

  real_running = false; ctrl_running = false;
  current_step_data_status_ = StepDataStatus4;

  walking_time_ = 0; reference_time_ = 0;
  balancing_index_ = 0;

  preview_time_ = 1.6;
  preview_size_ = round(preview_time_/TIME_UNIT);

  //These parameters are for preview control
  k_s_ = 0;
  current_start_idx_for_ref_zmp_ = 0;
  ref_zmp_x_at_this_time_ = 0;
  ref_zmp_y_at_this_time_ = 0;
  sum_of_zmp_x_ = 0;
  sum_of_zmp_y_ = 0;
  sum_of_cx_ = 0;
  sum_of_cy_ = 0;

  // variables for balance
  hip_roll_feedforward_angle_rad_ = 0;

  current_right_fx_N_  = current_right_fy_N_  = current_right_fz_N_  = 0;
  current_right_tx_Nm_ = current_right_ty_Nm_ = current_right_tz_Nm_ = 0;
  current_left_fx_N_  = current_left_fy_N_  = current_left_fz_N_  = 0;
  current_left_tx_Nm_ = current_left_ty_Nm_ = current_left_tz_Nm_ = 0;

  current_imu_roll_rad_ = current_imu_pitch_rad_ = 0;
  current_gyro_roll_rad_per_sec_ = current_gyro_pitch_rad_per_sec_ = 0;


  double total_mass_of_robot = thormang3_kd_->calcTotalMass(0);

  right_dsp_fz_N_ = -1.0*(total_mass_of_robot)*9.8*0.5;
  right_ssp_fz_N_ = -1.0*(total_mass_of_robot)*9.8;
  left_dsp_fz_N_  = -1.0*(total_mass_of_robot)*9.8*0.5;
  left_ssp_fz_N_  = -1.0*(total_mass_of_robot)*9.8;

  left_fz_sigmoid_start_time_ = 0;
  left_fz_sigmoid_end_time_  = 0;
  left_fz_sigmoid_target_  = left_dsp_fz_N_;
  left_fz_sigmoid_shift_   = left_dsp_fz_N_;


  balance_error_ = BalanceControlError::NoError;

}

RobotisOnlineWalking::~RobotisOnlineWalking()
{  }

bool RobotisOnlineWalking::setInitialPose(double r_foot_x, double r_foot_y, double r_foot_z, double r_foot_roll, double r_foot_pitch, double r_foot_yaw,
    double l_foot_x, double l_foot_y, double l_foot_z, double l_foot_roll, double l_foot_pitch, double l_foot_yaw,
    double center_of_body_x, double center_of_body_y, double center_of_body_z,
    double center_of_body_roll, double center_of_body_pitch, double center_of_body_yaw)
{

  if(real_running || ctrl_running)
    return false;

  previous_step_right_foot_pose_.x     = r_foot_x;
  previous_step_right_foot_pose_.y     = r_foot_y;
  previous_step_right_foot_pose_.z     = r_foot_z;
  previous_step_right_foot_pose_.roll  = r_foot_roll;
  previous_step_right_foot_pose_.pitch = r_foot_pitch;
  previous_step_right_foot_pose_.yaw   = r_foot_yaw;

  previous_step_left_foot_pose_.x     = l_foot_x;
  previous_step_left_foot_pose_.y     = l_foot_y;
  previous_step_left_foot_pose_.z     = l_foot_z;
  previous_step_left_foot_pose_.roll  = l_foot_roll;
  previous_step_left_foot_pose_.pitch = l_foot_pitch;
  previous_step_left_foot_pose_.yaw   = l_foot_yaw;

  previous_step_body_pose_.x     = center_of_body_x;
  previous_step_body_pose_.y     = center_of_body_y;
  previous_step_body_pose_.z     = center_of_body_z;
  previous_step_body_pose_.roll  = center_of_body_roll;
  previous_step_body_pose_.pitch = center_of_body_pitch;
  previous_step_body_pose_.yaw   = center_of_body_yaw;

  initial_right_foot_pose_ = previous_step_right_foot_pose_;
  initial_left_foot_pose_  = previous_step_left_foot_pose_;
  initial_body_pose_       = previous_step_body_pose_;

  return true;
}

void RobotisOnlineWalking::setInitalWaistYawAngle(double waist_yaw_angle_rad)
{
  goal_waist_yaw_angle_rad_ = waist_yaw_angle_rad;
}

void RobotisOnlineWalking::setInitialRightShoulderAngle(double shoulder_angle_rad)
{
  r_init_shoulder_angle_rad_ = shoulder_angle_rad;
}

void RobotisOnlineWalking::setInitialLeftShoulderAngle(double shoulder_angle_rad)
{
  l_init_shoulder_angle_rad_ = shoulder_angle_rad;
}

void RobotisOnlineWalking::setInitialRightElbowAngle(double elbow_angle_rad)
{
  r_init_elbow_angle_rad_ = elbow_angle_rad;
}

void RobotisOnlineWalking::setInitialLeftElbowAngle(double elbow_angle_rad)
{
  l_init_elbow_angle_rad_ = elbow_angle_rad;
}

void RobotisOnlineWalking::initialize()
{
  if(real_running)
    return;

  mutex_lock_.lock();
  added_step_data_.clear();

  // initialize balance
  balance_ctrl_.initialize(TIME_UNIT*1000.0);

  //Initialize Time
  walking_time_ = 0; reference_time_ = 0;

  present_right_foot_pose_ = previous_step_right_foot_pose_;
  present_left_foot_pose_  = previous_step_left_foot_pose_;
  present_body_pose_       = previous_step_body_pose_;

  mat_g_to_cob_ = robotis_framework::getTransformationXYZRPY(previous_step_body_pose_.x, previous_step_body_pose_.y, previous_step_body_pose_.z,
      previous_step_body_pose_.roll, previous_step_body_pose_.pitch, previous_step_body_pose_.yaw);

  mat_cob_to_g_ = robotis_framework::getInverseTransformation(mat_g_to_cob_);

  mat_robot_to_cob_ = robotis_framework::getRotation4d(previous_step_body_pose_.roll, previous_step_body_pose_.pitch, 0);
  mat_cob_to_robot_ = robotis_framework::getInverseTransformation(mat_robot_to_cob_);
  mat_g_to_robot_   = mat_g_to_cob_ * mat_cob_to_robot_;
  mat_robot_to_g_   = robotis_framework::getInverseTransformation(mat_g_to_robot_);


  mat_g_to_rfoot_ = robotis_framework::getTransformationXYZRPY(previous_step_right_foot_pose_.x, previous_step_right_foot_pose_.y, previous_step_right_foot_pose_.z,
      previous_step_right_foot_pose_.roll, previous_step_right_foot_pose_.pitch, previous_step_right_foot_pose_.yaw);
  mat_g_to_lfoot_ = robotis_framework::getTransformationXYZRPY(previous_step_left_foot_pose_.x, previous_step_left_foot_pose_.y, previous_step_left_foot_pose_.z,
      previous_step_left_foot_pose_.roll, previous_step_left_foot_pose_.pitch, previous_step_left_foot_pose_.yaw);

  mat_robot_to_rfoot_ = mat_robot_to_g_*mat_g_to_rfoot_;
  mat_robot_to_lfoot_ = mat_robot_to_g_*mat_g_to_lfoot_;
  //mat_rhip_to_rfoot_ = ((((mat_rhip_to_cob_ * mat_cob_to_robot_) * mat_cob_adjustment_ ) * mat_robot_to_cob_ ) * mat_cob_to_g_ ) * mat_g_to_rfoot_;
  //mat_lhip_to_lfoot_ = ((((mat_lhip_to_cob_ * mat_cob_to_robot_) * mat_cob_adjustment_ ) * mat_robot_to_cob_ ) * mat_cob_to_g_ ) * mat_g_to_lfoot_;

  balance_ctrl_.process(&balance_error_, &mat_cob_adjustment_, &mat_rfoot_adjustment_, &mat_lfoot_adjustment_);

  mat_robot_to_cob_modified_ = mat_robot_to_cob_*mat_cob_adjustment_;
  mat_cob_to_robot_modified_ = robotis_framework::getInverseTransformation(mat_robot_to_cob_modified_);
  mat_robot_to_rf_modified_  = mat_robot_to_g_*mat_g_to_rfoot_*mat_rfoot_adjustment_;
  mat_robot_to_lf_modified_  = mat_robot_to_g_*mat_g_to_lfoot_*mat_lfoot_adjustment_;

  rhip_to_rfoot_pose_ = getPose3DfromTransformMatrix((mat_rhip_to_cob_ * mat_cob_to_robot_modified_) * mat_robot_to_rf_modified_);
  lhip_to_lfoot_pose_ = getPose3DfromTransformMatrix((mat_lhip_to_cob_ * mat_cob_to_robot_modified_) * mat_robot_to_lf_modified_);

//  epr.x -=  balance_ctrl_.getCOBManualAdjustmentX();
//  epl.x -=  balance_ctrl_.getCOBManualAdjustmentX();

  if(thormang3_kd_->calcInverseKinematicsForRightLeg(&r_leg_out_angle_rad_[0], rhip_to_rfoot_pose_.x, rhip_to_rfoot_pose_.y, rhip_to_rfoot_pose_.z, rhip_to_rfoot_pose_.roll, rhip_to_rfoot_pose_.pitch, rhip_to_rfoot_pose_.yaw) == false)
  {
    printf("IK not Solved EPR : %f %f %f %f %f %f\n", rhip_to_rfoot_pose_.x, rhip_to_rfoot_pose_.y, rhip_to_rfoot_pose_.z, rhip_to_rfoot_pose_.roll, rhip_to_rfoot_pose_.pitch, rhip_to_rfoot_pose_.yaw);
    return;
  }

  if(thormang3_kd_->calcInverseKinematicsForLeftLeg(&l_leg_out_angle_rad_[0], lhip_to_lfoot_pose_.x, lhip_to_lfoot_pose_.y, lhip_to_lfoot_pose_.z, lhip_to_lfoot_pose_.roll, lhip_to_lfoot_pose_.pitch, lhip_to_lfoot_pose_.yaw) == false)
  {
    printf("IK not Solved EPL : %f %f %f %f %f %f\n", lhip_to_lfoot_pose_.x, lhip_to_lfoot_pose_.y, lhip_to_lfoot_pose_.z, lhip_to_lfoot_pose_.roll, lhip_to_lfoot_pose_.pitch, lhip_to_lfoot_pose_.yaw);
    return;
  }

  for(int angle_idx = 0; angle_idx < 6; angle_idx++)
  {
    out_angle_rad_[angle_idx+0] = r_leg_out_angle_rad_[angle_idx];
    out_angle_rad_[angle_idx+6] = l_leg_out_angle_rad_[angle_idx];
  }

  reference_step_data_for_addition_.position_data.moving_foot = STANDING;
  reference_step_data_for_addition_.position_data.elbow_swing_gain = 0.0;
  reference_step_data_for_addition_.position_data.shoulder_swing_gain = 0.0;
  reference_step_data_for_addition_.position_data.foot_z_swap = 0.0;
  reference_step_data_for_addition_.position_data.waist_pitch_angle = 0.0;
  reference_step_data_for_addition_.position_data.waist_yaw_angle = goal_waist_yaw_angle_rad_;
  reference_step_data_for_addition_.position_data.body_z_swap = 0.0;
  reference_step_data_for_addition_.position_data.body_pose = previous_step_body_pose_;
  reference_step_data_for_addition_.position_data.right_foot_pose = previous_step_right_foot_pose_;
  reference_step_data_for_addition_.position_data.left_foot_pose = previous_step_left_foot_pose_;
  reference_step_data_for_addition_.time_data.walking_state = IN_WALKING_ENDING;
  reference_step_data_for_addition_.time_data.abs_step_time = 0.0;
  reference_step_data_for_addition_.time_data.dsp_ratio = 0.2;
  reference_step_data_for_addition_.time_data.sigmoid_ratio_x = 1.0;
  reference_step_data_for_addition_.time_data.sigmoid_ratio_y = 1.0;
  reference_step_data_for_addition_.time_data.sigmoid_ratio_z = 1.0;
  reference_step_data_for_addition_.time_data.sigmoid_ratio_roll = 1.0;
  reference_step_data_for_addition_.time_data.sigmoid_ratio_pitch = 1.0;
  reference_step_data_for_addition_.time_data.sigmoid_ratio_yaw = 1.0;
  reference_step_data_for_addition_.time_data.sigmoid_distortion_x = 1.0;
  reference_step_data_for_addition_.time_data.sigmoid_distortion_y = 1.0;
  reference_step_data_for_addition_.time_data.sigmoid_distortion_z = 1.0;
  reference_step_data_for_addition_.time_data.sigmoid_distortion_roll = 1.0;
  reference_step_data_for_addition_.time_data.sigmoid_distortion_pitch = 1.0;
  reference_step_data_for_addition_.time_data.sigmoid_distortion_yaw = 1.0;

  present_waist_yaw_angle_rad_ = goal_waist_yaw_angle_rad_;
  previous_step_waist_yaw_angle_rad_ = goal_waist_yaw_angle_rad_;

  current_step_data_status_ = StepDataStatus4;

  // Initialize Matrix for Preview Control
  double t = 0;
  if(TIME_UNIT < 1.0)
    t=TIME_UNIT;
  else
    t=TIME_UNIT/1000.0;

  //double z_COM_mm = m_PreviousStepBodyPosition.z;
  A_.resize(3,3); b_.resize(3,1); c_.resize(1,3);
  A_ << 1,  t, t*t/2.0,
      0,  1,   t,
      0,  0,   1;
  b_(0,0) = t*t*t/6.0;
  b_(1,0) =   t*t/2.0;
  b_(2,0) =     t;

  c_(0,0) = 1; c_(0,1) = 0; c_(0,2) = -500.0*MMtoM/G;

  double Q_e = 1.0, R = 0.000001;//1.0e-6;
  preview_size_ = round(preview_time_/TIME_UNIT);

  step_idx_data_.resize(preview_size_);
  step_idx_data_.fill(NO_STEP_IDX);
  current_start_idx_for_ref_zmp_ = 0;
  reference_zmp_x_.resize(preview_size_, 1);
  reference_zmp_x_.fill(0.5*(present_right_foot_pose_.x + present_left_foot_pose_.x));
  reference_zmp_y_.resize(preview_size_, 1);
  reference_zmp_y_.fill(0.5*(present_right_foot_pose_.y + present_left_foot_pose_.y));

  k_s_ = 608.900142915471; //500.0

  k_x_.resize(1,3);
  k_x_ << 35904.1790662895, 8609.63092261379, 112.710622775482; // 500

  f_.resize(1, preview_size_);
  //500
  f_ << 608.9001429,  760.1988656,  921.629618,  1034.769891,   1089.313783,   1096.765451,  1074.295061,  1036.842257,   994.5569472,  953.0295724,
      914.5708045,  879.5752596,  847.5633512,  817.8237132,   789.7293917,   762.8419074,  736.9038288,  711.7889296,  687.4484508,  663.8698037,
      641.050971,   618.987811,   597.6697941,  577.0801943,   557.1979969,   537.9999833,  519.4623366,  501.5616312,  484.2753137,  467.581849,
      451.4606896,  435.8921765,  420.8574331,  406.3382777,   392.3171623,   378.7771307,  365.7017923,  353.0753019,  340.8823445,  329.1081203,
      317.7383294,  306.759157,   296.1572572,  285.919738,    276.0341458,   266.4884502,  257.2710302,  248.3706597,  239.7764944,  231.4780587,
      223.4652335,  215.7282439,  208.2576474,  201.0443231,   194.0794606,   187.3545495,  180.8613691,  174.5919788,  168.5387087,  162.6941501,
      157.0511468,  151.6027868,  146.3423936,  141.2635185,   136.3599326,   131.62562,    127.0547695,  122.6417688,  118.3811969,  114.2678179,
      110.2965751,  106.462584,   102.7611275,   99.18764938,   95.73774938,   92.40717757,  89.19182939,  86.08774068,  83.0910829,   80.19815849,
      77.40539648,  74.70934812,  72.10668274,  69.59418373,   67.16874468,   64.82736558,  62.56714924,  60.38529775,  58.27910913,  56.24597404,
      54.28337262,  52.38887145,  50.5601206,   48.79485075,   47.09087052,   45.44606371,  43.8583868,   42.32586646,  40.84659712,  39.4187387,
      38.04051434,  36.71020825,  35.42616363,  34.18678064,   32.99051448,   31.83587345,  30.72141721,  29.64575495,  28.60754377,  27.60548695,
      26.63833247,  25.70487138,  24.80393641,  23.93440049,   23.09517539,   22.28521038,  21.50349096,  20.74903761,  20.0209046,   19.3181788,
      18.63997859,  17.98545279,  17.35377958,  16.74416551,   16.15584454,   15.58807707,  15.04014906,  14.51137112,  14.00107771,  13.50862627,
      13.03339646,  12.57478939,  12.13222688,  11.70515076,   11.29302214,   10.89532081,  10.51154456,  10.14120854,   9.783844714,  9.439001248,
      9.106241955,  8.78514576,   8.475306179,  8.176330819,   7.887840885,   7.609470721,  7.340867346,  7.081690027,  6.83160985,   6.590309314,
      6.357481938,  6.132831881,  5.916073573,  5.706931359,   5.505139163,   5.310440149,  5.122586408,  4.941338646,  4.766465888,  4.597745188,
      4.434961356,  4.277906685,  4.126380694,  3.980189879,   3.839147471,   3.703073201,  3.571793078,  3.445139169,  3.322949391,  3.205067309,
      3.091341936,  2.981627549,  2.875783507,  2.773674068,   2.675168228,   2.58013955,   2.488466009,  2.400029838,  2.314717381,  2.232418947,
      2.153028678,  2.076444411,  2.002567552,  1.931302952,   1.862558786,   1.796246438,  1.732280393,  1.670578121,  1.611059983,  1.553649123,
      1.498271374,  1.444855165,  1.393331431,  1.343633522,   1.295697125,   1.249460178,  1.204862791,  1.161847176,  1.120357568,  1.080340156;

  u_x.resize(1,1);
  u_y.resize(1,1);

  x_lipm_.resize(3, 1);    y_lipm_.resize(3, 1);
  x_lipm_.fill(0.0);       y_lipm_.fill(0.0);

  mutex_lock_.unlock();

  //printf("RLEG : %f %f %f %f %f %f\n", m_OutAngleRad[0]*180.0/M_PI, m_OutAngleRad[1]*180.0/M_PI, m_OutAngleRad[2]*180.0/M_PI,
   //   m_OutAngleRad[3]*180.0/M_PI, m_OutAngleRad[4]*180.0/M_PI, m_OutAngleRad[5]*180.0/M_PI);
  //printf("LLEG : %f %f %f %f %f %f\n", m_OutAngleRad[6]*180.0/M_PI, m_OutAngleRad[7]*180.0/M_PI, m_OutAngleRad[8]*180.0/M_PI,
  //    m_OutAngleRad[9]*180.0/M_PI, m_OutAngleRad[10]*180.0/M_PI, m_OutAngleRad[11]*180.0/M_PI);

//  out_angle_rad_[12] = ((double)dir_output_[12]*(epr.x - epl.x)*shouler_swing_gain_ + init_angle_[12])*M_PI/180.0;
//  out_angle_rad_[13] = ((double)dir_output_[13]*(epl.x - epr.x)*shouler_swing_gain_ + init_angle_[13])*M_PI/180.0;
//  out_angle_rad_[14] = ((double)dir_output_[14]*(epr.x - epl.x)*elbow_swing_gain_   + init_angle_[14])*M_PI/180.0;
//  out_angle_rad_[15] = ((double)dir_output_[15]*(epl.x - epr.x)*elbow_swing_gain_   + init_angle_[15])*M_PI/180.0;

  r_shoulder_out_angle_rad_ = r_shoulder_dir_*(mat_robot_to_rfoot_.coeff(0, 3) - mat_robot_to_lfoot_.coeff(0, 3))*shouler_swing_gain_ + r_init_shoulder_angle_rad_;
  l_shoulder_out_angle_rad_ = l_shoulder_dir_*(mat_robot_to_lfoot_.coeff(0, 3) - mat_robot_to_rfoot_.coeff(0, 3))*shouler_swing_gain_ + l_init_shoulder_angle_rad_;
  r_elbow_out_angle_rad_ = r_elbow_dir_*(mat_robot_to_rfoot_.coeff(0, 3) - mat_robot_to_lfoot_.coeff(0, 3))*elbow_swing_gain_ + r_init_elbow_angle_rad_;
  l_elbow_out_angle_rad_ = l_elbow_dir_*(mat_robot_to_lfoot_.coeff(0, 3) - mat_robot_to_rfoot_.coeff(0, 3))*elbow_swing_gain_ + l_init_elbow_angle_rad_;

  left_fz_sigmoid_start_time_ = 0;
  left_fz_sigmoid_end_time_  = 0;
  left_fz_sigmoid_target_  = left_dsp_fz_N_;
  left_fz_sigmoid_shift_   = left_dsp_fz_N_;

}

void RobotisOnlineWalking::reInitialize()
{
  if(real_running)
    return;

  mutex_lock_.lock();
  added_step_data_.clear();

  //Initialize Time
  walking_time_ = 0; reference_time_ = 0;

  previous_step_right_foot_pose_ = getPose3DfromTransformMatrix(mat_robot_to_rfoot_);
  previous_step_left_foot_pose_  = getPose3DfromTransformMatrix(mat_robot_to_lfoot_);
  previous_step_body_pose_ = getPose3DfromTransformMatrix(mat_robot_to_cob_);

  present_right_foot_pose_ = previous_step_right_foot_pose_;
  present_left_foot_pose_  = previous_step_left_foot_pose_;
  present_body_pose_       = previous_step_body_pose_;

  mat_g_to_cob_ = robotis_framework::getTransformationXYZRPY(previous_step_body_pose_.x, previous_step_body_pose_.y, previous_step_body_pose_.z,
      previous_step_body_pose_.roll, previous_step_body_pose_.pitch, previous_step_body_pose_.yaw);

  mat_cob_to_g_ = robotis_framework::getInverseTransformation(mat_g_to_cob_);
  mat_g_to_robot_ = mat_g_to_cob_ * mat_cob_to_robot_;
  mat_robot_to_g_ = robotis_framework::getInverseTransformation(mat_g_to_robot_);

  mat_g_to_rfoot_ = robotis_framework::getTransformationXYZRPY(previous_step_right_foot_pose_.x, previous_step_right_foot_pose_.y, previous_step_right_foot_pose_.z,
      previous_step_right_foot_pose_.roll, previous_step_right_foot_pose_.pitch, previous_step_right_foot_pose_.yaw);
  mat_g_to_lfoot_ = robotis_framework::getTransformationXYZRPY(previous_step_left_foot_pose_.x, previous_step_left_foot_pose_.y, previous_step_left_foot_pose_.z,
      previous_step_left_foot_pose_.roll, previous_step_left_foot_pose_.pitch, previous_step_left_foot_pose_.yaw);

  mat_robot_to_rfoot_ = mat_rhip_to_cob_*mat_cob_to_g_*mat_g_to_rfoot_;
  mat_robot_to_lfoot_ = mat_lhip_to_cob_*mat_cob_to_g_*mat_g_to_lfoot_;

  balance_ctrl_.process(&balance_error_, &mat_cob_adjustment_, &mat_rfoot_adjustment_, &mat_lfoot_adjustment_);
  mat_robot_to_cob_modified_ = mat_robot_to_cob_*mat_cob_adjustment_;
  mat_cob_to_robot_modified_ = robotis_framework::getInverseTransformation(mat_robot_to_cob_modified_);
  mat_robot_to_rf_modified_  = mat_robot_to_g_*mat_g_to_rfoot_*mat_rfoot_adjustment_;
  mat_robot_to_lf_modified_  = mat_robot_to_g_*mat_g_to_lfoot_*mat_lfoot_adjustment_;

  rhip_to_rfoot_pose_ = getPose3DfromTransformMatrix((mat_rhip_to_cob_ * mat_cob_to_robot_modified_) * mat_robot_to_rf_modified_);
  lhip_to_lfoot_pose_ = getPose3DfromTransformMatrix((mat_lhip_to_cob_ * mat_cob_to_robot_modified_) * mat_robot_to_lf_modified_);

  if(thormang3_kd_->calcInverseKinematicsForRightLeg(&r_leg_out_angle_rad_[0], rhip_to_rfoot_pose_.x, rhip_to_rfoot_pose_.y, rhip_to_rfoot_pose_.z, rhip_to_rfoot_pose_.roll, rhip_to_rfoot_pose_.pitch, rhip_to_rfoot_pose_.yaw) == false)
  {
    printf("IK not Solved EPR : %f %f %f %f %f %f\n", rhip_to_rfoot_pose_.x, rhip_to_rfoot_pose_.y, rhip_to_rfoot_pose_.z, rhip_to_rfoot_pose_.roll, rhip_to_rfoot_pose_.pitch, rhip_to_rfoot_pose_.yaw);
    return;
  }

  if(thormang3_kd_->calcInverseKinematicsForLeftLeg(&l_leg_out_angle_rad_[0], lhip_to_lfoot_pose_.x, lhip_to_lfoot_pose_.y, lhip_to_lfoot_pose_.z, lhip_to_lfoot_pose_.roll, lhip_to_lfoot_pose_.pitch, lhip_to_lfoot_pose_.yaw) == false)
  {
    printf("IK not Solved EPL : %f %f %f %f %f %f\n", lhip_to_lfoot_pose_.x, lhip_to_lfoot_pose_.y, lhip_to_lfoot_pose_.z, lhip_to_lfoot_pose_.roll, lhip_to_lfoot_pose_.pitch, lhip_to_lfoot_pose_.yaw);
    return;
  }

  for(int angle_idx = 0; angle_idx < 6; angle_idx++)
  {
    out_angle_rad_[angle_idx+0] = r_leg_out_angle_rad_[angle_idx];
    out_angle_rad_[angle_idx+6] = l_leg_out_angle_rad_[angle_idx];
  }

  reference_step_data_for_addition_.position_data.moving_foot = STANDING;
  reference_step_data_for_addition_.position_data.elbow_swing_gain = 0.0;
  reference_step_data_for_addition_.position_data.shoulder_swing_gain = 0.0;
  reference_step_data_for_addition_.position_data.foot_z_swap = 0.0;
  reference_step_data_for_addition_.position_data.waist_pitch_angle = 0.0;
  reference_step_data_for_addition_.position_data.waist_yaw_angle = goal_waist_yaw_angle_rad_;
  reference_step_data_for_addition_.position_data.body_z_swap = 0.0;
  reference_step_data_for_addition_.position_data.body_pose = previous_step_body_pose_;
  reference_step_data_for_addition_.position_data.right_foot_pose = previous_step_right_foot_pose_;
  reference_step_data_for_addition_.position_data.left_foot_pose = previous_step_left_foot_pose_;
  reference_step_data_for_addition_.time_data.walking_state = IN_WALKING_ENDING;
  reference_step_data_for_addition_.time_data.abs_step_time = 0.0;
  reference_step_data_for_addition_.time_data.dsp_ratio = 0.2;
  reference_step_data_for_addition_.time_data.sigmoid_ratio_x = 1.0;
  reference_step_data_for_addition_.time_data.sigmoid_ratio_y = 1.0;
  reference_step_data_for_addition_.time_data.sigmoid_ratio_z = 1.0;
  reference_step_data_for_addition_.time_data.sigmoid_ratio_roll = 1.0;
  reference_step_data_for_addition_.time_data.sigmoid_ratio_pitch = 1.0;
  reference_step_data_for_addition_.time_data.sigmoid_ratio_yaw = 1.0;
  reference_step_data_for_addition_.time_data.sigmoid_distortion_x = 1.0;
  reference_step_data_for_addition_.time_data.sigmoid_distortion_y = 1.0;
  reference_step_data_for_addition_.time_data.sigmoid_distortion_z = 1.0;
  reference_step_data_for_addition_.time_data.sigmoid_distortion_roll = 1.0;
  reference_step_data_for_addition_.time_data.sigmoid_distortion_pitch = 1.0;
  reference_step_data_for_addition_.time_data.sigmoid_distortion_yaw = 1.0;

  present_waist_yaw_angle_rad_ = goal_waist_yaw_angle_rad_;
  previous_step_waist_yaw_angle_rad_ = goal_waist_yaw_angle_rad_;

  current_step_data_status_ = StepDataStatus4;

  step_idx_data_.fill(NO_STEP_IDX);
  current_start_idx_for_ref_zmp_ = 0;
  reference_zmp_x_.fill(0.5*(present_right_foot_pose_.x + present_left_foot_pose_.x));
  reference_zmp_y_.fill(0.5*(present_right_foot_pose_.y + present_left_foot_pose_.y));
  sum_of_zmp_x_ = 0.0;
  sum_of_zmp_y_ = 0.0;

  sum_of_cx_ = 0.0;
  sum_of_cy_ = 0.0;
  x_lipm_.fill(0.0);        y_lipm_.fill(0.0);

  mutex_lock_.unlock();

  left_fz_sigmoid_start_time_ = 0;
  left_fz_sigmoid_end_time_  = 0;
  left_fz_sigmoid_target_  = left_dsp_fz_N_;
  left_fz_sigmoid_shift_   = left_dsp_fz_N_;

}

void RobotisOnlineWalking::start()
{
  ctrl_running = true;
  real_running = true;
}

void RobotisOnlineWalking::stop()
{
  ctrl_running = false;
}

bool RobotisOnlineWalking::isRunning()
{
  return real_running;
}

bool RobotisOnlineWalking::addStepData(StepData step_data)
{
  mutex_lock_.lock();
  added_step_data_.push_back(step_data);

  calcStepIdxData();
  mutex_lock_.unlock();

  return true;
}

int RobotisOnlineWalking::getNumofRemainingUnreservedStepData()
{
  int step_idx = step_idx_data_(preview_size_ - 1);
  int remain_step_num = 0;
  if(step_idx != NO_STEP_IDX)
  {
    remain_step_num = (added_step_data_.size() - 1 - step_idx);
  }
  else
  {
    remain_step_num = 0;
  }
  return remain_step_num;
}

void RobotisOnlineWalking::eraseLastStepData()
{
  mutex_lock_.lock();
  if(getNumofRemainingUnreservedStepData() != 0)
  {
    added_step_data_.pop_back();
  }
  mutex_lock_.unlock();
}

void RobotisOnlineWalking::getReferenceStepDatafotAddition(StepData *ref_step_data_for_addition)
{
  reference_step_data_for_addition_.time_data.sigmoid_ratio_x = 1.0;
  reference_step_data_for_addition_.time_data.sigmoid_ratio_y = 1.0;
  reference_step_data_for_addition_.time_data.sigmoid_ratio_z = 1.0;
  reference_step_data_for_addition_.time_data.sigmoid_ratio_roll = 1.0;
  reference_step_data_for_addition_.time_data.sigmoid_ratio_pitch = 1.0;
  reference_step_data_for_addition_.time_data.sigmoid_ratio_yaw = 1.0;
  reference_step_data_for_addition_.time_data.sigmoid_distortion_x = 1.0;
  reference_step_data_for_addition_.time_data.sigmoid_distortion_y = 1.0;
  reference_step_data_for_addition_.time_data.sigmoid_distortion_z = 1.0;
  reference_step_data_for_addition_.time_data.sigmoid_distortion_roll = 1.0;
  reference_step_data_for_addition_.time_data.sigmoid_distortion_pitch = 1.0;
  reference_step_data_for_addition_.time_data.sigmoid_distortion_yaw = 1.0;
  (*ref_step_data_for_addition) = reference_step_data_for_addition_;
}

void RobotisOnlineWalking::calcStepIdxData()
{
  unsigned int step_idx = 0, previous_step_idx = 0;
  unsigned int step_data_size = added_step_data_.size();
  if(added_step_data_.size() == 0)
  {
    step_idx_data_.fill(NO_STEP_IDX);
    current_step_data_status_ = StepDataStatus4;
    real_running = false;
  }
  else
  {
    if(walking_time_ >= added_step_data_[0].time_data.abs_step_time - 0.5*MStoS)
    {
      previous_step_waist_yaw_angle_rad_ = added_step_data_[0].position_data.waist_yaw_angle;
      previous_step_left_foot_pose_ = added_step_data_[0].position_data.left_foot_pose;
      previous_step_right_foot_pose_ = added_step_data_[0].position_data.right_foot_pose;
      previous_step_body_pose_ = added_step_data_[0].position_data.body_pose;
      previous_step_body_pose_.x = present_body_pose_.x;
      previous_step_body_pose_.y = present_body_pose_.y;
      reference_time_ = added_step_data_[0].time_data.abs_step_time;
      added_step_data_.erase(added_step_data_.begin());
      if(added_step_data_.size() == 0)
      {
        step_idx_data_.fill(NO_STEP_IDX);
        current_step_data_status_ = StepDataStatus4;
        real_running = false;
      }
      else
      {
        for(int idx = 0; idx < preview_size_; idx++)
        {
          //Get STepIDx
          if(walking_time_ + (idx+1)*TIME_UNIT > added_step_data_[step_data_size -1].time_data.abs_step_time)
            step_idx_data_(idx) = NO_STEP_IDX;
          else
          {
            for(step_idx = previous_step_idx; step_idx < step_data_size; step_idx++)
            {
              if(walking_time_ + (idx+1)*TIME_UNIT <= added_step_data_[step_idx].time_data.abs_step_time)
                break;
            }
            step_idx_data_(idx) = step_idx;
            previous_step_idx = step_idx;
          }
        }
      }
    }
    else
    {
      for(int idx = 0; idx < preview_size_; idx++)
      {
        //Get StepIdx
        if(walking_time_ + (idx+1)*TIME_UNIT > added_step_data_[step_data_size -1].time_data.abs_step_time)
          step_idx_data_(idx) = NO_STEP_IDX;
        else
        {
          for(step_idx = previous_step_idx; step_idx < step_data_size; step_idx++)
          {
            if(walking_time_ + (idx+1)*TIME_UNIT <= added_step_data_[step_idx].time_data.abs_step_time)
              break;
          }
          step_idx_data_(idx) = step_idx;
          previous_step_idx = step_idx;
        }
      }
    }
  }

  if(step_idx_data_(preview_size_ - 1) != NO_STEP_IDX)
  {
    if(getNumofRemainingUnreservedStepData() != 0)
    {
      current_step_data_status_ = StepDataStatus1;
      reference_step_data_for_addition_ = added_step_data_[step_idx_data_(preview_size_-1)];
    }
    else
    {
      current_step_data_status_ = StepDataStatus2;
      reference_step_data_for_addition_ = added_step_data_[step_idx_data_(preview_size_-1)];
    }
  }
  else
  {
    if(step_idx_data_(0) != NO_STEP_IDX)
    {
      reference_step_data_for_addition_ = added_step_data_[step_idx_data_(0)];
      reference_step_data_for_addition_.time_data.walking_state = IN_WALKING_ENDING;
      reference_step_data_for_addition_.time_data.abs_step_time += preview_time_;

      current_step_data_status_ = StepDataStatus3;
    }
    else
    {
      reference_step_data_for_addition_.time_data.walking_state = IN_WALKING_ENDING;
      reference_step_data_for_addition_.time_data.abs_step_time = walking_time_ + preview_time_;
      current_step_data_status_ = StepDataStatus4;
    }
  }
}

void RobotisOnlineWalking::calcRefZMP()
{
  int ref_zmp_idx = 0;
  int step_idx = 0;
  if(walking_time_ == 0)
  {
    if((step_idx_data_(ref_zmp_idx) == NO_STEP_IDX)/* && (m_StepData.size() == 0)*/)
    {
      reference_zmp_x_.fill((present_left_foot_pose_.x + present_right_foot_pose_.x)*0.5);
      reference_zmp_y_.fill((present_left_foot_pose_.y + present_right_foot_pose_.y)*0.5);
      return;
    }

    for(ref_zmp_idx = 0; ref_zmp_idx < preview_size_;  ref_zmp_idx++)
    {
      step_idx = step_idx_data_(ref_zmp_idx);
      if(step_idx == NO_STEP_IDX)
      {
        reference_zmp_x_(ref_zmp_idx, 0) = reference_zmp_x_(ref_zmp_idx - 1, 0);
        reference_zmp_y_(ref_zmp_idx, 0) = reference_zmp_y_(ref_zmp_idx - 1, 0);
      }
      else
      {
        if(added_step_data_[step_idx].time_data.walking_state == IN_WALKING)
        {
          if( added_step_data_[step_idx].position_data.moving_foot == RIGHT_FOOT_SWING )
          {
            reference_zmp_x_(ref_zmp_idx, 0) = added_step_data_[step_idx].position_data.left_foot_pose.x;
            reference_zmp_y_(ref_zmp_idx, 0) = added_step_data_[step_idx].position_data.left_foot_pose.y;
          }
          else if( added_step_data_[step_idx].position_data.moving_foot == LEFT_FOOT_SWING )
          {
            reference_zmp_x_(ref_zmp_idx, 0) = added_step_data_[step_idx].position_data.right_foot_pose.x;
            reference_zmp_y_(ref_zmp_idx, 0) = added_step_data_[step_idx].position_data.right_foot_pose.y;
          }
          else if( added_step_data_[step_idx].position_data.moving_foot == STANDING )
          {
            reference_zmp_x_(ref_zmp_idx, 0) = (added_step_data_[step_idx].position_data.left_foot_pose.x + added_step_data_[step_idx].position_data.right_foot_pose.x)*0.5;
            reference_zmp_y_(ref_zmp_idx, 0) = (added_step_data_[step_idx].position_data.left_foot_pose.y + added_step_data_[step_idx].position_data.right_foot_pose.y)*0.5;
          }
          else
          {
            reference_zmp_x_(ref_zmp_idx, 0) = (added_step_data_[step_idx].position_data.left_foot_pose.x + added_step_data_[step_idx].position_data.right_foot_pose.x)*0.5;
            reference_zmp_y_(ref_zmp_idx, 0) = (added_step_data_[step_idx].position_data.left_foot_pose.y + added_step_data_[step_idx].position_data.right_foot_pose.y)*0.5;
          }
        }
        else if(added_step_data_[step_idx].time_data.walking_state == IN_WALKING_STARTING)
        {
          reference_zmp_x_(ref_zmp_idx, 0) = (added_step_data_[step_idx].position_data.left_foot_pose.x + added_step_data_[step_idx].position_data.right_foot_pose.x)*0.5;
          reference_zmp_y_(ref_zmp_idx, 0) = (added_step_data_[step_idx].position_data.left_foot_pose.y + added_step_data_[step_idx].position_data.right_foot_pose.y)*0.5;
        }
        else if(added_step_data_[step_idx].time_data.walking_state == IN_WALKING_ENDING)
        {
          reference_zmp_x_(ref_zmp_idx, 0) = (added_step_data_[step_idx].position_data.left_foot_pose.x + added_step_data_[step_idx].position_data.right_foot_pose.x)*0.5;
          reference_zmp_y_(ref_zmp_idx, 0) = (added_step_data_[step_idx].position_data.left_foot_pose.y + added_step_data_[step_idx].position_data.right_foot_pose.y)*0.5;
        }
        else
        {
          reference_zmp_x_(ref_zmp_idx, 0) = (added_step_data_[step_idx].position_data.left_foot_pose.x + added_step_data_[step_idx].position_data.right_foot_pose.x)*0.5;
          reference_zmp_y_(ref_zmp_idx, 0) = (added_step_data_[step_idx].position_data.left_foot_pose.y + added_step_data_[step_idx].position_data.right_foot_pose.y)*0.5;
        }
      }
    }
    current_start_idx_for_ref_zmp_ = 0;
  }
  else
  {
    step_idx = step_idx_data_(preview_size_ - 1);

    if(current_start_idx_for_ref_zmp_ == 0)
      ref_zmp_idx = preview_size_ - 1;
    else
      ref_zmp_idx = current_start_idx_for_ref_zmp_ - 1;

    if(step_idx == NO_STEP_IDX)
    {
      reference_zmp_x_(ref_zmp_idx, 0) = 0.5*(reference_step_data_for_addition_.position_data.right_foot_pose.x + reference_step_data_for_addition_.position_data.left_foot_pose.x);
      reference_zmp_y_(ref_zmp_idx, 0) = 0.5*(reference_step_data_for_addition_.position_data.right_foot_pose.y + reference_step_data_for_addition_.position_data.left_foot_pose.y);
    }
    else
    {
      if(added_step_data_[step_idx].time_data.walking_state == IN_WALKING)
      {
        if( added_step_data_[step_idx].position_data.moving_foot == RIGHT_FOOT_SWING )
        {
          reference_zmp_x_(ref_zmp_idx, 0) = added_step_data_[step_idx].position_data.left_foot_pose.x;
          reference_zmp_y_(ref_zmp_idx, 0) = added_step_data_[step_idx].position_data.left_foot_pose.y;
        }
        else if( added_step_data_[step_idx].position_data.moving_foot == LEFT_FOOT_SWING )
        {
          reference_zmp_x_(ref_zmp_idx, 0) = added_step_data_[step_idx].position_data.right_foot_pose.x;
          reference_zmp_y_(ref_zmp_idx, 0) = added_step_data_[step_idx].position_data.right_foot_pose.y;
        }
        else if( added_step_data_[step_idx].position_data.moving_foot == STANDING )
        {
          reference_zmp_x_(ref_zmp_idx, 0) = (added_step_data_[step_idx].position_data.left_foot_pose.x + added_step_data_[step_idx].position_data.right_foot_pose.x)*0.5;
          reference_zmp_y_(ref_zmp_idx, 0) = (added_step_data_[step_idx].position_data.left_foot_pose.y + added_step_data_[step_idx].position_data.right_foot_pose.y)*0.5;
        }
        else
        {
          reference_zmp_x_(ref_zmp_idx, 0) = (added_step_data_[step_idx].position_data.left_foot_pose.x + added_step_data_[step_idx].position_data.right_foot_pose.x)*0.5;
          reference_zmp_y_(ref_zmp_idx, 0) = (added_step_data_[step_idx].position_data.left_foot_pose.y + added_step_data_[step_idx].position_data.right_foot_pose.y)*0.5;
        }
      }
      else if(added_step_data_[step_idx].time_data.walking_state == IN_WALKING_STARTING)
      {
        reference_zmp_x_(ref_zmp_idx, 0) = (added_step_data_[step_idx].position_data.left_foot_pose.x + added_step_data_[step_idx].position_data.right_foot_pose.x)*0.5;
        reference_zmp_y_(ref_zmp_idx, 0) = (added_step_data_[step_idx].position_data.left_foot_pose.y + added_step_data_[step_idx].position_data.right_foot_pose.y)*0.5;
      }
      else if(added_step_data_[step_idx].time_data.walking_state == IN_WALKING_ENDING)
      {
        reference_zmp_x_(ref_zmp_idx, 0) = (added_step_data_[step_idx].position_data.left_foot_pose.x + added_step_data_[step_idx].position_data.right_foot_pose.x)*0.5;
        reference_zmp_y_(ref_zmp_idx, 0) = (added_step_data_[step_idx].position_data.left_foot_pose.y + added_step_data_[step_idx].position_data.right_foot_pose.y)*0.5;
      }
      else
      {
        reference_zmp_x_(ref_zmp_idx, 0) = (added_step_data_[step_idx].position_data.left_foot_pose.x + added_step_data_[step_idx].position_data.right_foot_pose.x)*0.5;
        reference_zmp_y_(ref_zmp_idx, 0) = (added_step_data_[step_idx].position_data.left_foot_pose.y + added_step_data_[step_idx].position_data.right_foot_pose.y)*0.5;
      }
    }
  }
}

void RobotisOnlineWalking::calcDesiredPose()
{
  calcStepIdxData();
  calcRefZMP();
  //        //Original LIPM
  //        u_x = -K*x_LIPM + x_feed_forward_term;
  //        x_LIPM = A*x_LIPM + b*u_x;
  //
  //        u_y = -K*y_LIPM + y_feed_forward_term;
  //        y_LIPM = A*y_LIPM + b*u_y;

  //Calc LIPM with Integral
  Eigen::MatrixXd  x_feed_forward_term2; //f_Preview*m_ZMP_Reference_X;
  Eigen::MatrixXd  y_feed_forward_term2; //f_Preview*m_ZMP_Reference_Y;

  x_feed_forward_term2.resize(1,1);
  x_feed_forward_term2.fill(0.0);
  y_feed_forward_term2.resize(1,1);
  y_feed_forward_term2.fill(0.0);

  for(int i = 0; i < preview_size_; i++)
  {
    if(current_start_idx_for_ref_zmp_ + i < preview_size_) {
      x_feed_forward_term2(0,0) += f_(i)*reference_zmp_x_(current_start_idx_for_ref_zmp_ + i, 0);
      y_feed_forward_term2(0,0) += f_(i)*reference_zmp_y_(current_start_idx_for_ref_zmp_ + i, 0);
    }
    else {
      x_feed_forward_term2(0,0) += f_(i)*reference_zmp_x_(current_start_idx_for_ref_zmp_ + i - preview_size_, 0);
      y_feed_forward_term2(0,0) += f_(i)*reference_zmp_y_(current_start_idx_for_ref_zmp_ + i - preview_size_, 0);
    }
  }

  sum_of_cx_ += c_(0,0)*x_lipm_(0,0) +  c_(0,1)*x_lipm_(1,0) +  c_(0,2)*x_lipm_(2,0);
  sum_of_cy_ += c_(0,0)*y_lipm_(0,0) +  c_(0,1)*y_lipm_(1,0) +  c_(0,2)*y_lipm_(2,0);

  u_x(0,0) = -k_s_*(sum_of_cx_ - sum_of_zmp_x_) - (k_x_(0,0)*x_lipm_(0,0) + k_x_(0,1)*x_lipm_(1,0) + k_x_(0,2)*x_lipm_(2,0)) + x_feed_forward_term2(0,0);
  u_y(0,0) = -k_s_*(sum_of_cy_ - sum_of_zmp_y_) - (k_x_(0,0)*y_lipm_(0,0) + k_x_(0,1)*y_lipm_(1,0) + k_x_(0,2)*y_lipm_(2,0)) + y_feed_forward_term2(0,0);
  x_lipm_ = A_*x_lipm_ + b_*u_x;
  y_lipm_ = A_*y_lipm_ + b_*u_y;


  ref_zmp_x_at_this_time_ = reference_zmp_x_(current_start_idx_for_ref_zmp_, 0);
  ref_zmp_y_at_this_time_ = reference_zmp_y_(current_start_idx_for_ref_zmp_, 0);

  sum_of_zmp_x_ += reference_zmp_x_(current_start_idx_for_ref_zmp_, 0);
  sum_of_zmp_y_ += reference_zmp_y_(current_start_idx_for_ref_zmp_, 0);

  present_body_pose_.x = x_lipm_(0,0);
  present_body_pose_.y = y_lipm_(0,0);

  reference_step_data_for_addition_.position_data.body_pose.x = x_lipm_(0,0);
  reference_step_data_for_addition_.position_data.body_pose.y = y_lipm_(0,0);

  current_start_idx_for_ref_zmp_++;
  if(current_start_idx_for_ref_zmp_ == (preview_size_))
    current_start_idx_for_ref_zmp_ = 0;
}

void RobotisOnlineWalking::process()
{
  if(!ctrl_running)
  {
    return;
  }
  else
  {
    mutex_lock_.lock();

    calcDesiredPose();

    double body_roll_swap = 0;
    int detail_balance_time_idx = 0;
    if((added_step_data_.size() != 0) && real_running)
    {
      double body_move_periodTime = added_step_data_[0].time_data.abs_step_time - reference_time_;
      double wp_move_amp = added_step_data_[0].position_data.waist_yaw_angle - previous_step_waist_yaw_angle_rad_;
      double wp_move_amp_shift = previous_step_waist_yaw_angle_rad_;

      double bz_move_amp = added_step_data_[0].position_data.body_pose.z - previous_step_body_pose_.z;
      double bz_move_amp_shift = previous_step_body_pose_.z;

      double ba_move_amp = added_step_data_[0].position_data.body_pose.roll - previous_step_body_pose_.roll;
      double ba_move_amp_shift = previous_step_body_pose_.roll;

      double bb_move_amp = added_step_data_[0].position_data.body_pose.pitch - previous_step_body_pose_.pitch;
      double bb_move_amp_shift = previous_step_body_pose_.pitch;

      double bc_move_amp = added_step_data_[0].position_data.body_pose.yaw - previous_step_body_pose_.yaw;
      double bc_move_amp_shift = previous_step_body_pose_.yaw;

      double z_swap_amp = 0.5*(added_step_data_[0].position_data.body_z_swap);
      double z_swap_amp_shift = z_swap_amp;
      double z_swap_phase_shift = M_PI*0.5;

      double body_roll_swap_dir = 1.0;
      double body_roll_swap_amp = 0.5*(hip_roll_feedforward_angle_rad_);
      double body_roll_swap_amp_shift = body_roll_swap_amp;

      if(bc_move_amp >= M_PI)
        bc_move_amp -= 2.0*M_PI;
      else if(bc_move_amp <= -M_PI)
        bc_move_amp += 2.0*M_PI;

      detail_balance_time_idx = (int)( (body_move_periodTime - walking_time_ + reference_time_) / (double)TIME_UNIT );
      if(detail_balance_time_idx >= preview_size_)
        detail_balance_time_idx = preview_size_ - 1;

      double z_swap  = wsin(walking_time_ - reference_time_, body_move_periodTime, z_swap_phase_shift, z_swap_amp, z_swap_amp_shift);

      double wp_move = wsigmoid(walking_time_ - reference_time_, body_move_periodTime, 0, wp_move_amp, wp_move_amp_shift, 1.0, 1.0);
      double bz_move = wsigmoid(walking_time_ - reference_time_, body_move_periodTime, 0, bz_move_amp, bz_move_amp_shift, 1.0, 1.0);
      double ba_move = wsigmoid(walking_time_ - reference_time_, body_move_periodTime, 0, ba_move_amp, ba_move_amp_shift, 1.0, 1.0);
      double bb_move = wsigmoid(walking_time_ - reference_time_, body_move_periodTime, 0, bb_move_amp, bb_move_amp_shift, 1.0, 1.0);
      double bc_move = wsigmoid(walking_time_ - reference_time_, body_move_periodTime, 0, bc_move_amp, bc_move_amp_shift, 1.0, 1.0);

      present_waist_yaw_angle_rad_ = wp_move;
      present_body_pose_.z = bz_move + z_swap;
      present_body_pose_.roll = ba_move;
      present_body_pose_.pitch = bb_move;
      present_body_pose_.yaw = bc_move;

      //Feet
      double time = walking_time_ - reference_time_;
      double period_time, dsp_ratio, ssp_ratio, foot_move_period_time, ssp_time_start, ssp_time_end;
      double x_move_amp, y_move_amp, z_move_amp, a_move_amp, b_move_amp, c_move_amp, z_vibe_amp;
      double x_move_amp_shift, y_move_amp_shift, z_move_amp_shift, a_move_amp_shift, b_move_amp_shift, c_move_amp_shift, z_vibe_amp_shift;
      double z_vibe_phase_shift;
      double x_move, y_move, z_move, a_move, b_move, c_move, z_vibe;

      period_time = added_step_data_[0].time_data.abs_step_time - reference_time_;
      dsp_ratio = added_step_data_[0].time_data.dsp_ratio;
      ssp_ratio = 1 - dsp_ratio;
      foot_move_period_time = ssp_ratio*period_time;

      ssp_time_start = dsp_ratio*period_time/2.0;
      ssp_time_end = (1 + ssp_ratio)*period_time / 2.0;

      if(added_step_data_[0].position_data.moving_foot == RIGHT_FOOT_SWING)
      {
        x_move_amp = (added_step_data_[0].position_data.right_foot_pose.x - previous_step_right_foot_pose_.x);
        x_move_amp_shift = previous_step_right_foot_pose_.x;

        y_move_amp = (added_step_data_[0].position_data.right_foot_pose.y - previous_step_right_foot_pose_.y);
        y_move_amp_shift = previous_step_right_foot_pose_.y;

        z_move_amp = (added_step_data_[0].position_data.right_foot_pose.z - previous_step_right_foot_pose_.z);
        z_move_amp_shift = previous_step_right_foot_pose_.z;

        a_move_amp = (added_step_data_[0].position_data.right_foot_pose.roll - previous_step_right_foot_pose_.roll);
        a_move_amp_shift = previous_step_right_foot_pose_.roll;

        b_move_amp = (added_step_data_[0].position_data.right_foot_pose.pitch - previous_step_right_foot_pose_.pitch);
        b_move_amp_shift = previous_step_right_foot_pose_.pitch;

        c_move_amp = (added_step_data_[0].position_data.right_foot_pose.yaw - previous_step_right_foot_pose_.yaw);
        c_move_amp_shift = previous_step_right_foot_pose_.yaw;

        z_vibe_amp = added_step_data_[0].position_data.foot_z_swap*0.5;
        z_vibe_amp_shift = z_vibe_amp;
        z_vibe_phase_shift = M_PI*0.5;

        body_roll_swap_dir = -1.0;
      }
      else if(added_step_data_[0].position_data.moving_foot == LEFT_FOOT_SWING)    {
        x_move_amp = (added_step_data_[0].position_data.left_foot_pose.x - previous_step_left_foot_pose_.x);
        x_move_amp_shift = previous_step_left_foot_pose_.x;

        y_move_amp = (added_step_data_[0].position_data.left_foot_pose.y - previous_step_left_foot_pose_.y);
        y_move_amp_shift = previous_step_left_foot_pose_.y;

        z_move_amp = (added_step_data_[0].position_data.left_foot_pose.z - previous_step_left_foot_pose_.z);
        z_move_amp_shift = previous_step_left_foot_pose_.z;

        a_move_amp = (added_step_data_[0].position_data.left_foot_pose.roll - previous_step_left_foot_pose_.roll);
        a_move_amp_shift = previous_step_left_foot_pose_.roll;

        b_move_amp = (added_step_data_[0].position_data.left_foot_pose.pitch - previous_step_left_foot_pose_.pitch);
        b_move_amp_shift = previous_step_left_foot_pose_.pitch;

        c_move_amp = (added_step_data_[0].position_data.left_foot_pose.yaw - previous_step_left_foot_pose_.yaw);
        c_move_amp_shift = previous_step_left_foot_pose_.yaw;

        z_vibe_amp = added_step_data_[0].position_data.foot_z_swap*0.5;
        z_vibe_amp_shift = z_vibe_amp;
        z_vibe_phase_shift = M_PI*0.5;

        body_roll_swap_dir = 1.0;
      }
      else
      {
        x_move_amp = 0.0;
        x_move_amp_shift = previous_step_left_foot_pose_.x;

        y_move_amp = 0.0;
        y_move_amp_shift = previous_step_left_foot_pose_.y;

        z_move_amp = 0.0;
        z_move_amp_shift = previous_step_left_foot_pose_.z;

        a_move_amp = 0.0;
        a_move_amp_shift = previous_step_left_foot_pose_.roll;

        b_move_amp = 0.0;
        b_move_amp_shift = previous_step_left_foot_pose_.pitch;

        c_move_amp = 0.0;
        c_move_amp_shift = previous_step_left_foot_pose_.yaw;

        z_vibe_amp = 0.0;
        z_vibe_amp_shift = z_vibe_amp;
        z_vibe_phase_shift = M_PI*0.5;

        body_roll_swap_dir = 0.0;
      }

      if(c_move_amp >= M_PI)
        c_move_amp -= 2.0*M_PI;
      else if(c_move_amp <= -M_PI)
        c_move_amp += 2.0*M_PI;


      if( time <= ssp_time_start)
      {
        x_move = wsigmoid(ssp_time_start, foot_move_period_time, ssp_time_start, x_move_amp, x_move_amp_shift, added_step_data_[0].time_data.sigmoid_ratio_x,     added_step_data_[0].time_data.sigmoid_distortion_x);
        y_move = wsigmoid(ssp_time_start, foot_move_period_time, ssp_time_start, y_move_amp, y_move_amp_shift, added_step_data_[0].time_data.sigmoid_ratio_y,     added_step_data_[0].time_data.sigmoid_distortion_y);
        z_move = wsigmoid(ssp_time_start, foot_move_period_time, ssp_time_start, z_move_amp, z_move_amp_shift, added_step_data_[0].time_data.sigmoid_ratio_z,     added_step_data_[0].time_data.sigmoid_distortion_z);
        a_move = wsigmoid(ssp_time_start, foot_move_period_time, ssp_time_start, a_move_amp, a_move_amp_shift, added_step_data_[0].time_data.sigmoid_ratio_roll,  added_step_data_[0].time_data.sigmoid_distortion_roll);
        b_move = wsigmoid(ssp_time_start, foot_move_period_time, ssp_time_start, b_move_amp, b_move_amp_shift, added_step_data_[0].time_data.sigmoid_ratio_pitch, added_step_data_[0].time_data.sigmoid_distortion_pitch);
        c_move = wsigmoid(ssp_time_start, foot_move_period_time, ssp_time_start, c_move_amp, c_move_amp_shift, added_step_data_[0].time_data.sigmoid_ratio_yaw,   added_step_data_[0].time_data.sigmoid_distortion_yaw);

        z_vibe         = wsin(ssp_time_start, foot_move_period_time, z_vibe_phase_shift + 2.0*M_PI*ssp_time_start/foot_move_period_time, z_vibe_amp, z_vibe_amp_shift);
        body_roll_swap = wsin(ssp_time_start, foot_move_period_time, z_vibe_phase_shift + 2.0*M_PI*ssp_time_start/foot_move_period_time, body_roll_swap_amp, body_roll_swap_amp_shift);
        if(added_step_data_[0].position_data.moving_foot == RIGHT_FOOT_SWING)
        {
          balancing_index_ = 1;
        }
        else if(added_step_data_[0].position_data.moving_foot == LEFT_FOOT_SWING)    {
          balancing_index_ = 5;
        }
        else {
          balancing_index_ = 0;
        }
      }
      else if(time <= ssp_time_end) {
        x_move = wsigmoid(time, foot_move_period_time, ssp_time_start, x_move_amp, x_move_amp_shift, added_step_data_[0].time_data.sigmoid_ratio_x,     added_step_data_[0].time_data.sigmoid_distortion_x);
        y_move = wsigmoid(time, foot_move_period_time, ssp_time_start, y_move_amp, y_move_amp_shift, added_step_data_[0].time_data.sigmoid_ratio_y,     added_step_data_[0].time_data.sigmoid_distortion_y);
        z_move = wsigmoid(time, foot_move_period_time, ssp_time_start, z_move_amp, z_move_amp_shift, added_step_data_[0].time_data.sigmoid_ratio_z,     added_step_data_[0].time_data.sigmoid_distortion_z);
        a_move = wsigmoid(time, foot_move_period_time, ssp_time_start, a_move_amp, a_move_amp_shift, added_step_data_[0].time_data.sigmoid_ratio_roll,  added_step_data_[0].time_data.sigmoid_distortion_roll);
        b_move = wsigmoid(time, foot_move_period_time, ssp_time_start, b_move_amp, b_move_amp_shift, added_step_data_[0].time_data.sigmoid_ratio_pitch, added_step_data_[0].time_data.sigmoid_distortion_pitch);
        c_move = wsigmoid(time, foot_move_period_time, ssp_time_start, c_move_amp, c_move_amp_shift, added_step_data_[0].time_data.sigmoid_ratio_yaw,   added_step_data_[0].time_data.sigmoid_distortion_yaw);

        z_vibe         = wsin(time, foot_move_period_time, z_vibe_phase_shift + 2.0*M_PI*ssp_time_start/foot_move_period_time, z_vibe_amp,         z_vibe_amp_shift);
        body_roll_swap = wsin(time, foot_move_period_time, z_vibe_phase_shift + 2.0*M_PI*ssp_time_start/foot_move_period_time, body_roll_swap_amp, body_roll_swap_amp_shift);

        if(added_step_data_[0].position_data.moving_foot == RIGHT_FOOT_SWING)
        {
          if(time <= (ssp_time_end + ssp_time_start)*0.5)
            balancing_index_ = 2;
          else
            balancing_index_ = 3;
        }
        else if(added_step_data_[0].position_data.moving_foot == LEFT_FOOT_SWING)    {
          if(time <= (ssp_time_end + ssp_time_start)*0.5)
            balancing_index_ = 6;
          else
            balancing_index_ = 7;
        }
        else {
          balancing_index_ = 0;
        }
      }
      else {
        x_move = wsigmoid(ssp_time_end, foot_move_period_time, ssp_time_start, x_move_amp, x_move_amp_shift, added_step_data_[0].time_data.sigmoid_ratio_x,     added_step_data_[0].time_data.sigmoid_distortion_x);
        y_move = wsigmoid(ssp_time_end, foot_move_period_time, ssp_time_start, y_move_amp, y_move_amp_shift, added_step_data_[0].time_data.sigmoid_ratio_y,     added_step_data_[0].time_data.sigmoid_distortion_y);
        z_move = wsigmoid(ssp_time_end, foot_move_period_time, ssp_time_start, z_move_amp, z_move_amp_shift, added_step_data_[0].time_data.sigmoid_ratio_z,     added_step_data_[0].time_data.sigmoid_distortion_z);
        a_move = wsigmoid(ssp_time_end, foot_move_period_time, ssp_time_start, a_move_amp, a_move_amp_shift, added_step_data_[0].time_data.sigmoid_ratio_roll,  added_step_data_[0].time_data.sigmoid_distortion_roll);
        b_move = wsigmoid(ssp_time_end, foot_move_period_time, ssp_time_start, b_move_amp, b_move_amp_shift, added_step_data_[0].time_data.sigmoid_ratio_pitch, added_step_data_[0].time_data.sigmoid_distortion_pitch);
        c_move = wsigmoid(ssp_time_end, foot_move_period_time, ssp_time_start, c_move_amp, c_move_amp_shift, added_step_data_[0].time_data.sigmoid_ratio_yaw,   added_step_data_[0].time_data.sigmoid_distortion_yaw);

        z_vibe         = wsin(ssp_time_end, foot_move_period_time, z_vibe_phase_shift + 2.0*M_PI*ssp_time_start/foot_move_period_time, z_vibe_amp,         z_vibe_amp_shift);
        body_roll_swap = wsin(ssp_time_end, foot_move_period_time, z_vibe_phase_shift + 2.0*M_PI*ssp_time_start/foot_move_period_time, body_roll_swap_amp, body_roll_swap_amp_shift);
        if(added_step_data_[0].position_data.moving_foot == RIGHT_FOOT_SWING)
        {
          balancing_index_ = 4;
        }
        else if(added_step_data_[0].position_data.moving_foot == LEFT_FOOT_SWING)    {
          balancing_index_ = 8;
        }
        else {
          balancing_index_ = 0;
        }
      }

      body_roll_swap = body_roll_swap_dir * body_roll_swap;

      //            printf("m_Stepdata.size :  %d, time : %f, Foot : %d  x_move: %f %f\n",
          //                    m_StepData.size(), m_StepData[0].TimeData.dAbsStepTime, m_StepData[0].PositionData.bMovingFoot, x_move, time);
      if(added_step_data_[0].position_data.moving_foot == RIGHT_FOOT_SWING)
      {
        present_right_foot_pose_.x = x_move;
        present_right_foot_pose_.y = y_move;
        present_right_foot_pose_.z = z_move + z_vibe;
        present_right_foot_pose_.roll = a_move;
        present_right_foot_pose_.pitch = b_move;
        present_right_foot_pose_.yaw = c_move;

        present_left_foot_pose_ = added_step_data_[0].position_data.left_foot_pose;
      }
      else if(added_step_data_[0].position_data.moving_foot == LEFT_FOOT_SWING)    {
        present_right_foot_pose_ = added_step_data_[0].position_data.right_foot_pose;

        present_left_foot_pose_.x = x_move;
        present_left_foot_pose_.y = y_move;
        present_left_foot_pose_.z = z_move + z_vibe;
        present_left_foot_pose_.roll = a_move;
        present_left_foot_pose_.pitch = b_move;
        present_left_foot_pose_.yaw = c_move;
      }
      else {
        //printf("%d %f\n", m_WalkingTime, m_PresentLeftFootPosition.x );
        present_right_foot_pose_ = added_step_data_[0].position_data.right_foot_pose;
        present_left_foot_pose_ = added_step_data_[0].position_data.left_foot_pose;
      }

      shouler_swing_gain_ = added_step_data_[0].position_data.shoulder_swing_gain;
      elbow_swing_gain_ = added_step_data_[0].position_data.elbow_swing_gain;

      walking_time_ += TIME_UNIT;

      //fprintf(stderr, "%d, %f\n", m_WalkingTime, m_StepData[m_StepData.size() - 1].TimeData.dAbsStepTime);
      if(walking_time_ > added_step_data_[added_step_data_.size() - 1].time_data.abs_step_time - 0.5) {
        //                m_WalkingTime = m_StepData[m_StepData.size() - 1].TimeData.dAbsStepTime;
        real_running = false;
        calcStepIdxData();
        mutex_lock_.unlock();
        reInitialize();
        //                printf("Localize\n");
      }

      if(balancing_index_ == 0 || balancing_index_ == 9)
      {
        left_fz_sigmoid_start_time_ = walking_time_;
        left_fz_sigmoid_end_time_   = walking_time_;
        left_fz_sigmoid_target_  = left_dsp_fz_N_;
        left_fz_sigmoid_shift_   = left_dsp_fz_N_;
      }
      else if(balancing_index_ == 1 )
      {
        left_fz_sigmoid_end_time_ = ssp_time_start + reference_time_;
        left_fz_sigmoid_target_ = left_ssp_fz_N_;
      }
      else if(balancing_index_ == 4 )
      {
        left_fz_sigmoid_start_time_ = ssp_time_end + reference_time_;
        left_fz_sigmoid_shift_ = left_ssp_fz_N_;
        if(added_step_data_.size() > 1)
        {
          if(added_step_data_[1].position_data.moving_foot == STANDING)
          {
            left_fz_sigmoid_target_ = left_dsp_fz_N_;
            left_fz_sigmoid_end_time_ = added_step_data_[0].time_data.abs_step_time;
          }
          else if(added_step_data_[1].position_data.moving_foot == LEFT_FOOT_SWING)
          {
            left_fz_sigmoid_target_ = 0.0;
            left_fz_sigmoid_end_time_ = (added_step_data_[1].time_data.abs_step_time - added_step_data_[0].time_data.abs_step_time)*0.5*added_step_data_[1].time_data.dsp_ratio + added_step_data_[0].time_data.abs_step_time;
          }
          else
          {
            left_fz_sigmoid_target_ = left_ssp_fz_N_;
            left_fz_sigmoid_end_time_ = (added_step_data_[1].time_data.abs_step_time - added_step_data_[0].time_data.abs_step_time)*0.5*added_step_data_[1].time_data.dsp_ratio + added_step_data_[0].time_data.abs_step_time;
          }
        }
        else {
          left_fz_sigmoid_target_ = left_dsp_fz_N_;
          left_fz_sigmoid_end_time_ = added_step_data_[0].time_data.abs_step_time;
        }

      }
      else if(balancing_index_ == 5 )
      {
        left_fz_sigmoid_end_time_ = ssp_time_start + reference_time_;
        left_fz_sigmoid_target_ = 0.0;
      }
      else if(balancing_index_ == 8)
      {
        left_fz_sigmoid_start_time_ = ssp_time_end + reference_time_;
        left_fz_sigmoid_shift_ = 0.0;
        if(added_step_data_.size() > 1)
        {
          if(added_step_data_[1].position_data.moving_foot == STANDING)
          {
            left_fz_sigmoid_target_ = left_dsp_fz_N_;
            left_fz_sigmoid_end_time_ = added_step_data_[0].time_data.abs_step_time;
          }
          else if(added_step_data_[1].position_data.moving_foot == LEFT_FOOT_SWING)
          {
            left_fz_sigmoid_target_ = 0.0;
            left_fz_sigmoid_end_time_ = (added_step_data_[1].time_data.abs_step_time - added_step_data_[0].time_data.abs_step_time)*0.5*added_step_data_[1].time_data.dsp_ratio + added_step_data_[0].time_data.abs_step_time;
          }
          else
          {
            left_fz_sigmoid_target_ = left_ssp_fz_N_;
            left_fz_sigmoid_end_time_ = (added_step_data_[1].time_data.abs_step_time - added_step_data_[0].time_data.abs_step_time)*0.5*added_step_data_[1].time_data.dsp_ratio + added_step_data_[0].time_data.abs_step_time;
          }
        }
        else
        {
          left_fz_sigmoid_target_ = left_dsp_fz_N_;
          left_fz_sigmoid_end_time_ = added_step_data_[0].time_data.abs_step_time;
        }
      }
      else
      {
        left_fz_sigmoid_start_time_ = walking_time_;
        left_fz_sigmoid_end_time_   = walking_time_;
      }
    }

    mutex_lock_.unlock();

    mat_g_to_cob_ = robotis_framework::getTransformationXYZRPY(present_body_pose_.x, present_body_pose_.y, present_body_pose_.z,
        present_body_pose_.roll, present_body_pose_.pitch, present_body_pose_.yaw);

    mat_g_to_rfoot_ = robotis_framework::getTransformationXYZRPY(present_right_foot_pose_.x, present_right_foot_pose_.y, present_right_foot_pose_.z,
        present_right_foot_pose_.roll, present_right_foot_pose_.pitch, present_right_foot_pose_.yaw);

    mat_g_to_lfoot_ = robotis_framework::getTransformationXYZRPY(present_left_foot_pose_.x, present_left_foot_pose_.y, present_left_foot_pose_.z,
        present_left_foot_pose_.roll, present_left_foot_pose_.pitch, present_left_foot_pose_.yaw);

    mat_cob_to_g_ = robotis_framework::getInverseTransformation(mat_g_to_cob_);

    mat_robot_to_cob_ = robotis_framework::getRotation4d(present_body_pose_.roll, present_body_pose_.pitch, 0);
    mat_cob_to_robot_ = robotis_framework::getInverseTransformation(mat_robot_to_cob_);
    mat_g_to_robot_   = mat_g_to_cob_ * mat_cob_to_robot_;
    mat_robot_to_g_   = robotis_framework::getInverseTransformation(mat_g_to_robot_);

    mat_robot_to_rfoot_ = mat_robot_to_g_*mat_g_to_rfoot_;
    mat_robot_to_lfoot_ = mat_robot_to_g_*mat_g_to_lfoot_;



    //Stabilizer Start
    //Balancing Algorithm
    double target_fz_N  = 0;
    double right_roll_dir = thormang3_kd_->thormang3_link_data_[ID_R_LEG_START+2*1]->joint_axis_.coeff(0,0);
    double left_roll_dir  = thormang3_kd_->thormang3_link_data_[ID_L_LEG_START+2*1]->joint_axis_.coeff(0,0);


    double right_leg_fx_N  = current_right_fx_N_;
    double right_leg_fy_N  = current_right_fy_N_;
    double right_leg_fz_N  = current_right_fz_N_;
    double right_leg_Tx_Nm = current_right_tx_Nm_;
    double right_leg_Ty_Nm = current_right_ty_Nm_;
    double right_leg_Tz_Nm = current_right_tz_Nm_;

    double left_leg_fx_N  = current_left_fx_N_;
    double left_leg_fy_N  = current_left_fy_N_;
    double left_leg_fz_N  = current_left_fz_N_;
    double left_leg_Tx_Nm = current_left_tx_Nm_;
    double left_leg_Ty_Nm = current_left_ty_Nm_;
    double left_leg_Tz_Nm = current_left_tz_Nm_;

    Eigen::MatrixXd  mat_right_force, mat_right_torque;
    mat_right_force.resize(4,1);    mat_right_force.fill(0);
    mat_right_torque.resize(4,1);   mat_right_torque.fill(0);
    mat_right_force(0,0) = right_leg_fx_N;
    mat_right_force(1,0) = right_leg_fy_N;
    mat_right_force(2,0) = right_leg_fz_N;
    mat_right_torque(0,0) = right_leg_Tx_Nm;
    mat_right_torque(1,0) = right_leg_Ty_Nm;
    mat_right_torque(2,0) = right_leg_Tz_Nm;

    Eigen::MatrixXd  mat_left_force, mat_left_torque;
    mat_left_force.resize(4,1);     mat_left_force.fill(0);
    mat_left_torque.resize(4,1);    mat_left_torque.fill(0);
    mat_left_force(0,0) = left_leg_fx_N;
    mat_left_force(1,0) = left_leg_fy_N;
    mat_left_force(2,0) = left_leg_fz_N;
    mat_left_torque(0,0) = left_leg_Tx_Nm;
    mat_left_torque(1,0) = left_leg_Ty_Nm;
    mat_left_torque(2,0) = left_leg_Tz_Nm;

    mat_right_force  = mat_robot_to_rfoot_*mat_rfoot_to_rft_*mat_right_force;
    mat_right_torque = mat_robot_to_lfoot_*mat_lfoot_to_lft_*mat_right_torque;

    mat_left_force  = mat_robot_to_lfoot_*mat_rfoot_to_rft_*mat_left_force;
    mat_left_torque = mat_robot_to_lfoot_*mat_lfoot_to_lft_*mat_left_torque;

    double gyro_roll_rad_per_sec  = current_gyro_roll_rad_per_sec_;
    double gyro_pitch_rad_per_sec =  current_gyro_pitch_rad_per_sec_;

    Eigen::MatrixXd imu_mat = (rot_x_pi_3d_ * quat_current_imu_.toRotationMatrix()) * rot_z_pi_3d_;

    current_imu_roll_rad_  = atan2( imu_mat.coeff(2,1), imu_mat.coeff(2,2));
    current_imu_pitch_rad_ = atan2(-imu_mat.coeff(2,0), sqrt(robotis_framework::powDI(imu_mat.coeff(2,1), 2) + robotis_framework::powDI(imu_mat.coeff(2,2), 2)));

    double iu_roll_rad  = current_imu_roll_rad_;
    double iu_pitch_rad = current_imu_pitch_rad_;

    //switch(Balancing_Index){
    switch(balancing_index_)
    {
    case 0:
      //fprintf(stderr, "DSP : START\n");
      target_fz_N = left_dsp_fz_N_ - right_dsp_fz_N_;
      right_roll_dir = left_roll_dir = 1.0;
      break;
    case 1:
      //fprintf(stderr, "DSP : R--O->L\n");
      target_fz_N = left_dsp_fz_N_ - right_dsp_fz_N_;
      right_roll_dir = left_roll_dir = 1.0;
      break;
    case 2:
      //fprintf(stderr, "SSP : L_BALANCING1\n");
      target_fz_N = left_ssp_fz_N_;
      right_roll_dir = -1.0;
      left_roll_dir = 1.0;
      break;
    case 3:
      //fprintf(stderr, "SSP : L_BALANCING2\n");
      target_fz_N = left_ssp_fz_N_;
      right_roll_dir = -1.0;
      left_roll_dir = 1.0;
      break;
    case 4:
      //fprintf(stderr, "DSP : R--O<-L\n");
      target_fz_N = left_dsp_fz_N_ - right_dsp_fz_N_;
      right_roll_dir = 1.0;
      left_roll_dir = 1.0;
      break;
    case 5:
      //fprintf(stderr, "DSP : R<-O--L\n");
      target_fz_N = left_dsp_fz_N_ - right_dsp_fz_N_;
      right_roll_dir = 1.0;
      left_roll_dir = 1.0;
      break;
    case 6:
      //fprintf(stderr, "SSP : R_BALANCING1\n");
      target_fz_N = -right_ssp_fz_N_;
      right_roll_dir = 1.0;
      left_roll_dir = -1.0;
      break;
    case 7:
      //fprintf(stderr, "SSP : R_BALANCING2\n");
      target_fz_N =  -right_ssp_fz_N_;
      right_roll_dir = 1.0;
      left_roll_dir = -1.0;
      break;
    case 8:
      //fprintf(stderr, "DSP : R->O--L");
      target_fz_N = left_dsp_fz_N_ - right_dsp_fz_N_;
      right_roll_dir = 1.0;
      left_roll_dir =  1.0;
      break;
    case 9:
      //fprintf(stderr, "DSP : END");
      target_fz_N = left_dsp_fz_N_ - right_dsp_fz_N_;
      right_roll_dir = 1.0;
      left_roll_dir =  1.0;
      break;
    default:
      break;
    }

    bool IsDSP = false;

    double r_target_fz_N = right_dsp_fz_N_;
    double l_target_fz_N = left_dsp_fz_N_;

    if( (balancing_index_ == 0) ||
        (balancing_index_ == 1) ||
        (balancing_index_ == 4) ||
        (balancing_index_ == 5) ||
        (balancing_index_ == 8) ||
        (balancing_index_ == 9) )
    {
      IsDSP = true;
    }
    else
      IsDSP = false;

    if(IsDSP)
    {
      if( (balancing_index_ == 0) || (balancing_index_ == 9) )
      {
        r_target_fz_N = right_dsp_fz_N_;
        l_target_fz_N = left_dsp_fz_N_;
      }
      else
      {
        double l_target_fz_N_amp = 0;
        double l_target_fz_N_amp_shift = 0;

        l_target_fz_N = wsigmoid(walking_time_ - TIME_UNIT, left_fz_sigmoid_end_time_ -  left_fz_sigmoid_start_time_, left_fz_sigmoid_start_time_, left_fz_sigmoid_target_ - left_fz_sigmoid_shift_, left_fz_sigmoid_shift_, 1.0, 1.0);
        r_target_fz_N = left_ssp_fz_N_ - l_target_fz_N;
      }

    }
    else
    {
      if( (balancing_index_ == 2) || (balancing_index_ == 3) )
      {
        r_target_fz_N = 0;
        l_target_fz_N = left_ssp_fz_N_;
      }
      else
      {
        r_target_fz_N = right_ssp_fz_N_;
        l_target_fz_N = 0;
      }
    }

    balance_ctrl_.setDesiredCOBGyro(0,0);
    balance_ctrl_.setDesiredCOBOrientation(present_body_pose_.roll, present_body_pose_.pitch);
    balance_ctrl_.setDesiredFootForceTorque(0, 0, r_target_fz_N, 0, 0, 0,
                                            0, 0, l_target_fz_N, 0, 0, 0);
    balance_ctrl_.setDesiredPose(mat_g_to_cob_, mat_g_to_rfoot_, mat_g_to_lfoot_);
    balance_ctrl_.setCurrentGyroSensorOutput(current_gyro_roll_rad_per_sec_, current_gyro_pitch_rad_per_sec_);
    balance_ctrl_.setCurrentOrientationSensorOutput(current_imu_roll_rad_, current_imu_pitch_rad_);
    balance_ctrl_.setCurrentFootForceTorqueSensorOutput(mat_right_force.coeff(0,0),  mat_right_force.coeff(1,0),  mat_right_force.coeff(2,0),
                                                        mat_right_torque.coeff(0,0), mat_right_torque.coeff(1,0), mat_right_torque.coeff(2,0),
                                                        mat_left_force.coeff(0,0),   mat_left_force.coeff(1,0),   mat_left_force.coeff(2,0),
                                                        mat_left_torque.coeff(0,0),  mat_left_torque.coeff(1,0),  mat_left_torque.coeff(2,0));

    balance_ctrl_.process(&balance_error_, &mat_cob_adjustment_, &mat_rfoot_adjustment_, &mat_lfoot_adjustment_);

    mat_robot_to_cob_modified_ = mat_robot_to_cob_*mat_cob_adjustment_;
    mat_cob_to_robot_modified_ = robotis_framework::getInverseTransformation(mat_robot_to_cob_modified_);
    mat_robot_to_rf_modified_  = mat_robot_to_g_*mat_g_to_rfoot_*mat_rfoot_adjustment_;
    mat_robot_to_lf_modified_  = mat_robot_to_g_*mat_g_to_lfoot_*mat_lfoot_adjustment_;
    //Stabilizer End

    rhip_to_rfoot_pose_ = getPose3DfromTransformMatrix((mat_rhip_to_cob_ * mat_cob_to_robot_modified_) * mat_robot_to_rf_modified_);
    lhip_to_lfoot_pose_ = getPose3DfromTransformMatrix((mat_lhip_to_cob_ * mat_cob_to_robot_modified_) * mat_robot_to_lf_modified_);

    if((rhip_to_rfoot_pose_.yaw > 30.0*M_PI/180.0) || (rhip_to_rfoot_pose_.yaw < -30.0*M_PI/180.0) ) {
      //printf("yawyaw\n");
      return;
    }

    if((lhip_to_lfoot_pose_.yaw < -30.0*M_PI/180.0) || (lhip_to_lfoot_pose_.yaw > 30.0*M_PI/180.0) ) {
      //printf("yawyaw2\n");
      return;
    }

    if(thormang3_kd_->calcInverseKinematicsForRightLeg(&r_leg_out_angle_rad_[0], rhip_to_rfoot_pose_.x, rhip_to_rfoot_pose_.y, rhip_to_rfoot_pose_.z, rhip_to_rfoot_pose_.roll, rhip_to_rfoot_pose_.pitch, rhip_to_rfoot_pose_.yaw) == false)
    {
      printf("IK not Solved EPR : %f %f %f %f %f %f\n", rhip_to_rfoot_pose_.x, rhip_to_rfoot_pose_.y, rhip_to_rfoot_pose_.z, rhip_to_rfoot_pose_.roll, rhip_to_rfoot_pose_.pitch, rhip_to_rfoot_pose_.yaw);
      return;
    }

    if(thormang3_kd_->calcInverseKinematicsForLeftLeg(&l_leg_out_angle_rad_[0], lhip_to_lfoot_pose_.x, lhip_to_lfoot_pose_.y, lhip_to_lfoot_pose_.z, lhip_to_lfoot_pose_.roll, lhip_to_lfoot_pose_.pitch, lhip_to_lfoot_pose_.yaw) == false)
    {
      printf("IK not Solved EPL : %f %f %f %f %f %f\n", lhip_to_lfoot_pose_.x, lhip_to_lfoot_pose_.y, lhip_to_lfoot_pose_.z, lhip_to_lfoot_pose_.roll, lhip_to_lfoot_pose_.pitch, lhip_to_lfoot_pose_.yaw);
      return;
    }


    r_shoulder_out_angle_rad_ = r_shoulder_dir_*(mat_robot_to_rfoot_.coeff(0, 3) - mat_robot_to_lfoot_.coeff(0, 3))*shouler_swing_gain_ + r_init_shoulder_angle_rad_;
    l_shoulder_out_angle_rad_ = l_shoulder_dir_*(mat_robot_to_lfoot_.coeff(0, 3) - mat_robot_to_rfoot_.coeff(0, 3))*shouler_swing_gain_ + l_init_shoulder_angle_rad_;
    r_elbow_out_angle_rad_ = r_elbow_dir_*(mat_robot_to_rfoot_.coeff(0, 3) - mat_robot_to_lfoot_.coeff(0, 3))*elbow_swing_gain_ + r_init_elbow_angle_rad_;
    l_elbow_out_angle_rad_ = l_elbow_dir_*(mat_robot_to_lfoot_.coeff(0, 3) - mat_robot_to_rfoot_.coeff(0, 3))*elbow_swing_gain_ + l_init_elbow_angle_rad_;


    if(added_step_data_.size() != 0)
    {
      if(added_step_data_[0].position_data.moving_foot == LEFT_FOOT_SWING)
        r_leg_out_angle_rad_[1] = r_leg_out_angle_rad_[1] + body_roll_swap;
      else if(added_step_data_[0].position_data.moving_foot == RIGHT_FOOT_SWING)
        r_leg_out_angle_rad_[1] = r_leg_out_angle_rad_[1] - 0.35*body_roll_swap;
    }

    if(added_step_data_.size() != 0)
    {
      if(added_step_data_[0].position_data.moving_foot == RIGHT_FOOT_SWING)
        l_leg_out_angle_rad_[1] = l_leg_out_angle_rad_[1] + body_roll_swap;
      else if(added_step_data_[0].position_data.moving_foot == LEFT_FOOT_SWING)
        l_leg_out_angle_rad_[1] = l_leg_out_angle_rad_[1] - 0.35*body_roll_swap;
    }

    for(int angle_idx = 0; angle_idx < 6; angle_idx++)
    {
      out_angle_rad_[angle_idx+0] = r_leg_out_angle_rad_[angle_idx];
      out_angle_rad_[angle_idx+6] = l_leg_out_angle_rad_[angle_idx];
    }

  }
}


double RobotisOnlineWalking::wsin(double time, double period, double period_shift, double mag, double mag_shift)
{
  return mag * sin(2 * M_PI / period * time - period_shift) + mag_shift;
}

double RobotisOnlineWalking::wsigmoid(double time, double period, double time_shift, double mag, double mag_shift, double sigmoid_ratio, double distortion_ratio)
{
  double value = mag_shift, Amplitude = 0.0, sigmoid_distor_gain = 1.0, t = 0.0;
  if ((sigmoid_ratio >= 1.0) && (sigmoid_ratio < 2.0))
  {
    if( time >= time_shift+period*(2-sigmoid_ratio)) {
      value = mag_shift + mag;
    }
    else
    {
      t = 2.0*M_PI*(time - time_shift)/(period*(2-sigmoid_ratio));
      sigmoid_distor_gain = distortion_ratio + (1-distortion_ratio)*(time-(time_shift+period*(1-sigmoid_ratio)))/(period*(2-sigmoid_ratio));
      Amplitude = mag/(2.0*M_PI);
      value = mag_shift + Amplitude*(t - sigmoid_distor_gain*sin(t));
    }
  }
  else if( (sigmoid_ratio >= 0.0) && (sigmoid_ratio < 1.0))
  {
    if( time <= time_shift+period*(1-sigmoid_ratio))
      value = mag_shift;
    else {
      t = 2.0*M_PI*(time - time_shift-period*(1-sigmoid_ratio))/(period*sigmoid_ratio);
      sigmoid_distor_gain = distortion_ratio + (1-distortion_ratio)*(time-(time_shift+period*(1-sigmoid_ratio)))/(period*sigmoid_ratio);
      Amplitude = mag/(2.0*M_PI);
      value = mag_shift + Amplitude*(t - sigmoid_distor_gain*sin(t));
    }
  }
  else if(( sigmoid_ratio >= 2.0) && ( sigmoid_ratio < 3.0))
  {
    double nsigmoid_ratio = sigmoid_ratio - 2.0;
    if(time <= time_shift + period*(1.0-nsigmoid_ratio)*0.5)
      value = mag_shift;
    else if(time >= time_shift + period*(1.0+nsigmoid_ratio)*0.5)
      value = mag + mag_shift;
    else {
      t = 2.0*M_PI*(time - (time_shift+period*(1.0-nsigmoid_ratio)*0.5))/(period*nsigmoid_ratio);
      sigmoid_distor_gain = distortion_ratio + (1.0-distortion_ratio)*(time-(time_shift+period*(1.0-nsigmoid_ratio)*0.5))/(period*nsigmoid_ratio);
      Amplitude = mag/(2.0*M_PI);
      value = mag_shift + Amplitude*(t - sigmoid_distor_gain*sin(t));
    }
  }
  else
    value = mag_shift;

  return value;
}
