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

#include "thormang3_wholebody_module/wholebody_module.h"

using namespace thormang3;

WholebodyModule::WholebodyModule()
  : control_cycle_sec_(0.008),
    is_moving_(false),
    is_balancing_(false),
    is_offset_updating_(false),
    goal_initialize_(false),
    balance_control_initialize_(false),
    body_offset_initialize_(false),
    joint_control_initialize_(false),
    wholebody_initialize_(false),
    walking_initialize_(false),
    is_foot_step_2d_(false),
    walking_phase_(DSP),
    body_offset_x_(0.0),
    body_offset_y_(0.0),
    total_mass_(42.533)
{
  enable_       = false;
  module_name_  = "wholebody_module";
  control_mode_ = robotis_framework::PositionControl;
  control_type_ = NONE;
  balance_type_ = OFF;

  thormang3_kdl_ = new Thormang3Kinematics();

  /* arm */
  result_["r_arm_sh_p1"]   = new robotis_framework::DynamixelState();
  result_["l_arm_sh_p1"]   = new robotis_framework::DynamixelState();
  result_["r_arm_sh_r"]    = new robotis_framework::DynamixelState();
  result_["l_arm_sh_r"]    = new robotis_framework::DynamixelState();
  result_["r_arm_sh_p2"]   = new robotis_framework::DynamixelState();
  result_["l_arm_sh_p2"]   = new robotis_framework::DynamixelState();
  result_["r_arm_el_y"]    = new robotis_framework::DynamixelState();
  result_["l_arm_el_y"]    = new robotis_framework::DynamixelState();
  result_["r_arm_wr_r"]    = new robotis_framework::DynamixelState();
  result_["l_arm_wr_r"]    = new robotis_framework::DynamixelState();
  result_["r_arm_wr_y"]    = new robotis_framework::DynamixelState();
  result_["l_arm_wr_y"]    = new robotis_framework::DynamixelState();
  result_["r_arm_wr_p"]    = new robotis_framework::DynamixelState();
  result_["l_arm_wr_p"]    = new robotis_framework::DynamixelState();

  /* leg */
  result_["r_leg_hip_y"]   = new robotis_framework::DynamixelState();
  result_["r_leg_hip_r"]   = new robotis_framework::DynamixelState();
  result_["r_leg_hip_p"]   = new robotis_framework::DynamixelState();
  result_["r_leg_kn_p"]    = new robotis_framework::DynamixelState();
  result_["r_leg_an_p"]    = new robotis_framework::DynamixelState();
  result_["r_leg_an_r"]    = new robotis_framework::DynamixelState();
  result_["l_leg_hip_y"]   = new robotis_framework::DynamixelState();
  result_["l_leg_hip_r"]   = new robotis_framework::DynamixelState();
  result_["l_leg_hip_p"]   = new robotis_framework::DynamixelState();
  result_["l_leg_kn_p"]    = new robotis_framework::DynamixelState();
  result_["l_leg_an_p"]    = new robotis_framework::DynamixelState();
  result_["l_leg_an_r"]    = new robotis_framework::DynamixelState();

  /* body */
  result_["torso_y"]       = new robotis_framework::DynamixelState();

  /* arm */
  joint_name_to_id_["r_arm_sh_p1"]  = 1;
  joint_name_to_id_["l_arm_sh_p1"]  = 2;
  joint_name_to_id_["r_arm_sh_r"]   = 3;
  joint_name_to_id_["l_arm_sh_r"]   = 4;
  joint_name_to_id_["r_arm_sh_p2"]  = 5;
  joint_name_to_id_["l_arm_sh_p2"]  = 6;
  joint_name_to_id_["r_arm_el_y"]   = 7;
  joint_name_to_id_["l_arm_el_y"]   = 8;
  joint_name_to_id_["r_arm_wr_r"]   = 9;
  joint_name_to_id_["l_arm_wr_r"]   = 10;
  joint_name_to_id_["r_arm_wr_y"]   = 11;
  joint_name_to_id_["l_arm_wr_y"]   = 12;
  joint_name_to_id_["r_arm_wr_p"]   = 13;
  joint_name_to_id_["l_arm_wr_p"]   = 14;

  /* leg */
  joint_name_to_id_["r_leg_hip_y"]  = 15;
  joint_name_to_id_["l_leg_hip_y"]  = 16;
  joint_name_to_id_["r_leg_hip_r"]  = 17;
  joint_name_to_id_["l_leg_hip_r"]  = 18;
  joint_name_to_id_["r_leg_hip_p"]  = 19;
  joint_name_to_id_["l_leg_hip_p"]  = 20;
  joint_name_to_id_["r_leg_kn_p"]   = 21;
  joint_name_to_id_["l_leg_kn_p"]   = 22;
  joint_name_to_id_["r_leg_an_p"]   = 23;
  joint_name_to_id_["l_leg_an_p"]   = 24;
  joint_name_to_id_["r_leg_an_r"]   = 25;
  joint_name_to_id_["l_leg_an_r"]   = 26;

  /* body */
  joint_name_to_id_["torso_y"]      = 27;

  /* end effector */
  joint_name_to_id_["r_arm_end"]    = 35;
  joint_name_to_id_["l_arm_end"]    = 34;
  joint_name_to_id_["r_leg_end"]    = 45;
  joint_name_to_id_["l_leg_end"]    = 46;

  /* parameter */
  number_of_joints_ = NUM_OF_JOINTS;

  curr_joint_accel_.resize(number_of_joints_, 0.0);
  curr_joint_vel_.resize(number_of_joints_, 0.0);
  curr_joint_pos_.resize(number_of_joints_, 0.0);

  des_joint_accel_.resize(number_of_joints_, 0.0);
  des_joint_vel_.resize(number_of_joints_, 0.0);
  des_joint_pos_.resize(number_of_joints_, 0.0);

  goal_joint_accel_.resize(number_of_joints_, 0.0);
  goal_joint_vel_.resize(number_of_joints_, 0.0);
  goal_joint_pos_.resize(number_of_joints_, 0.0);

  des_joint_feedback_.resize(number_of_joints_, 0.0);
  des_joint_feedforward_.resize(number_of_joints_, 0.0);
  des_joint_pos_to_robot_.resize(number_of_joints_, 0.0);

  joint_feedforward_gain_.resize(number_of_joints_, 0.0);

  // body position default
  des_body_pos_.resize(3, 0.0);
  des_body_vel_.resize(3, 0.0);
  des_body_accel_.resize(3, 0.0);
  des_body_Q_.resize(4, 0.0);

  // left foot position default
  des_l_leg_pos_.resize(3, 0.0);
  des_l_leg_vel_.resize(3, 0.0);
  des_l_leg_accel_.resize(3, 0.0);
  des_l_leg_Q_.resize(4, 0.0);

  // right foot position default
  des_r_leg_pos_.resize(3, 0.0);
  des_r_leg_vel_.resize(3, 0.0);
  des_r_leg_accel_.resize(3, 0.0);
  des_r_leg_Q_.resize(4, 0.0);

  x_lipm_.resize(3, 0.0);
  y_lipm_.resize(3, 0.0);

  resetBodyPose();

  // walking parameter default
  walking_param_.dsp_ratio        = 0.2;
  walking_param_.lipm_height      = 0.7;
  walking_param_.foot_height_max  = 0.07;
  walking_param_.zmp_offset_x     = 0.0;
  walking_param_.zmp_offset_y     = -0.015;

  des_balance_gain_ratio_.resize(1, 0.0);
  goal_balance_gain_ratio_.resize(1, 0.0);

  balance_control_.initialize(control_cycle_sec_*1000.0);
  balance_control_.setGyroBalanceEnable(false); // Gyro
  balance_control_.setOrientationBalanceEnable(false); // IMU
  balance_control_.setForceTorqueBalanceEnable(false); // FT

  balance_l_foot_force_x_   = 0.0;
  balance_l_foot_force_y_   = 0.0;
  balance_l_foot_force_z_   = 0.0;
  balance_l_foot_torque_x_  = 0.0;
  balance_l_foot_torque_y_  = 0.0;
  balance_l_foot_torque_z_  = 0.0;

  balance_r_foot_force_x_   = 0.0;
  balance_r_foot_force_y_   = 0.0;
  balance_r_foot_force_z_   = 0.0;
  balance_r_foot_torque_x_  = 0.0;
  balance_r_foot_torque_y_  = 0.0;
  balance_r_foot_torque_z_  = 0.0;

  // Body Offset
  des_body_offset_.resize(3, 0.0);
  goal_body_offset_.resize(3, 0.0);

  std::string balance_gain_path = ros::package::getPath("thormang3_wholebody_module") + "/config/balance_gain.yaml";
  parseBalanceGainData(balance_gain_path);

  std::string joint_feedback_gain_path = ros::package::getPath("thormang3_wholebody_module") + "/config/joint_feedback_gain.yaml";
  parseJointFeedbackGainData(joint_feedback_gain_path);

  std::string joint_feedforward_gain_path = ros::package::getPath("thormang3_wholebody_module") + "/config/joint_feedforward_gain.yaml";
  parseJointFeedforwardGainData(joint_feedforward_gain_path);
}

WholebodyModule::~WholebodyModule()
{
  queue_thread_.join();
}

void WholebodyModule::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
{
  control_cycle_sec_ = control_cycle_msec * 0.001;
  queue_thread_      = boost::thread(boost::bind(&WholebodyModule::queueThread, this));

  ros::NodeHandle ros_node;

  // Publisher
  status_msg_pub_       = ros_node.advertise<robotis_controller_msgs::StatusMsg>("/robotis/status", 1);
  movement_done_pub_    = ros_node.advertise<std_msgs::String>("/robotis/movement_done", 1);
  goal_joint_state_pub_ = ros_node.advertise<sensor_msgs::JointState>("/robotis/wholebody/goal_joint_states", 1);

  // Service
  get_preview_matrix_client_ = ros_node.serviceClient<thormang3_wholebody_module_msgs::GetPreviewMatrix>("/robotis/get_preview_matrix", 0);
}

void WholebodyModule::queueThread()
{
  ros::NodeHandle     ros_node;
  ros::CallbackQueue  callback_queue;

  ros_node.setCallbackQueue(&callback_queue);

  // Subscriber
  ros::Subscriber reset_body_sub_ = ros_node.subscribe("/robotis/reset_body", 5,
                                                       &WholebodyModule::setResetBodyCallback, this);
  ros::Subscriber joint_pose_sub_ = ros_node.subscribe("/robotis/goal_joint_pose", 5,
                                                       &WholebodyModule::goalJointPoseCallback, this);
  ros::Subscriber kinematics_pose_sub_ = ros_node.subscribe("/robotis/goal_kinematics_pose", 5,
                                                            &WholebodyModule::goalKinematicsPoseCallback, this);
  ros::Subscriber foot_step_command_sub_ = ros_node.subscribe("/robotis/foot_step_command", 5,
                                                              &WholebodyModule::footStepCommandCallback, this);
  ros::Subscriber walking_param_sub_ = ros_node.subscribe("/robotis/walking_param", 5,
                                                          &WholebodyModule::walkingParamCallback, this);
  ros::Subscriber wholebody_balance_msg_sub = ros_node.subscribe("/robotis/wholebody_balance_msg", 5,
                                                                 &WholebodyModule::setWholebodyBalanceMsgCallback, this);
  ros::Subscriber body_offset_msg_sub = ros_node.subscribe("/robotis/wholebody/body_offset", 5,
                                                           &WholebodyModule::setBodyOffsetCallback, this);

  ros::Subscriber footsteps_sub = ros_node.subscribe("/robotis/wholebody/footsteps_2d", 5,
                                                     &WholebodyModule::footStep2DCallback, this);

  ros::Subscriber imu_data_sub = ros_node.subscribe("/robotis/sensor/imu/imu", 5,
                                                    &WholebodyModule::imuDataCallback, this);
  ros::Subscriber l_foot_ft_sub = ros_node.subscribe("/robotis/sensor/l_foot_ft", 3,
                                                     &WholebodyModule::leftFootForceTorqueOutputCallback, this);
  ros::Subscriber r_foot_ft_sub = ros_node.subscribe("/robotis/sensor/r_foot_ft", 3,
                                                     &WholebodyModule::rightFootForceTorqueOutputCallback, this);

  // Service
  ros::ServiceServer get_joint_pose_server = ros_node.advertiseService("/robotis/get_joint_pose",
                                                                       &WholebodyModule::getJointPoseCallback, this);
  ros::ServiceServer get_kinematics_pose_server = ros_node.advertiseService("/robotis/get_kinematics_pose",
                                                                            &WholebodyModule::getKinematicsPoseCallback, this);

  ros::WallDuration duration(control_cycle_sec_);
  while(ros_node.ok())
    callback_queue.callAvailable(duration);
}

void WholebodyModule::resetBodyPose()
{
  des_body_pos_[0] = 0.0; //body_offset_x_;
  des_body_pos_[1] = 0.0; //body_offset_y_;
  des_body_pos_[2] = 0.7340152;

  des_body_Q_[0] = 0.0;
  des_body_Q_[1] = 0.0;
  des_body_Q_[2] = 0.0;
  des_body_Q_[3] = 1.0;

  des_r_leg_pos_[0] = 0.0;
  des_r_leg_pos_[1] = -0.093;
  des_r_leg_pos_[2] = 0.0;

  des_r_leg_Q_[0] = 0.0;
  des_r_leg_Q_[1] = 0.0;
  des_r_leg_Q_[2] = 0.0;
  des_r_leg_Q_[3] = 1.0;

  des_l_leg_pos_[0] = 0.0;
  des_l_leg_pos_[1] = 0.093;
  des_l_leg_pos_[2] = 0.0;

  des_l_leg_Q_[0] = 0.0;
  des_l_leg_Q_[1] = 0.0;
  des_l_leg_Q_[2] = 0.0;
  des_l_leg_Q_[3] = 1.0;

  x_lipm_[0] = des_body_pos_[0];
  x_lipm_[1] = 0.0;
  x_lipm_[2] = 0.0;

  y_lipm_[0] = des_body_pos_[1];
  y_lipm_[1] = 0.0;
  y_lipm_[2] = 0.0;
}

void WholebodyModule::parseBalanceGainData(const std::string &path)
{
  YAML::Node doc;
  try
  {
    // load yaml
    doc = YAML::LoadFile(path.c_str());
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("Fail to load yaml file.");
    return;
  }

  //  ROS_INFO("Parse Balance Gain Data");

  foot_roll_gyro_p_gain_  = doc["foot_roll_gyro_p_gain"].as<double>();
  foot_roll_gyro_d_gain_  = doc["foot_roll_gyro_d_gain"].as<double>();
  foot_pitch_gyro_p_gain_ = doc["foot_pitch_gyro_p_gain"].as<double>();
  foot_pitch_gyro_d_gain_ = doc["foot_pitch_gyro_d_gain"].as<double>();

  foot_roll_angle_p_gain_   = doc["foot_roll_angle_p_gain"].as<double>();
  foot_roll_angle_d_gain_   = doc["foot_roll_angle_d_gain"].as<double>();
  foot_pitch_angle_p_gain_  = doc["foot_pitch_angle_p_gain"].as<double>();
  foot_pitch_angle_d_gain_  = doc["foot_pitch_angle_d_gain"].as<double>();

  foot_x_force_p_gain_ = doc["foot_x_force_p_gain"].as<double>();
  foot_x_force_d_gain_ = doc["foot_x_force_d_gain"].as<double>();
  foot_y_force_p_gain_ = doc["foot_y_force_p_gain"].as<double>();
  foot_y_force_d_gain_ = doc["foot_y_force_d_gain"].as<double>();
  foot_z_force_p_gain_ = doc["foot_z_force_p_gain"].as<double>();
  foot_z_force_d_gain_ = doc["foot_z_force_d_gain"].as<double>();

  foot_roll_torque_p_gain_  = doc["foot_roll_torque_p_gain"].as<double>();
  foot_roll_torque_d_gain_  = doc["foot_roll_torque_d_gain"].as<double>();
  foot_pitch_torque_p_gain_ = doc["foot_pitch_torque_p_gain"].as<double>();
  foot_pitch_torque_d_gain_ = doc["foot_pitch_torque_d_gain"].as<double>();

  roll_gyro_cut_off_frequency_  = doc["roll_gyro_cut_off_frequency"].as<double>();
  pitch_gyro_cut_off_frequency_ = doc["pitch_gyro_cut_off_frequency"].as<double>();

  roll_angle_cut_off_frequency_   = doc["roll_angle_cut_off_frequency"].as<double>();
  pitch_angle_cut_off_frequency_  = doc["pitch_angle_cut_off_frequency"].as<double>();

  foot_x_force_cut_off_frequency_ = doc["foot_x_force_cut_off_frequency"].as<double>();
  foot_y_force_cut_off_frequency_ = doc["foot_y_force_cut_off_frequency"].as<double>();
  foot_z_force_cut_off_frequency_ = doc["foot_z_force_cut_off_frequency"].as<double>();

  foot_roll_torque_cut_off_frequency_   = doc["foot_roll_torque_cut_off_frequency"].as<double>();
  foot_pitch_torque_cut_off_frequency_  = doc["foot_pitch_torque_cut_off_frequency"].as<double>();
}

void WholebodyModule::parseJointFeedbackGainData(const std::string &path)
{
  YAML::Node doc;
  try
  {
    // load yaml
    doc = YAML::LoadFile(path.c_str());
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("Fail to load yaml file.");
    return;
  }

  joint_feedback_[joint_name_to_id_["r_leg_hip_y"]-1].p_gain_ = doc["r_leg_hip_y_p_gain"].as<double>();
  joint_feedback_[joint_name_to_id_["r_leg_hip_y"]-1].d_gain_ = doc["r_leg_hip_y_d_gain"].as<double>();
  joint_feedback_[joint_name_to_id_["r_leg_hip_r"]-1].p_gain_ = doc["r_leg_hip_r_p_gain"].as<double>();
  joint_feedback_[joint_name_to_id_["r_leg_hip_r"]-1].d_gain_ = doc["r_leg_hip_r_d_gain"].as<double>();
  joint_feedback_[joint_name_to_id_["r_leg_hip_p"]-1].p_gain_ = doc["r_leg_hip_p_p_gain"].as<double>();
  joint_feedback_[joint_name_to_id_["r_leg_hip_p"]-1].d_gain_ = doc["r_leg_hip_p_d_gain"].as<double>();
  joint_feedback_[joint_name_to_id_["r_leg_kn_p"]-1].p_gain_  = doc["r_leg_kn_p_p_gain"].as<double>();
  joint_feedback_[joint_name_to_id_["r_leg_kn_p"]-1].d_gain_  = doc["r_leg_kn_p_d_gain"].as<double>();
  joint_feedback_[joint_name_to_id_["r_leg_an_p"]-1].p_gain_  = doc["r_leg_an_p_p_gain"].as<double>();
  joint_feedback_[joint_name_to_id_["r_leg_an_p"]-1].d_gain_  = doc["r_leg_an_p_d_gain"].as<double>();
  joint_feedback_[joint_name_to_id_["r_leg_an_r"]-1].p_gain_  = doc["r_leg_an_r_p_gain"].as<double>();
  joint_feedback_[joint_name_to_id_["r_leg_an_r"]-1].d_gain_  = doc["r_leg_an_r_d_gain"].as<double>();

  joint_feedback_[joint_name_to_id_["l_leg_hip_y"]-1].p_gain_ = doc["l_leg_hip_y_p_gain"].as<double>();
  joint_feedback_[joint_name_to_id_["l_leg_hip_y"]-1].d_gain_ = doc["l_leg_hip_y_d_gain"].as<double>();
  joint_feedback_[joint_name_to_id_["l_leg_hip_r"]-1].p_gain_ = doc["l_leg_hip_r_p_gain"].as<double>();
  joint_feedback_[joint_name_to_id_["l_leg_hip_r"]-1].d_gain_ = doc["l_leg_hip_r_d_gain"].as<double>();
  joint_feedback_[joint_name_to_id_["l_leg_hip_p"]-1].p_gain_ = doc["l_leg_hip_p_p_gain"].as<double>();
  joint_feedback_[joint_name_to_id_["l_leg_hip_p"]-1].d_gain_ = doc["l_leg_hip_p_d_gain"].as<double>();
  joint_feedback_[joint_name_to_id_["l_leg_kn_p"]-1].p_gain_  = doc["l_leg_kn_p_p_gain"].as<double>();
  joint_feedback_[joint_name_to_id_["l_leg_kn_p"]-1].d_gain_  = doc["l_leg_kn_p_d_gain"].as<double>();
  joint_feedback_[joint_name_to_id_["l_leg_an_p"]-1].p_gain_  = doc["l_leg_an_p_p_gain"].as<double>();
  joint_feedback_[joint_name_to_id_["l_leg_an_p"]-1].d_gain_  = doc["l_leg_an_p_d_gain"].as<double>();
  joint_feedback_[joint_name_to_id_["l_leg_an_r"]-1].p_gain_  = doc["l_leg_an_r_p_gain"].as<double>();
  joint_feedback_[joint_name_to_id_["l_leg_an_r"]-1].d_gain_  = doc["l_leg_an_r_d_gain"].as<double>();
}

void WholebodyModule::parseJointFeedforwardGainData(const std::string &path)
{
  YAML::Node doc;
  try
  {
    // load yaml
    doc = YAML::LoadFile(path.c_str());
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("Fail to load yaml file.");
    return;
  }

  joint_feedforward_gain_[joint_name_to_id_["r_leg_hip_y"]-1] = doc["r_leg_hip_y_gain"].as<double>();
  joint_feedforward_gain_[joint_name_to_id_["r_leg_hip_r"]-1] = doc["r_leg_hip_r_gain"].as<double>();
  joint_feedforward_gain_[joint_name_to_id_["r_leg_hip_p"]-1] = doc["r_leg_hip_p_gain"].as<double>();
  joint_feedforward_gain_[joint_name_to_id_["r_leg_kn_p"]-1]  = doc["r_leg_kn_p_gain"].as<double>();
  joint_feedforward_gain_[joint_name_to_id_["r_leg_an_p"]-1]  = doc["r_leg_an_p_gain"].as<double>();
  joint_feedforward_gain_[joint_name_to_id_["r_leg_an_r"]-1]  = doc["r_leg_an_r_gain"].as<double>();

  joint_feedforward_gain_[joint_name_to_id_["l_leg_hip_y"]-1] = doc["l_leg_hip_y_gain"].as<double>();
  joint_feedforward_gain_[joint_name_to_id_["l_leg_hip_r"]-1] = doc["l_leg_hip_r_gain"].as<double>();
  joint_feedforward_gain_[joint_name_to_id_["l_leg_hip_p"]-1] = doc["l_leg_hip_p_gain"].as<double>();
  joint_feedforward_gain_[joint_name_to_id_["l_leg_kn_p"]-1]  = doc["l_leg_kn_p_gain"].as<double>();
  joint_feedforward_gain_[joint_name_to_id_["l_leg_an_p"]-1]  = doc["l_leg_an_p_gain"].as<double>();
  joint_feedforward_gain_[joint_name_to_id_["l_leg_an_r"]-1]  = doc["l_leg_an_r_gain"].as<double>();
}

void WholebodyModule::setWholebodyBalanceMsgCallback(const std_msgs::String::ConstPtr& msg)
{
  if (enable_ == false)
    return;

  std::string balance_gain_path = ros::package::getPath("thormang3_wholebody_module") + "/config/balance_gain.yaml";
  parseBalanceGainData(balance_gain_path);

  std::string joint_feedback_gain_path = ros::package::getPath("thormang3_wholebody_module") + "/config/joint_feedback_gain.yaml";
  parseJointFeedbackGainData(joint_feedback_gain_path);

  std::string joint_feedforward_gain_path = ros::package::getPath("thormang3_wholebody_module") + "/config/joint_feedforward_gain.yaml";
  parseJointFeedforwardGainData(joint_feedforward_gain_path);

  if (msg->data == "balance_on")
    goal_balance_gain_ratio_[0] = 1.0;
  else if(msg->data == "balance_off")
    goal_balance_gain_ratio_[0] = 0.0;

  balance_control_initialize_ = false;
  balance_type_ = ON;
  walking_phase_ = DSP;
}

void WholebodyModule::initBalanceControl()
{
  if (balance_control_initialize_ == true)
    return;

  balance_control_initialize_ = true;

  double ini_time = 0.0;
  double mov_time = 1.0;

  balance_step_ = 0;
  balance_size_ = (int) (mov_time / control_cycle_sec_) + 1;

  std::vector<double_t> balance_zero;
  balance_zero.resize(1, 0.0);

  balance_tra_ =
      new robotis_framework::MinimumJerk(ini_time, mov_time,
                                         des_balance_gain_ratio_, balance_zero, balance_zero,
                                         goal_balance_gain_ratio_, balance_zero, balance_zero);

  if (is_balancing_ == true)
    ROS_INFO("[UPDATE] Balance Gain");
  else
  {
    is_balancing_ = true;
    ROS_INFO("[START] Balance Gain");
  }
}

void WholebodyModule::calcBalanceControl()
{
  if (is_balancing_ == true)
  {
    double cur_time = (double) balance_step_ * control_cycle_sec_;
    des_balance_gain_ratio_ = balance_tra_->getPosition(cur_time);

    if (balance_step_ == balance_size_-1)
    {
      balance_step_ = 0;
      is_balancing_ = false;
      delete balance_tra_;

      if (des_balance_gain_ratio_[0] == 0.0)
      {
        control_type_ = NONE;
        balance_type_ = OFF;
      }

      ROS_INFO("[END] Balance Gain");
    }
    else
      balance_step_++;
  }
}

void WholebodyModule::imuDataCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  imu_data_mutex_lock_.lock();

  imu_data_msg_ = *msg;

  imu_data_msg_.angular_velocity.x *= -1.0;
  imu_data_msg_.angular_velocity.y *= -1.0;

  imu_data_mutex_lock_.unlock();
}

void WholebodyModule::leftFootForceTorqueOutputCallback(const geometry_msgs::WrenchStamped::ConstPtr &msg)
{
  Eigen::MatrixXd force = Eigen::MatrixXd::Zero(3,1);
  force.coeffRef(0,0) = msg->wrench.force.x;
  force.coeffRef(1,0) = msg->wrench.force.y;
  force.coeffRef(2,0) = msg->wrench.force.z;

  Eigen::MatrixXd torque = Eigen::MatrixXd::Zero(3,1);
  torque.coeffRef(0,0) = msg->wrench.torque.x;
  torque.coeffRef(1,0) = msg->wrench.torque.y;
  torque.coeffRef(2,0) = msg->wrench.torque.z;

  Eigen::MatrixXd force_new = robotis_framework::getRotationX(M_PI)*robotis_framework::getRotationZ(-0.5*M_PI)*force;
  Eigen::MatrixXd torque_new = robotis_framework::getRotationX(M_PI)*robotis_framework::getRotationZ(-0.5*M_PI)*torque;

  double l_foot_fx_N  = force_new.coeff(0,0);
  double l_foot_fy_N  = force_new.coeff(1,0);
  double l_foot_fz_N  = force_new.coeff(2,0);
  double l_foot_Tx_Nm = torque_new.coeff(0,0);
  double l_foot_Ty_Nm = torque_new.coeff(1,0);
  double l_foot_Tz_Nm = torque_new.coeff(2,0);


  l_foot_fx_N = robotis_framework::sign(l_foot_fx_N) * fmin( fabs(l_foot_fx_N), 2000.0);
  l_foot_fy_N = robotis_framework::sign(l_foot_fy_N) * fmin( fabs(l_foot_fy_N), 2000.0);
  l_foot_fz_N = robotis_framework::sign(l_foot_fz_N) * fmin( fabs(l_foot_fz_N), 2000.0);
  l_foot_Tx_Nm = robotis_framework::sign(l_foot_Tx_Nm) * fmin(fabs(l_foot_Tx_Nm), 300.0);
  l_foot_Ty_Nm = robotis_framework::sign(l_foot_Ty_Nm) * fmin(fabs(l_foot_Ty_Nm), 300.0);
  l_foot_Tz_Nm = robotis_framework::sign(l_foot_Tz_Nm) * fmin(fabs(l_foot_Tz_Nm), 300.0);

  l_foot_ft_data_msg_.force.x = l_foot_fx_N;
  l_foot_ft_data_msg_.force.y = l_foot_fy_N;
  l_foot_ft_data_msg_.force.z = l_foot_fz_N;
  l_foot_ft_data_msg_.torque.x = l_foot_Tx_Nm;
  l_foot_ft_data_msg_.torque.y = l_foot_Ty_Nm;
  l_foot_ft_data_msg_.torque.z = l_foot_Tz_Nm;
}

void WholebodyModule::rightFootForceTorqueOutputCallback(const geometry_msgs::WrenchStamped::ConstPtr &msg)
{
  Eigen::MatrixXd force = Eigen::MatrixXd::Zero(3,1);
  force.coeffRef(0,0) = msg->wrench.force.x;
  force.coeffRef(1,0) = msg->wrench.force.y;
  force.coeffRef(2,0) = msg->wrench.force.z;

  Eigen::MatrixXd torque = Eigen::MatrixXd::Zero(3,1);
  torque.coeffRef(0,0) = msg->wrench.torque.x;
  torque.coeffRef(1,0) = msg->wrench.torque.y;
  torque.coeffRef(2,0) = msg->wrench.torque.z;

  Eigen::MatrixXd force_new = robotis_framework::getRotationX(M_PI)*robotis_framework::getRotationZ(-0.5*M_PI)*force;
  Eigen::MatrixXd torque_new = robotis_framework::getRotationX(M_PI)*robotis_framework::getRotationZ(-0.5*M_PI)*torque;

  double r_foot_fx_N  = force_new.coeff(0,0);
  double r_foot_fy_N  = force_new.coeff(1,0);
  double r_foot_fz_N  = force_new.coeff(2,0);
  double r_foot_Tx_Nm = torque_new.coeff(0,0);
  double r_foot_Ty_Nm = torque_new.coeff(1,0);
  double r_foot_Tz_Nm = torque_new.coeff(2,0);

  r_foot_fx_N = robotis_framework::sign(r_foot_fx_N) * fmin( fabs(r_foot_fx_N), 2000.0);
  r_foot_fy_N = robotis_framework::sign(r_foot_fy_N) * fmin( fabs(r_foot_fy_N), 2000.0);
  r_foot_fz_N = robotis_framework::sign(r_foot_fz_N) * fmin( fabs(r_foot_fz_N), 2000.0);
  r_foot_Tx_Nm = robotis_framework::sign(r_foot_Tx_Nm) *fmin(fabs(r_foot_Tx_Nm), 300.0);
  r_foot_Ty_Nm = robotis_framework::sign(r_foot_Ty_Nm) *fmin(fabs(r_foot_Ty_Nm), 300.0);
  r_foot_Tz_Nm = robotis_framework::sign(r_foot_Tz_Nm) *fmin(fabs(r_foot_Tz_Nm), 300.0);

  r_foot_ft_data_msg_.force.x = r_foot_fx_N;
  r_foot_ft_data_msg_.force.y = r_foot_fy_N;
  r_foot_ft_data_msg_.force.z = r_foot_fz_N;
  r_foot_ft_data_msg_.torque.x = r_foot_Tx_Nm;
  r_foot_ft_data_msg_.torque.y = r_foot_Ty_Nm;
  r_foot_ft_data_msg_.torque.z = r_foot_Tz_Nm;
}

void WholebodyModule::setResetBodyCallback(const std_msgs::Bool::ConstPtr& msg)
{
  if (msg->data == true)
  {
    des_body_offset_[0] = 0.0;
    des_body_offset_[1] = 0.0;
    des_body_offset_[2] = 0.0;

    resetBodyPose();
  }
}

void WholebodyModule::walkingParamCallback(const thormang3_wholebody_module_msgs::WalkingParam& msg)
{
  walking_param_ = msg;
}

void WholebodyModule::goalJointPoseCallback(const thormang3_wholebody_module_msgs::JointPose& msg)
{
  if (enable_ == false)
    return;

  size_t joint_size = msg.pose.name.size();

  if (control_type_ == NONE || control_type_ == JOINT_CONTROL)
  {
    mov_time_ = msg.mov_time;

    for (size_t i = 0; i < msg.pose.name.size(); i++)
    {
      std::string joint_name = msg.pose.name[i];
      goal_joint_pos_[joint_name_to_id_[joint_name] - 1] = msg.pose.position[i];
    }

    joint_control_initialize_ = false;
    control_type_ = JOINT_CONTROL;
    balance_type_ = OFF;
    des_balance_gain_ratio_[0] = 0.0;
  }
  else
    ROS_WARN("[WARN] Control type is different!");
}

void WholebodyModule::initJointControl()
{
  if (joint_control_initialize_ == true)
    return;

  joint_control_initialize_ = true;

  double ini_time = 0.0;
  double mov_time = mov_time_;

  mov_step_ = 0;
  mov_size_ = (int) (mov_time / control_cycle_sec_) + 1;

  joint_tra_ =
      new robotis_framework::MinimumJerk(ini_time, mov_time,
                                         des_joint_pos_, des_joint_vel_, des_joint_accel_,
                                         goal_joint_pos_, goal_joint_vel_, goal_joint_accel_);
  if (is_moving_ == true)
    ROS_INFO("[UPDATE] Joint Control");
  else
  {
    is_moving_ = true;
    ROS_INFO("[START] Joint Control");
  }
}

void WholebodyModule::calcJointControl()
{
  if (is_moving_ == true)
  {
    double cur_time = (double) mov_step_ * control_cycle_sec_;

    queue_mutex_.lock();

    des_joint_pos_ = joint_tra_->getPosition(cur_time);
    des_joint_vel_ = joint_tra_->getVelocity(cur_time);
    des_joint_accel_ = joint_tra_->getAcceleration(cur_time);

    queue_mutex_.unlock();

    if (mov_step_ == mov_size_-1)
    {
      mov_step_ = 0;
      is_moving_ = false;
      delete joint_tra_;

      control_type_ = NONE;

      ROS_INFO("[END] Joint Control");
    }
    else
      mov_step_++;
  }
}

void WholebodyModule::setBodyOffsetCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
  if (enable_ == false)
    return;

  if (balance_type_ == OFF)
  {
    ROS_WARN("[WARN] Balance is off!");
    return;
  }

  if (control_type_ == NONE || control_type_ == OFFSET_CONTROL)
  {
    goal_body_offset_[0] = msg->position.x;
    goal_body_offset_[1] = msg->position.y;
    goal_body_offset_[2] = msg->position.z;

    body_offset_initialize_ = false;
    control_type_ = OFFSET_CONTROL;
  }
  else
    ROS_WARN("[WARN] Control type is different!");
}

void WholebodyModule::initOffsetControl()
{
  if (body_offset_initialize_ == true)
    return;

  body_offset_initialize_ = true;

  double ini_time = 0.0;
  double mov_time = 1.0;

  body_offset_step_ = 0;
  body_offset_size_ = (int) (mov_time / control_cycle_sec_) + 1;

  std::vector<double_t> offset_zero;
  offset_zero.resize(2, 0.0);

  body_offset_tra_ =
      new robotis_framework::MinimumJerk(ini_time, mov_time,
                                         des_body_offset_, offset_zero, offset_zero,
                                         goal_body_offset_, offset_zero, offset_zero);

  if (is_moving_ == true)
    ROS_INFO("[UPDATE] Body Offset");
  else
  {
    is_moving_ = true;
    ROS_INFO("[START] Body Offset");
  }
}

void WholebodyModule::calcOffsetControl()
{
  if (is_moving_ == true)
  {
    double cur_time = (double) body_offset_step_ * control_cycle_sec_;

    queue_mutex_.lock();

    des_body_offset_ = body_offset_tra_->getPosition(cur_time);

    queue_mutex_.unlock();

    if (body_offset_step_ == mov_size_-1)
    {
      body_offset_step_ = 0;
      is_moving_ = false;
      delete body_offset_tra_;

      control_type_ = NONE;

      ROS_INFO("[END] Body Offset");
    }
    else
      body_offset_step_++;
  }
}

void WholebodyModule::goalKinematicsPoseCallback(const thormang3_wholebody_module_msgs::KinematicsPose& msg)
{
  if (enable_ == false)
    return;

  if (balance_type_ == OFF)
  {
    ROS_WARN("[WARN] Balance is off!");
    return;
  }

  if (control_type_ == NONE || control_type_ == WHOLEBODY_CONTROL)
  {
    if (is_moving_ == true)
    {
      if (wholegbody_control_group_!=msg.name)
      {
        ROS_WARN("[WARN] Control group is different!");
        return;
      }
    }
    mov_time_ = msg.mov_time;
    wholegbody_control_group_ = msg.name;
    wholebody_goal_msg_ = msg.pose;

    wholebody_initialize_ = false;
    control_type_ = WHOLEBODY_CONTROL;
  }
  else
    ROS_WARN("[WARN] Control type is different!");
}

void WholebodyModule::initWholebodyControl()
{
  if (wholebody_initialize_ == true)
    return;

  wholebody_initialize_ = true;

  double ini_time = 0.0;
  double mov_time = mov_time_;

  mov_step_ = 0;
  mov_size_ = (int) (mov_time / control_cycle_sec_) + 1;

  wholebody_control_ =
      new WholebodyControl(wholegbody_control_group_,
                           ini_time, mov_time,
                           wholebody_goal_msg_);

  if (is_moving_ == true)
  {
    //    ROS_INFO("[UPDATE] Wholebody Control");
    //    wholebody_control_->update(desired_body_position_, desired_body_orientation_,
    //                               desired_task_position_, desired_task_velocity_, desired_task_acceleration_);
  }
  else
  {
    ROS_INFO("[START] Wholebody Control");

    wholebody_control_->initialize(des_body_pos_, des_body_Q_,
                                   des_r_leg_pos_, des_r_leg_Q_,
                                   des_l_leg_pos_, des_l_leg_Q_);
    is_moving_ = true;
  }
}

void WholebodyModule::calcWholebodyControl()
{
  if (is_moving_ == true)
  {
    double cur_time = (double) mov_step_ * control_cycle_sec_;

    wholebody_control_->set(cur_time);

    wholebody_control_->getTaskPosition(des_l_leg_pos_,
                                        des_r_leg_pos_,
                                        des_body_pos_);
    wholebody_control_->getTaskOrientation(des_l_leg_Q_,
                                           des_r_leg_Q_,
                                           des_body_Q_);

    if (mov_step_ == mov_size_-1)
    {
      mov_step_ = 0;
      is_moving_ = false;
      wholebody_control_->finalize();

      control_type_ = NONE;

      ROS_INFO("[END] Wholebody Control");
    }
    else
      mov_step_++;
  }
}

void WholebodyModule::footStep2DCallback(const thormang3_foot_step_generator::Step2DArray& msg)
{
  if (enable_ == false)
    return;

  if (balance_type_ == OFF)
  {
    ROS_WARN("[WARN] Balance is off!");
    return;
  }

  thormang3_foot_step_generator::Step2DArray foot_step_msg;

  int old_size = msg.footsteps_2d.size();
  int new_size = old_size + 3;

  thormang3_foot_step_generator::Step2D first_msg;
  thormang3_foot_step_generator::Step2D second_msg;

  first_msg.moving_foot = msg.footsteps_2d[0].moving_foot - 1;
  second_msg.moving_foot = first_msg.moving_foot + 1;

  if (first_msg.moving_foot == LEFT_LEG)
  {
    first_msg.step2d.x = des_l_leg_pos_[0];
    first_msg.step2d.y = des_l_leg_pos_[1];
    first_msg.step2d.theta = 0.0;

    second_msg.step2d.x = des_r_leg_pos_[0];
    second_msg.step2d.y = des_r_leg_pos_[1];
    second_msg.step2d.theta = 0.0;
  }
  else if (first_msg.moving_foot == RIGHT_LEG)
  {
    first_msg.step2d.x = des_r_leg_pos_[0];
    first_msg.step2d.y = des_r_leg_pos_[1];
    first_msg.step2d.theta = 0.0;

    second_msg.step2d.x = des_l_leg_pos_[0];
    second_msg.step2d.y = des_l_leg_pos_[1];
    second_msg.step2d.theta = 0.0;
  }

  foot_step_msg.footsteps_2d.push_back(first_msg);
  foot_step_msg.footsteps_2d.push_back(second_msg);

  if (control_type_ == NONE || control_type_ == WALKING_CONTROL)
  {
    for (int i=0; i<old_size; i++)
    {
      thormang3_foot_step_generator::Step2D step_msg = msg.footsteps_2d[i];
      step_msg.moving_foot -= 1;

      foot_step_msg.footsteps_2d.push_back(step_msg);

//      ROS_INFO("===== OLD =====");
//      ROS_INFO("step: %d", i);
//      ROS_INFO("foot_step_2d_.footsteps_2d[%d].moving_foot: %d", i, step_msg.moving_foot);
//      ROS_INFO("foot_step_2d_.footsteps_2d[%d].step2d x: %f", i, step_msg.step2d.x);
//      ROS_INFO("foot_step_2d_.footsteps_2d[%d].step2d y: %f", i, step_msg.step2d.y);
//      ROS_INFO("foot_step_2d_.footsteps_2d[%d].step2d theta: %f", i, step_msg.step2d.theta);
    }

    thormang3_foot_step_generator::Step2D step_msg = msg.footsteps_2d[old_size-1];

    if (step_msg.moving_foot - 1 == LEFT_LEG)
      first_msg.moving_foot = RIGHT_LEG;
    else
      first_msg.moving_foot = LEFT_LEG;

    first_msg.step2d.x      = 0.0;
    first_msg.step2d.y      = 0.0;
    first_msg.step2d.theta  = step_msg.step2d.theta;

    foot_step_msg.footsteps_2d.push_back(first_msg);

//    for (int i=0; i<new_size; i++)
//    {
//      thormang3_foot_step_generator::Step2D step_msg = foot_step_msg.footsteps_2d[i];

//      ROS_INFO("===== NEW =====");
//      ROS_INFO("step: %d", i);
//      ROS_INFO("foot_step_2d_.footsteps_2d[%d].moving_foot: %d", i, step_msg.moving_foot);
//      ROS_INFO("foot_step_2d_.footsteps_2d[%d].step2d x: %f", i, step_msg.step2d.x);
//      ROS_INFO("foot_step_2d_.footsteps_2d[%d].step2d y: %f", i, step_msg.step2d.y);
//      ROS_INFO("foot_step_2d_.footsteps_2d[%d].step2d theta: %f", i, step_msg.step2d.theta);
//    }

    foot_step_2d_ = foot_step_msg;

    walking_size_ = new_size;
    mov_time_ = 1.0;
    is_foot_step_2d_ = true;
    control_type_ = WALKING_CONTROL;

    if (is_moving_ == false)
      initWalkingControl();
    else
      ROS_WARN("[WARN] Previous task is alive!");
  }
  else
    ROS_WARN("[WARN] Control type is different!");



}

void WholebodyModule::footStepCommandCallback(const thormang3_wholebody_module_msgs::FootStepCommand& msg)
{
  if (enable_ == false)
    return;

  if (balance_type_ == OFF)
  {
    ROS_WARN("[WARN] Balance is off!");
    return;
  }

  is_foot_step_2d_ = false;

  if (control_type_ == NONE || control_type_ == WALKING_CONTROL)
  {
    walking_size_ = msg.step_num + 2;
    mov_time_ = msg.step_time;

    foot_step_command_ = msg;
    foot_step_command_.step_num = walking_size_;

    control_type_ = WALKING_CONTROL;

    if (is_moving_ == false)
      initWalkingControl();
    else
      ROS_WARN("[WARN] Previous task is alive!");
  }
  else
    ROS_WARN("[WARN] Control type is different!");
}

void WholebodyModule::initWalkingControl()
{
  double mov_time = mov_time_;

  mov_step_ = 0;
  mov_size_ = (int) (mov_time / control_cycle_sec_) + 1;

  walking_step_ = 0;

  walking_control_ = new WalkingControl(control_cycle_sec_,
                                        walking_param_.dsp_ratio, walking_param_.lipm_height, walking_param_.foot_height_max,
                                        walking_param_.zmp_offset_x, walking_param_.zmp_offset_y,
                                        x_lipm_, y_lipm_);

  double lipm_height = walking_control_->getLipmHeight();
  preview_request_.lipm_height = lipm_height;
  preview_request_.control_cycle = control_cycle_sec_;

  bool get_preview_matrix = false;
  get_preview_matrix = getPreviewMatrix(preview_request_);

  if (get_preview_matrix == true)
  {
    if (is_moving_ == true)
    {
      // Update - TODO
      //      walking_control_->update(walking_leg_, walking_phase_,
      //                               foot_step_command_,
      //                               des_body_pos_, des_body_Q_,
      //                               des_l_leg_pos_, des_l_leg_vel_, des_l_leg_accel_,
      //                               des_r_leg_pos_, des_r_leg_vel_, des_r_leg_accel_);

      //      walking_control_->calcPreviewParam(preview_response_);
      //      ROS_INFO("[UPDATE] Walking Control (%d/%d)", walking_step_+1, walking_size_);
    }
    else
    {
      if (is_foot_step_2d_ == true)
      {
        walking_control_->initialize(foot_step_2d_,
                                     des_body_pos_, des_body_Q_,
                                     des_r_leg_pos_, des_r_leg_Q_,
                                     des_l_leg_pos_, des_l_leg_Q_);
      }
      else
      {
        walking_control_->initialize(foot_step_command_,
                                     des_body_pos_, des_body_Q_,
                                     des_r_leg_pos_, des_r_leg_Q_,
                                     des_l_leg_pos_, des_l_leg_Q_);
      }

      walking_control_->calcPreviewParam(preview_response_);
      is_moving_ = true;

      initFeedforwardControl();

      ROS_INFO("[START] Walking Control (%d/%d)", walking_step_+1, walking_size_);
    }

    walking_initialize_ = true;
  }
  else
    ROS_WARN("[FAIL] Cannot get preview matrix");
}

void WholebodyModule::calcWalkingControl()
{
  if (is_moving_ == true)
  {
    double cur_time = (double) mov_step_ * control_cycle_sec_;
    walking_control_->set(cur_time, walking_step_,is_foot_step_2d_);

//    ROS_INFO("cur_time: %f", cur_time);

    //      queue_mutex_.lock();

    // Set joint position
    //      desired_joint_position_ = walking_control_->getJointPosition(walking_step_, cur_time);
    //      desired_joint_velocity_ = walking_control_->getJointVelocity(walking_step_, cur_time);
    //      desired_joint_acceleration_ = walking_control_->getJointAcceleration(walking_step_, cur_time);

    //      queue_mutex_.unlock();

    walking_control_->getWalkingPosition(des_l_leg_pos_,
                                         des_r_leg_pos_,
                                         des_body_pos_);
    //      walking_control_->getWalkingVelocity(desired_left_foot_velocity_,
    //                                           desired_right_foot_velocity_,
    //                                           desired_body_velocity_);
    //      walking_control_->getWalkingAccleration(desired_left_foot_acceleration_,
    //                                              desired_right_foot_acceleration_,
    //                                              desired_body_acceleration_);
    walking_control_->getWalkingOrientation(des_l_leg_Q_,
                                            des_r_leg_Q_,
                                            des_body_Q_);

    //      ROS_INFO("body x: %f, y: %f, z: %f", desired_body_position_[0], desired_body_position_[1], desired_body_position_[2]);
    //      ROS_INFO("lfoot x: %f, y: %f, z: %f", desired_left_foot_position_[0], desired_left_foot_position_[1], desired_left_foot_position_[2]);
    //      ROS_INFO("rfoot x: %f, y: %f, z: %f", desired_right_foot_position_[0], desired_right_foot_position_[1], desired_right_foot_position_[2]);

    walking_control_->getLIPM(x_lipm_, y_lipm_);

    //      ROS_INFO("x_lipm_ pos: %f , vel: %f , accel: %f", x_lipm_[0], x_lipm_[1], x_lipm_[2]);
    //      ROS_INFO("y_lipm_ pos: %f , vel: %f , accel: %f", y_lipm_[0], y_lipm_[1], y_lipm_[2]);

    walking_control_->getWalkingState(walking_leg_, walking_phase_);


    if (mov_step_ == mov_size_-1)
    {
      ROS_INFO("[END] Walking Control (%d/%d)", walking_step_+1, walking_size_);

      mov_step_ = 0;
      walking_control_->next();

      if (walking_step_ == walking_size_-1)
      {
        is_moving_ = false;
        is_foot_step_2d_ = false;
        walking_control_->finalize();
        resetBodyPose();

        control_type_ = NONE;
        walking_phase_ = DSP;
      }
      else
      {
        walking_step_++;
        ROS_INFO("[START] Walking Control (%d/%d)", walking_step_+1, walking_size_);
      }
    }
    else
      mov_step_++;
  }
}

void WholebodyModule::initFeedforwardControl()
{
  // feedforward trajectory
  std::vector<double_t> zero_vector;
  zero_vector.resize(1,0.0);

  std::vector<double_t> via_pos;
  via_pos.resize(3, 0.0);
  via_pos[0] = 1.0 * DEGREE2RADIAN;

  double init_time = 0.0;
  double fin_time = foot_step_command_.step_time;
  double via_time = 0.5 * (init_time + fin_time);
  double dsp_ratio = walking_param_.dsp_ratio;

  feed_forward_tra_ =
      new robotis_framework::MinimumJerkViaPoint(init_time, fin_time, via_time, dsp_ratio,
                                                 zero_vector, zero_vector, zero_vector,
                                                 zero_vector, zero_vector, zero_vector,
                                                 via_pos, zero_vector, zero_vector);
}

void WholebodyModule::calcRobotPose()
{
  Eigen::MatrixXd des_body_pos = Eigen::MatrixXd::Zero(3,1);
  des_body_pos.coeffRef(0,0) = des_body_pos_[0];
  des_body_pos.coeffRef(1,0) = des_body_pos_[1];
  des_body_pos.coeffRef(2,0) = des_body_pos_[2];

  Eigen::Quaterniond des_body_Q(des_body_Q_[3],des_body_Q_[0],des_body_Q_[1],des_body_Q_[2]);
  Eigen::MatrixXd des_body_rot = robotis_framework::convertQuaternionToRotation(des_body_Q);

  // Forward Kinematics
  thormang3_kdl_->initialize(des_body_pos, des_body_rot);

  Eigen::VectorXd r_leg_joint_pos, l_leg_joint_pos;

  r_leg_joint_pos.resize(6);
  r_leg_joint_pos(0) = des_joint_pos_[joint_name_to_id_["r_leg_hip_y"]-1];
  r_leg_joint_pos(1) = des_joint_pos_[joint_name_to_id_["r_leg_hip_r"]-1];
  r_leg_joint_pos(2) = des_joint_pos_[joint_name_to_id_["r_leg_hip_p"]-1];
  r_leg_joint_pos(3) = des_joint_pos_[joint_name_to_id_["r_leg_kn_p"]-1];
  r_leg_joint_pos(4) = des_joint_pos_[joint_name_to_id_["r_leg_an_p"]-1];
  r_leg_joint_pos(5) = des_joint_pos_[joint_name_to_id_["r_leg_an_r"]-1];

  l_leg_joint_pos.resize(6);
  l_leg_joint_pos(0) = des_joint_pos_[joint_name_to_id_["l_leg_hip_y"]-1];
  l_leg_joint_pos(1) = des_joint_pos_[joint_name_to_id_["l_leg_hip_r"]-1];
  l_leg_joint_pos(2) = des_joint_pos_[joint_name_to_id_["l_leg_hip_p"]-1];
  l_leg_joint_pos(3) = des_joint_pos_[joint_name_to_id_["l_leg_kn_p"]-1];
  l_leg_joint_pos(4) = des_joint_pos_[joint_name_to_id_["l_leg_an_p"]-1];
  l_leg_joint_pos(5) = des_joint_pos_[joint_name_to_id_["l_leg_an_r"]-1];

  thormang3_kdl_->setJointPosition(r_leg_joint_pos, l_leg_joint_pos);

  std::vector<double_t> r_leg_pos, r_leg_Q;
  r_leg_pos.resize(3,0.0);
  r_leg_Q.resize(4,0.0);

  std::vector<double_t> l_leg_pos, l_leg_Q;
  l_leg_pos.resize(3,0.0);
  l_leg_Q.resize(4,0.0);

  std::vector<double_t> r_leg_ft_pos, r_leg_ft_Q;
  r_leg_ft_pos.resize(3,0.0);
  r_leg_ft_Q.resize(4,0.0);

  std::vector<double_t> l_leg_ft_pos, l_leg_ft_Q;
  l_leg_ft_pos.resize(3,0.0);
  l_leg_ft_Q.resize(4,0.0);

  thormang3_kdl_->solveForwardKinematics(r_leg_pos, r_leg_Q,
                                         l_leg_pos, l_leg_Q,
                                         r_leg_ft_pos, r_leg_ft_Q,
                                         l_leg_ft_pos, l_leg_ft_Q);

  Eigen::Quaterniond curr_r_leg_Q(r_leg_Q[3],r_leg_Q[0],r_leg_Q[1],r_leg_Q[2]);
  Eigen::MatrixXd curr_r_leg_rot = robotis_framework::convertQuaternionToRotation(curr_r_leg_Q);

  Eigen::MatrixXd g_to_r_leg = Eigen::MatrixXd::Identity(4,4);
  g_to_r_leg.block(0,0,3,3) = curr_r_leg_rot;
  g_to_r_leg.coeffRef(0,3) = r_leg_pos[0];
  g_to_r_leg.coeffRef(1,3) = r_leg_pos[1];
  g_to_r_leg.coeffRef(2,3) = r_leg_pos[2];

  Eigen::Quaterniond curr_l_leg_Q(l_leg_Q[3],l_leg_Q[0],l_leg_Q[1],l_leg_Q[2]);
  Eigen::MatrixXd curr_l_leg_rot = robotis_framework::convertQuaternionToRotation(curr_l_leg_Q);

  Eigen::MatrixXd g_to_l_leg = Eigen::MatrixXd::Identity(4,4);
  g_to_l_leg.block(0,0,3,3) = curr_l_leg_rot;
  g_to_l_leg.coeffRef(0,3) = l_leg_pos[0];
  g_to_l_leg.coeffRef(1,3) = l_leg_pos[1];
  g_to_l_leg.coeffRef(2,3) = l_leg_pos[2];

  //  PRINT_MAT(g_to_r_leg);
  //  PRINT_MAT(g_to_l_leg);

  Eigen::Quaterniond curr_r_leg_ft_Q(r_leg_ft_Q[3],r_leg_ft_Q[0],r_leg_ft_Q[1],r_leg_ft_Q[2]);
  Eigen::MatrixXd curr_r_leg_ft_rot = robotis_framework::convertQuaternionToRotation(curr_r_leg_ft_Q);

  Eigen::MatrixXd g_to_r_leg_ft = Eigen::MatrixXd::Identity(4,4);
  g_to_r_leg_ft.block(0,0,3,3) = curr_r_leg_ft_rot;
  g_to_r_leg_ft.coeffRef(0,3) = r_leg_ft_pos[0];
  g_to_r_leg_ft.coeffRef(1,3) = r_leg_ft_pos[1];
  g_to_r_leg_ft.coeffRef(2,3) = r_leg_ft_pos[2];

  Eigen::Quaterniond curr_l_leg_ft_Q(l_leg_ft_Q[3],l_leg_ft_Q[0],l_leg_ft_Q[1],l_leg_ft_Q[2]);
  Eigen::MatrixXd curr_l_leg_ft_rot = robotis_framework::convertQuaternionToRotation(curr_l_leg_ft_Q);

  Eigen::MatrixXd g_to_l_leg_ft = Eigen::MatrixXd::Identity(4,4);
  g_to_l_leg_ft.block(0,0,3,3) = curr_l_leg_ft_rot;
  g_to_l_leg_ft.coeffRef(0,3) = l_leg_ft_pos[0];
  g_to_l_leg_ft.coeffRef(1,3) = l_leg_ft_pos[1];
  g_to_l_leg_ft.coeffRef(2,3) = l_leg_ft_pos[2];

  //  PRINT_MAT(g_to_r_leg_ft);
  //  PRINT_MAT(g_to_l_leg_ft);

  //  des_r_leg_pos_ = r_leg_pos;
  //  des_l_leg_pos_ = l_leg_pos;

  thormang3_kdl_->finalize();
}

void WholebodyModule::setTargetForceTorque()
{
  if (walking_phase_ == DSP)
  {
    balance_r_foot_force_x_ = -0.5 * total_mass_ * x_lipm_[2];
    balance_r_foot_force_y_ = -0.5 * total_mass_ * y_lipm_[2];
    balance_r_foot_force_z_ = -0.5 * total_mass_ * 9.81;

    balance_l_foot_force_x_ = -0.5 * total_mass_ * x_lipm_[2];
    balance_l_foot_force_y_ = -0.5 * total_mass_ * y_lipm_[2];
    balance_l_foot_force_z_ = -0.5 * total_mass_ * 9.81;
  }
  else if (walking_phase_ == SSP)
  {
    if (walking_leg_ == LEFT_LEG)
    {
      balance_r_foot_force_x_ = -1.0 * total_mass_ * x_lipm_[2];
      balance_r_foot_force_y_ = -1.0 * total_mass_ * y_lipm_[2];
      balance_r_foot_force_z_ = -1.0 * total_mass_ * 9.81;

      balance_l_foot_force_x_ = 0.0;
      balance_l_foot_force_y_ = 0.0;
      balance_l_foot_force_z_ = 0.0;
    }
    else if (walking_leg_ == RIGHT_LEG)
    {
      balance_r_foot_force_x_ = 0.0;
      balance_r_foot_force_y_ = 0.0;
      balance_r_foot_force_z_ = 0.0;

      balance_l_foot_force_x_ = -1.0 * total_mass_ * x_lipm_[2];
      balance_l_foot_force_y_ = -1.0 * total_mass_ * y_lipm_[2];
      balance_l_foot_force_z_ = -1.0 * total_mass_ * 9.81;
    }
  }

  //  ROS_INFO("r_foot_force x: %f, y: %f, z: %f", balance_r_foot_force_x_, balance_r_foot_force_y_, balance_r_foot_force_z_);
  //  ROS_INFO("l_foot_force x: %f, y: %f, z: %f", balance_l_foot_force_x_, balance_l_foot_force_y_, balance_l_foot_force_z_);
}

void WholebodyModule::setBalanceControlGain()
{
  //// set gain
  //gyro
  balance_control_.foot_roll_gyro_ctrl_.p_gain_ = foot_roll_gyro_p_gain_ * des_balance_gain_ratio_[0];
  balance_control_.foot_roll_gyro_ctrl_.d_gain_ = foot_roll_gyro_d_gain_ * des_balance_gain_ratio_[0];
  balance_control_.foot_pitch_gyro_ctrl_.p_gain_ = foot_pitch_gyro_p_gain_ * des_balance_gain_ratio_[0];
  balance_control_.foot_pitch_gyro_ctrl_.d_gain_ = foot_pitch_gyro_d_gain_ * des_balance_gain_ratio_[0];

  //orientation
  balance_control_.foot_roll_angle_ctrl_.p_gain_  = foot_roll_angle_p_gain_ * des_balance_gain_ratio_[0];
  balance_control_.foot_roll_angle_ctrl_.d_gain_  = foot_roll_angle_d_gain_ * des_balance_gain_ratio_[0];
  balance_control_.foot_pitch_angle_ctrl_.p_gain_ = foot_pitch_angle_p_gain_ * des_balance_gain_ratio_[0];
  balance_control_.foot_pitch_angle_ctrl_.d_gain_ = foot_pitch_angle_d_gain_ * des_balance_gain_ratio_[0];

  //force torque
  balance_control_.right_foot_force_x_ctrl_.p_gain_      = foot_x_force_p_gain_ * des_balance_gain_ratio_[0];
  balance_control_.right_foot_force_y_ctrl_.p_gain_      = foot_y_force_p_gain_ * des_balance_gain_ratio_[0];
  balance_control_.right_foot_force_z_ctrl_.p_gain_      = foot_z_force_p_gain_ * des_balance_gain_ratio_[0];
  balance_control_.right_foot_torque_roll_ctrl_.p_gain_  = foot_roll_torque_p_gain_ * des_balance_gain_ratio_[0];
  balance_control_.right_foot_torque_pitch_ctrl_.p_gain_ = foot_roll_torque_p_gain_ * des_balance_gain_ratio_[0];
  balance_control_.right_foot_force_x_ctrl_.d_gain_      = foot_x_force_d_gain_ * des_balance_gain_ratio_[0];
  balance_control_.right_foot_force_y_ctrl_.d_gain_      = foot_y_force_d_gain_ * des_balance_gain_ratio_[0];
  balance_control_.right_foot_force_z_ctrl_.d_gain_      = foot_z_force_d_gain_ * des_balance_gain_ratio_[0];
  balance_control_.right_foot_torque_roll_ctrl_.d_gain_  = foot_roll_torque_d_gain_ * des_balance_gain_ratio_[0];
  balance_control_.right_foot_torque_pitch_ctrl_.d_gain_ = foot_roll_torque_d_gain_ * des_balance_gain_ratio_[0];

  balance_control_.left_foot_force_x_ctrl_.p_gain_      = foot_x_force_p_gain_ * des_balance_gain_ratio_[0];
  balance_control_.left_foot_force_y_ctrl_.p_gain_      = foot_y_force_p_gain_ * des_balance_gain_ratio_[0];
  balance_control_.left_foot_force_z_ctrl_.p_gain_      = foot_z_force_p_gain_ * des_balance_gain_ratio_[0];
  balance_control_.left_foot_torque_roll_ctrl_.p_gain_  = foot_roll_torque_p_gain_ * des_balance_gain_ratio_[0];
  balance_control_.left_foot_torque_pitch_ctrl_.p_gain_ = foot_roll_torque_p_gain_ * des_balance_gain_ratio_[0];
  balance_control_.left_foot_force_x_ctrl_.d_gain_      = foot_x_force_d_gain_ * des_balance_gain_ratio_[0];
  balance_control_.left_foot_force_y_ctrl_.d_gain_      = foot_y_force_d_gain_ * des_balance_gain_ratio_[0];
  balance_control_.left_foot_force_z_ctrl_.d_gain_      = foot_z_force_d_gain_ * des_balance_gain_ratio_[0];
  balance_control_.left_foot_torque_roll_ctrl_.d_gain_  = foot_roll_torque_d_gain_ * des_balance_gain_ratio_[0];
  balance_control_.left_foot_torque_pitch_ctrl_.d_gain_ = foot_roll_torque_d_gain_ * des_balance_gain_ratio_[0];

  //// set cut off freq
  balance_control_.roll_gyro_lpf_.setCutOffFrequency(roll_gyro_cut_off_frequency_);
  balance_control_.pitch_gyro_lpf_.setCutOffFrequency(pitch_gyro_cut_off_frequency_);
  balance_control_.roll_angle_lpf_.setCutOffFrequency(roll_angle_cut_off_frequency_);
  balance_control_.pitch_angle_lpf_.setCutOffFrequency(pitch_angle_cut_off_frequency_);

  balance_control_.right_foot_force_x_lpf_.setCutOffFrequency(foot_x_force_cut_off_frequency_);
  balance_control_.right_foot_force_y_lpf_.setCutOffFrequency(foot_y_force_cut_off_frequency_);
  balance_control_.right_foot_force_z_lpf_.setCutOffFrequency(foot_z_force_cut_off_frequency_);
  balance_control_.right_foot_torque_roll_lpf_.setCutOffFrequency(foot_roll_torque_cut_off_frequency_);
  balance_control_.right_foot_torque_pitch_lpf_.setCutOffFrequency(foot_pitch_torque_cut_off_frequency_);

  balance_control_.left_foot_force_x_lpf_.setCutOffFrequency(foot_x_force_cut_off_frequency_);
  balance_control_.left_foot_force_y_lpf_.setCutOffFrequency(foot_y_force_cut_off_frequency_);
  balance_control_.left_foot_force_z_lpf_.setCutOffFrequency(foot_z_force_cut_off_frequency_);
  balance_control_.left_foot_torque_roll_lpf_.setCutOffFrequency(foot_roll_torque_cut_off_frequency_);
  balance_control_.left_foot_torque_pitch_lpf_.setCutOffFrequency(foot_pitch_torque_cut_off_frequency_);
}

bool WholebodyModule::setBalanceControl()
{
  // Set Balance Control
  balance_control_.setGyroBalanceEnable(true);
  balance_control_.setOrientationBalanceEnable(true);
  balance_control_.setForceTorqueBalanceEnable(true);

  balance_control_.setCOBManualAdjustment(des_body_offset_[0], des_body_offset_[1], des_body_offset_[2]);

  setBalanceControlGain();
  setTargetForceTorque();

  bool ik_success = true;

  // Body Pose
  Eigen::MatrixXd des_body_pos = Eigen::MatrixXd::Zero(3,1);
  des_body_pos.coeffRef(0,0) = des_body_pos_[0];
  des_body_pos.coeffRef(1,0) = des_body_pos_[1];
  des_body_pos.coeffRef(2,0) = des_body_pos_[2];

  Eigen::Quaterniond des_body_Q(des_body_Q_[3],des_body_Q_[0],des_body_Q_[1],des_body_Q_[2]);
  Eigen::MatrixXd des_body_rot = robotis_framework::convertQuaternionToRotation(des_body_Q);
  Eigen::MatrixXd des_body_rpy = robotis_framework::convertQuaternionToRPY(des_body_Q);

  // Right Leg Pose
  Eigen::MatrixXd des_r_foot_pos = Eigen::MatrixXd::Zero(3,1);
  des_r_foot_pos.coeffRef(0,0) = des_r_leg_pos_[0];
  des_r_foot_pos.coeffRef(1,0) = des_r_leg_pos_[1];
  des_r_foot_pos.coeffRef(2,0) = des_r_leg_pos_[2];

  Eigen::Quaterniond des_r_foot_Q(des_r_leg_Q_[3],des_r_leg_Q_[0],des_r_leg_Q_[1],des_r_leg_Q_[2]);
  Eigen::MatrixXd des_r_foot_rot = robotis_framework::convertQuaternionToRotation(des_r_foot_Q);

  // Left Leg Pose
  Eigen::MatrixXd des_l_foot_pos = Eigen::MatrixXd::Zero(3,1);
  des_l_foot_pos.coeffRef(0,0) = des_l_leg_pos_[0];
  des_l_foot_pos.coeffRef(1,0) = des_l_leg_pos_[1];
  des_l_foot_pos.coeffRef(2,0) = des_l_leg_pos_[2];

  Eigen::Quaterniond des_l_foot_Q(des_l_leg_Q_[3],des_l_leg_Q_[0],des_l_leg_Q_[1],des_l_leg_Q_[2]);
  Eigen::MatrixXd des_l_foot_rot = robotis_framework::convertQuaternionToRotation(des_l_foot_Q);

  // Set Desired Value for Balance Control
  Eigen::MatrixXd body_pose = Eigen::MatrixXd::Identity(4,4);
  body_pose.block<3,3>(0,0) = des_body_rot;
  body_pose.block<3,1>(0,3) = des_body_pos;

  Eigen::MatrixXd l_foot_pose = Eigen::MatrixXd::Identity(4,4);
  l_foot_pose.block<3,3>(0,0) = des_l_foot_rot;
  l_foot_pose.block<3,1>(0,3) = des_l_foot_pos;

  Eigen::MatrixXd r_foot_pose = Eigen::MatrixXd::Identity(4,4);
  r_foot_pose.block<3,3>(0,0) = des_r_foot_rot;
  r_foot_pose.block<3,1>(0,3) = des_r_foot_pos;

  //  PRINT_MAT(body_pose);
  //  PRINT_MAT(l_foot_pose);
  //  PRINT_MAT(r_foot_pose);

  // ===== Transformation =====
  Eigen::MatrixXd robot_to_body = Eigen::MatrixXd::Identity(4,4);
  Eigen::MatrixXd robot_to_l_foot = body_pose.inverse() * l_foot_pose;
  Eigen::MatrixXd robot_to_r_foot = body_pose.inverse() * r_foot_pose;

  //  PRINT_MAT(body_pose_trans);
  //  PRINT_MAT(l_foot_pose_trans);
  //  PRINT_MAT(r_foot_pose_trans);
  // =====

  // Set IMU
  imu_data_mutex_lock_.lock();

  balance_control_.setCurrentGyroSensorOutput(imu_data_msg_.angular_velocity.x, imu_data_msg_.angular_velocity.y);

  Eigen::Quaterniond imu_quaternion(imu_data_msg_.orientation.w,
                                    imu_data_msg_.orientation.x,
                                    imu_data_msg_.orientation.y,
                                    imu_data_msg_.orientation.z);
  Eigen::MatrixXd imu_rpy =
      robotis_framework::convertRotationToRPY(robotis_framework::getRotationX(M_PI) * imu_quaternion.toRotationMatrix() * robotis_framework::getRotationZ(M_PI));

  imu_data_mutex_lock_.unlock();

  // Set FT
  //  Eigen::Quaterniond g_to_r_foot_Q(des_r_leg_Q_[3],des_r_leg_Q_[0],des_r_leg_Q_[1],des_r_leg_Q_[2]);
  //  Eigen::Quaterniond g_to_l_foot_Q(des_l_leg_Q_[3],des_l_leg_Q_[0],des_l_leg_Q_[1],des_l_leg_Q_[2]);

  //  Eigen::MatrixXd g_to_r_foot_rot = robotis_framework::convertQuaternionToRotation(g_to_r_foot_Q);
  //  Eigen::MatrixXd g_to_l_foot_rot = robotis_framework::convertQuaternionToRotation(g_to_l_foot_Q);

  //  Eigen::MatrixXd g_to_r_foot_force =
  //      g_to_r_foot_rot * robotis_framework::getRotationX(M_PI) *
  //      robotis_framework::getTransitionXYZ(r_foot_ft_data_msg_.force.x, r_foot_ft_data_msg_.force.y, r_foot_ft_data_msg_.force.z);

  //  Eigen::MatrixXd g_to_r_foot_torque =
  //      g_to_r_foot_rot * robotis_framework::getRotationX(M_PI) *
  //      robotis_framework::getTransitionXYZ(r_foot_ft_data_msg_.torque.x, r_foot_ft_data_msg_.torque.y, r_foot_ft_data_msg_.torque.z);

  //  Eigen::MatrixXd g_to_l_foot_force =
  //      g_to_l_foot_rot * robotis_framework::getRotationX(M_PI) *
  //      robotis_framework::getTransitionXYZ(l_foot_ft_data_msg_.force.x, l_foot_ft_data_msg_.force.y, l_foot_ft_data_msg_.force.z);

  //  Eigen::MatrixXd g_to_l_foot_torque =
  //      g_to_l_foot_rot * robotis_framework::getRotationX(M_PI) *
  //      robotis_framework::getTransitionXYZ(l_foot_ft_data_msg_.torque.x, l_foot_ft_data_msg_.torque.y, l_foot_ft_data_msg_.torque.z);

  Eigen::MatrixXd robot_to_r_foot_force =
      robot_to_r_foot.block(0,0,3,3) * robotis_framework::getRotationX(M_PI) *
      robotis_framework::getTransitionXYZ(r_foot_ft_data_msg_.force.x, r_foot_ft_data_msg_.force.y, r_foot_ft_data_msg_.force.z);

  Eigen::MatrixXd robot_to_r_foot_torque =
      robot_to_r_foot.block(0,0,3,3) * robotis_framework::getRotationX(M_PI) *
      robotis_framework::getTransitionXYZ(r_foot_ft_data_msg_.torque.x, r_foot_ft_data_msg_.torque.y, r_foot_ft_data_msg_.torque.z);

  Eigen::MatrixXd robot_to_l_foot_force =
      robot_to_l_foot.block(0,0,3,3) * robotis_framework::getRotationX(M_PI) *
      robotis_framework::getTransitionXYZ(l_foot_ft_data_msg_.force.x, l_foot_ft_data_msg_.force.y, l_foot_ft_data_msg_.force.z);

  Eigen::MatrixXd robot_to_l_foot_torque =
      robot_to_l_foot.block(0,0,3,3) * robotis_framework::getRotationX(M_PI) *
      robotis_framework::getTransitionXYZ(l_foot_ft_data_msg_.torque.x, l_foot_ft_data_msg_.torque.y, l_foot_ft_data_msg_.torque.z);

  //  Eigen::MatrixXd robot_to_r_foot_force =
  //      robot_to_r_foot.block(0,0,3,3) *
  //      robotis_framework::getTransitionXYZ(r_foot_ft_data_msg_.force.x, r_foot_ft_data_msg_.force.y, r_foot_ft_data_msg_.force.z);

  //  Eigen::MatrixXd robot_to_r_foot_torque =
  //      robot_to_r_foot.block(0,0,3,3) *
  //      robotis_framework::getTransitionXYZ(r_foot_ft_data_msg_.torque.x, r_foot_ft_data_msg_.torque.y, r_foot_ft_data_msg_.torque.z);

  //  Eigen::MatrixXd robot_to_l_foot_force =
  //      robot_to_l_foot.block(0,0,3,3) *
  //      robotis_framework::getTransitionXYZ(l_foot_ft_data_msg_.force.x, l_foot_ft_data_msg_.force.y, l_foot_ft_data_msg_.force.z);

  //  Eigen::MatrixXd robot_to_l_foot_torque =
  //      robot_to_l_foot.block(0,0,3,3) *
  //      robotis_framework::getTransitionXYZ(l_foot_ft_data_msg_.torque.x, l_foot_ft_data_msg_.torque.y, l_foot_ft_data_msg_.torque.z);

  balance_control_.setCurrentOrientationSensorOutput(imu_rpy.coeff(0,0), imu_rpy.coeff(1,0));

  balance_control_.setCurrentFootForceTorqueSensorOutput(robot_to_r_foot_force.coeff(0,0),  robot_to_r_foot_force.coeff(1,0),  robot_to_r_foot_force.coeff(2,0),
                                                         robot_to_r_foot_torque.coeff(0,0), robot_to_r_foot_torque.coeff(1,0), robot_to_r_foot_torque.coeff(2,0),
                                                         robot_to_l_foot_force.coeff(0,0),  robot_to_l_foot_force.coeff(1,0),  robot_to_l_foot_force.coeff(2,0),
                                                         robot_to_l_foot_torque.coeff(0,0), robot_to_l_foot_torque.coeff(1,0), robot_to_l_foot_torque.coeff(2,0));

  //  PRINT_MAT(robot_to_r_foot_force);
  //  PRINT_MAT(robot_to_l_foot_force);

  balance_control_.setDesiredCOBGyro(0.0,0.0);

  //  Eigen::Quaterniond g_to_body_Q(des_body_Q_[3],des_body_Q_[0],des_body_Q_[1],des_body_Q_[2]);
  //  Eigen::MatrixXd g_to_body_rpy = robotis_framework::convertQuaternionToRotation(g_to_body_Q);
  //  balance_control_.setDesiredCOBOrientation(g_to_body_rpy.coeff(0,0),g_to_body_rpy.coeff(1,0));

  balance_control_.setDesiredCOBOrientation(des_body_rpy.coeff(0,0),des_body_rpy.coeff(1,0));

  balance_control_.setDesiredFootForceTorque(balance_r_foot_force_x_, balance_r_foot_force_y_, balance_r_foot_force_z_,
                                             balance_r_foot_torque_x_, balance_r_foot_torque_y_, balance_r_foot_torque_z_,
                                             balance_l_foot_force_x_, balance_l_foot_force_y_, balance_l_foot_force_z_,
                                             balance_l_foot_torque_x_, balance_l_foot_torque_y_, balance_l_foot_torque_z_);

  balance_control_.setDesiredPose(robot_to_body, robot_to_r_foot, robot_to_l_foot);

  int error;
  Eigen::MatrixXd robot_to_body_mod, robot_to_r_foot_mod, robot_to_l_foot_mod;
  balance_control_.process(&error, &robot_to_body_mod, &robot_to_r_foot_mod, &robot_to_l_foot_mod);

  // ===== Transformation =====
  Eigen::MatrixXd body_pose_mod = body_pose * robot_to_body_mod;
  Eigen::MatrixXd r_foot_pose_mod = body_pose * robot_to_r_foot_mod;
  Eigen::MatrixXd l_foot_pose_mod = body_pose * robot_to_l_foot_mod;

  //  PRINT_MAT(body_pose_new);
  //  PRINT_MAT(r_foot_pose_new);
  //  PRINT_MAT(l_foot_pose_new);
  // =====

  Eigen::MatrixXd des_body_rot_mod = body_pose_mod.block<3,3>(0,0);
  Eigen::MatrixXd des_body_pos_mod = body_pose_mod.block<3,1>(0,3);

  Eigen::MatrixXd des_r_foot_rot_mod = r_foot_pose_mod.block<3,3>(0,0);
  Eigen::MatrixXd des_r_foot_pos_mod = r_foot_pose_mod.block<3,1>(0,3);
  Eigen::MatrixXd des_l_foot_rot_mod = l_foot_pose_mod.block<3,3>(0,0);
  Eigen::MatrixXd des_l_foot_pos_mod = l_foot_pose_mod.block<3,1>(0,3);

  // ======= ======= //
  thormang3_kdl_->initialize(des_body_pos_mod, des_body_rot_mod);

  Eigen::VectorXd r_leg_joint_pos, l_leg_joint_pos;

  r_leg_joint_pos.resize(6);
  r_leg_joint_pos(0) = des_joint_pos_[joint_name_to_id_["r_leg_hip_y"]-1];
  r_leg_joint_pos(1) = des_joint_pos_[joint_name_to_id_["r_leg_hip_r"]-1];
  r_leg_joint_pos(2) = des_joint_pos_[joint_name_to_id_["r_leg_hip_p"]-1];
  r_leg_joint_pos(3) = des_joint_pos_[joint_name_to_id_["r_leg_kn_p"]-1];
  r_leg_joint_pos(4) = des_joint_pos_[joint_name_to_id_["r_leg_an_p"]-1];
  r_leg_joint_pos(5) = des_joint_pos_[joint_name_to_id_["r_leg_an_r"]-1];

  l_leg_joint_pos.resize(6);
  l_leg_joint_pos(0) = des_joint_pos_[joint_name_to_id_["l_leg_hip_y"]-1];
  l_leg_joint_pos(1) = des_joint_pos_[joint_name_to_id_["l_leg_hip_r"]-1];
  l_leg_joint_pos(2) = des_joint_pos_[joint_name_to_id_["l_leg_hip_p"]-1];
  l_leg_joint_pos(3) = des_joint_pos_[joint_name_to_id_["l_leg_kn_p"]-1];
  l_leg_joint_pos(4) = des_joint_pos_[joint_name_to_id_["l_leg_an_p"]-1];
  l_leg_joint_pos(5) = des_joint_pos_[joint_name_to_id_["l_leg_an_r"]-1];

  thormang3_kdl_->setJointPosition(r_leg_joint_pos, l_leg_joint_pos);

  //  Eigen::VectorXd rleg_joint_position;
  //  rleg_joint_position.resize(6);

  //  rleg_joint_position(0) = des_joint_pos_[joint_name_to_id_["r_leg_hip_y"]-1];
  //  rleg_joint_position(1) = des_joint_pos_[joint_name_to_id_["r_leg_hip_r"]-1];
  //  rleg_joint_position(2) = des_joint_pos_[joint_name_to_id_["r_leg_hip_p"]-1];
  //  rleg_joint_position(3) = des_joint_pos_[joint_name_to_id_["r_leg_kn_p"]-1];
  //  rleg_joint_position(4) = des_joint_pos_[joint_name_to_id_["r_leg_an_p"]-1];
  //  rleg_joint_position(5) = des_joint_pos_[joint_name_to_id_["r_leg_an_r"]-1];

  //  Eigen::VectorXd lleg_joint_position;
  //  lleg_joint_position.resize(6);

  //  lleg_joint_position(0) = des_joint_pos_[joint_name_to_id_["l_leg_hip_y"]-1];
  //  lleg_joint_position(1) = des_joint_pos_[joint_name_to_id_["l_leg_hip_r"]-1];
  //  lleg_joint_position(2) = des_joint_pos_[joint_name_to_id_["l_leg_hip_p"]-1];
  //  lleg_joint_position(3) = des_joint_pos_[joint_name_to_id_["l_leg_kn_p"]-1];
  //  lleg_joint_position(4) = des_joint_pos_[joint_name_to_id_["l_leg_an_p"]-1];
  //  lleg_joint_position(5) = des_joint_pos_[joint_name_to_id_["l_leg_an_r"]-1];

  //  thormang3_kdl_->setJointPosition(rleg_joint_position, lleg_joint_position);

  std::vector<double_t> r_leg_output, l_leg_output;

  Eigen::Quaterniond des_r_foot_Q_mod = robotis_framework::convertRotationToQuaternion(des_r_foot_rot_mod);
  Eigen::Quaterniond des_l_foot_Q_mod = robotis_framework::convertRotationToQuaternion(des_l_foot_rot_mod);

  ik_success = thormang3_kdl_->solveInverseKinematics(r_leg_output,
                                                      des_r_foot_pos_mod,des_r_foot_Q_mod,
                                                      l_leg_output,
                                                      des_l_foot_pos_mod,des_l_foot_Q_mod);

  thormang3_kdl_->finalize();

  if (ik_success == true)
  {
    des_joint_pos_[joint_name_to_id_["r_leg_hip_y"]-1] = r_leg_output[0];
    des_joint_pos_[joint_name_to_id_["r_leg_hip_r"]-1] = r_leg_output[1];
    des_joint_pos_[joint_name_to_id_["r_leg_hip_p"]-1] = r_leg_output[2];
    des_joint_pos_[joint_name_to_id_["r_leg_kn_p"]-1]  = r_leg_output[3];
    des_joint_pos_[joint_name_to_id_["r_leg_an_p"]-1]  = r_leg_output[4];
    des_joint_pos_[joint_name_to_id_["r_leg_an_r"]-1]  = r_leg_output[5];

    des_joint_pos_[joint_name_to_id_["l_leg_hip_y"]-1] = l_leg_output[0];
    des_joint_pos_[joint_name_to_id_["l_leg_hip_r"]-1] = l_leg_output[1];
    des_joint_pos_[joint_name_to_id_["l_leg_hip_p"]-1] = l_leg_output[2];
    des_joint_pos_[joint_name_to_id_["l_leg_kn_p"]-1]  = l_leg_output[3];
    des_joint_pos_[joint_name_to_id_["l_leg_an_p"]-1]  = l_leg_output[4];
    des_joint_pos_[joint_name_to_id_["l_leg_an_r"]-1]  = l_leg_output[5];
  }

  return ik_success;
//  return true;
}

void WholebodyModule::setFeedbackControl()
{
  for (int i=0; i<number_of_joints_; i++)
  {
    des_joint_pos_to_robot_[i] = des_joint_pos_[i] + des_joint_feedforward_[i];

    joint_feedback_[i].desired_ = des_joint_pos_[i];
    des_joint_feedback_[i] = joint_feedback_[i].getFeedBack(curr_joint_pos_[i]);

    des_joint_pos_to_robot_[i] += des_joint_feedback_[i];
  }
}

void WholebodyModule::setFeedforwardControl()
{
  double cur_time = (double) mov_step_ * control_cycle_sec_;

  std::vector<double_t> feed_forward_value = feed_forward_tra_->getPosition(cur_time);

  if (walking_phase_ == DSP)
    feed_forward_value[0] = 0.0;

  std::vector<double_t> support_leg_gain;
  support_leg_gain.resize(number_of_joints_, 0.0);

  if (walking_leg_ == LEFT_LEG)
  {
    support_leg_gain[joint_name_to_id_["r_leg_hip_y"]-1] = 1.0;
    support_leg_gain[joint_name_to_id_["r_leg_hip_r"]-1] = 1.0;
    support_leg_gain[joint_name_to_id_["r_leg_hip_p"]-1] = 1.0;
    support_leg_gain[joint_name_to_id_["r_leg_kn_p"]-1]  = 1.0;
    support_leg_gain[joint_name_to_id_["r_leg_an_p"]-1]  = 1.0;
    support_leg_gain[joint_name_to_id_["r_leg_an_r"]-1]  = 1.0;

    support_leg_gain[joint_name_to_id_["l_leg_hip_y"]-1] = 0.0;
    support_leg_gain[joint_name_to_id_["l_leg_hip_r"]-1] = 0.0;
    support_leg_gain[joint_name_to_id_["l_leg_hip_p"]-1] = 0.0;
    support_leg_gain[joint_name_to_id_["l_leg_kn_p"]-1]  = 0.0;
    support_leg_gain[joint_name_to_id_["l_leg_an_p"]-1]  = 0.0;
    support_leg_gain[joint_name_to_id_["l_leg_an_r"]-1]  = 0.0;
  }
  else if (walking_leg_ == RIGHT_LEG)
  {
    support_leg_gain[joint_name_to_id_["r_leg_hip_y"]-1] = 0.0;
    support_leg_gain[joint_name_to_id_["r_leg_hip_r"]-1] = 0.0;
    support_leg_gain[joint_name_to_id_["r_leg_hip_p"]-1] = 0.0;
    support_leg_gain[joint_name_to_id_["r_leg_kn_p"]-1]  = 0.0;
    support_leg_gain[joint_name_to_id_["r_leg_an_p"]-1]  = 0.0;
    support_leg_gain[joint_name_to_id_["r_leg_an_r"]-1]  = 0.0;

    support_leg_gain[joint_name_to_id_["l_leg_hip_y"]-1] = 1.0;
    support_leg_gain[joint_name_to_id_["l_leg_hip_r"]-1] = 1.0;
    support_leg_gain[joint_name_to_id_["l_leg_hip_p"]-1] = 1.0;
    support_leg_gain[joint_name_to_id_["l_leg_kn_p"]-1]  = 1.0;
    support_leg_gain[joint_name_to_id_["l_leg_an_p"]-1]  = 1.0;
    support_leg_gain[joint_name_to_id_["l_leg_an_r"]-1]  = 1.0;
  }

  for (int i=0; i<number_of_joints_; i++)
    des_joint_feedforward_[i] = joint_feedforward_gain_[i] * feed_forward_value[0] * support_leg_gain[i];
}

void WholebodyModule::process(std::map<std::string, robotis_framework::Dynamixel *> dxls,
                              std::map<std::string, double> sensors)
{
  if (enable_ == false)
    return;

  /*----- write curr position -----*/
  for (std::map<std::string, robotis_framework::DynamixelState *>::iterator state_iter = result_.begin();
       state_iter != result_.end(); state_iter++)
  {
    std::string joint_name = state_iter->first;

    robotis_framework::Dynamixel *dxl = NULL;
    std::map<std::string, robotis_framework::Dynamixel*>::iterator dxl_it = dxls.find(joint_name);
    if (dxl_it != dxls.end())
      dxl = dxl_it->second;
    else
      continue;

    double curr_joint_pos = dxl->dxl_state_->present_position_;
    double goal_joint_pos = dxl->dxl_state_->goal_position_;

    if (goal_initialize_ == false)
      des_joint_pos_[joint_name_to_id_[joint_name]-1] = goal_joint_pos;

    curr_joint_pos_[joint_name_to_id_[joint_name]-1] = curr_joint_pos;
  }

  goal_initialize_ = true;

  /* Trajectory Calculation */
  ros::Time begin = ros::Time::now();

  if (control_type_ == JOINT_CONTROL)
  {
    initJointControl();
    calcJointControl();
  }
  else if (control_type_ == WHOLEBODY_CONTROL)
  {
    initWholebodyControl();
    calcWholebodyControl();
  }
  else if (control_type_ == WALKING_CONTROL)
  {
    if(walking_initialize_ == true)
    {
      calcWalkingControl();
      setFeedforwardControl();
    }
  }
  else if (control_type_ == OFFSET_CONTROL)
  {
    initOffsetControl();
    calcOffsetControl();
  }

  //  calcRobotPose();

  if (balance_type_ == ON)
  {
    initBalanceControl();
    calcBalanceControl();

    if (setBalanceControl() == false)
    {
      is_moving_ = false;
      is_balancing_ = false;
      is_foot_step_2d_ = false;

      balance_type_ = OFF;
      control_type_ = NONE;

      resetBodyPose();

      ROS_INFO("[FAIL] Task Space Control");
    }
  }

  ros::Duration time_duration = ros::Time::now() - begin;

  if (time_duration.toSec() > 0.003)
    ROS_INFO("[Wholebody Module] Calc Time: %f", time_duration.toSec());

  setFeedbackControl();

  sensor_msgs::JointState goal_joint_msg;

  goal_joint_msg.header.stamp = ros::Time::now();
  /*----- set joint data -----*/
  for (std::map<std::string, robotis_framework::DynamixelState *>::iterator state_iter = result_.begin();
       state_iter != result_.end(); state_iter++)
  {
    std::string joint_name = state_iter->first;
    //    result_[joint_name]->goal_position_ = des_joint_pos_[joint_name_to_id_[joint_name]-1];
    result_[joint_name]->goal_position_ = des_joint_pos_to_robot_[joint_name_to_id_[joint_name]-1];

    goal_joint_msg.name.push_back(joint_name);
    goal_joint_msg.position.push_back(des_joint_pos_[joint_name_to_id_[joint_name]-1]);
  }

  goal_joint_state_pub_.publish(goal_joint_msg);
}

void WholebodyModule::stop()
{
  for (int i=0; i<number_of_joints_; i++)
  {
    des_joint_pos_[i]   = 0.0;
    des_joint_vel_[i]   = 0.0;
    des_joint_accel_[i] = 0.0;
  }

  goal_initialize_ = false;

  is_moving_    = false;
  is_balancing_ = false;

  joint_control_initialize_   = false;
  wholebody_initialize_       = false;
  walking_initialize_         = false;
  balance_control_initialize_ = false;

  control_type_ = NONE;

  return;
}

bool WholebodyModule::isRunning()
{
  return is_moving_;
}

void WholebodyModule::publishStatusMsg(unsigned int type, std::string msg)
{
  robotis_controller_msgs::StatusMsg status;
  status.header.stamp = ros::Time::now();
  status.type         = type;
  status.module_name  = "Wholebody";
  status.status_msg   = msg;

  status_msg_pub_.publish(status);
}

bool WholebodyModule::getJointPoseCallback(thormang3_wholebody_module_msgs::GetJointPose::Request &req,
                                           thormang3_wholebody_module_msgs::GetJointPose::Response &res)
{
  for (int i=0; i<number_of_joints_; i++)
  {
    res.pose.pose.name.push_back(joint_name_[i]);
    res.pose.pose.position.push_back(des_joint_pos_[i]);
  }

  return true;
}

bool WholebodyModule::getKinematicsPoseCallback(thormang3_wholebody_module_msgs::GetKinematicsPose::Request &req,
                                                thormang3_wholebody_module_msgs::GetKinematicsPose::Response &res)
{
  std::string group_name = req.name;

  geometry_msgs::Pose msg;

  if (group_name == "body")
  {
    msg.position.x = des_body_pos_[0];
    msg.position.y = des_body_pos_[1];
    msg.position.z = des_body_pos_[2];

    msg.orientation.x = des_body_Q_[0];
    msg.orientation.y = des_body_Q_[1];
    msg.orientation.z = des_body_Q_[2];
    msg.orientation.w = des_body_Q_[3];
  }
  else if (group_name == "left_leg")
  {
    msg.position.x = des_l_leg_pos_[0];
    msg.position.y = des_l_leg_pos_[1];
    msg.position.z = des_l_leg_pos_[2];

    msg.orientation.x = des_l_leg_Q_[0];
    msg.orientation.y = des_l_leg_Q_[1];
    msg.orientation.z = des_l_leg_Q_[2];
    msg.orientation.w = des_l_leg_Q_[3];
  }
  else if (group_name == "right_leg")
  {
    msg.position.x = des_r_leg_pos_[0];
    msg.position.y = des_r_leg_pos_[1];
    msg.position.z = des_r_leg_pos_[2];

    msg.orientation.x = des_r_leg_Q_[0];
    msg.orientation.y = des_r_leg_Q_[1];
    msg.orientation.z = des_r_leg_Q_[2];
    msg.orientation.w = des_r_leg_Q_[3];
  }

  res.pose.pose = msg;

  return true;
}

bool WholebodyModule::getPreviewMatrix(thormang3_wholebody_module_msgs::PreviewRequest msg)
{
  thormang3_wholebody_module_msgs::GetPreviewMatrix get_preview_matrix;

  // request
  get_preview_matrix.request.req.control_cycle = msg.control_cycle;
  get_preview_matrix.request.req.lipm_height   = msg.lipm_height;

  // response
  if ( get_preview_matrix_client_.call( get_preview_matrix ) )
  {
    preview_response_.K     = get_preview_matrix.response.res.K;
    preview_response_.K_row = get_preview_matrix.response.res.K_row;
    preview_response_.K_col = get_preview_matrix.response.res.K_col;

    preview_response_.P     = get_preview_matrix.response.res.P;
    preview_response_.P_row = get_preview_matrix.response.res.P_row;
    preview_response_.P_col = get_preview_matrix.response.res.P_col;

    return true;
  }
  else
    return false;
}
