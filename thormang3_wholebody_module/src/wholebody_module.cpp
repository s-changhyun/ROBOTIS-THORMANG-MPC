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
    balance_control_initialize_(false),
    joint_control_initialize_(false),
    wholebody_initialize_(false),
    walking_initialize_(false)
{
  enable_       = false;
  module_name_  = "wholebody_module";
  control_mode_ = robotis_framework::PositionControl;
  control_type_ = NONE;
  balance_type_ = OFF;

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
  number_of_joints_ = MAX_JOINT_ID;

  present_joint_torque_.resize(number_of_joints_, 0.0);
  present_joint_acceleration_.resize(number_of_joints_, 0.0);
  present_joint_velocity_.resize(number_of_joints_, 0.0);
  present_joint_position_.resize(number_of_joints_, 0.0);

  desired_joint_torque_.resize(number_of_joints_, 0.0);
  desired_joint_acceleration_.resize(number_of_joints_, 0.0);
  desired_joint_velocity_.resize(number_of_joints_, 0.0);
  desired_joint_position_.resize(number_of_joints_, 0.0);

  goal_joint_torque_.resize(number_of_joints_, 0.0);
  goal_joint_acceleration_.resize(number_of_joints_, 0.0);
  goal_joint_velocity_.resize(number_of_joints_, 0.0);
  goal_joint_position_.resize(number_of_joints_, 0.0);

  desired_task_position_.resize(3, 0.0);
  desired_task_velocity_.resize(3, 0.0);
  desired_task_acceleration_.resize(3, 0.0);
  desired_task_orientation_.resize(4, 0.0);

  goal_task_position_.resize(3, 0.0);
  goal_task_orientation_.resize(4, 0.0);

  // body position default
  desired_body_position_.resize(3, 0.0);
  desired_body_velocity_.resize(3, 0.0);
  desired_body_acceleration_.resize(3, 0.0);
  desired_body_orientation_.resize(4, 0.0);

  // left foot position default
  desired_left_foot_position_.resize(3, 0.0);
  desired_left_foot_velocity_.resize(3, 0.0);
  desired_left_foot_acceleration_.resize(3, 0.0);
  desired_left_foot_orientation_.resize(4, 0.0);

  // right foot position default
  desired_right_foot_position_.resize(3, 0.0);
  desired_right_foot_velocity_.resize(3, 0.0);
  desired_right_foot_acceleration_.resize(3, 0.0);
  desired_right_foot_orientation_.resize(4, 0.0);

  x_lipm_.resize(3, 0.0);
  y_lipm_.resize(3, 0.0);

  // walking parameter default
  walking_param_.dsp_ratio = 0.2;
  walking_param_.lipm_height = 0.7;
  walking_param_.foot_height_max = 0.1;
  walking_param_.zmp_offset_x = 0.0; // not applied
  walking_param_.zmp_offset_y = 0.0;

  desired_balance_gain_ratio_.resize(1, 0.0);
  goal_balance_gain_ratio_.resize(1, 0.0);

  reset();
  robotis_ = new KinematicsDynamics(WholeBody);

  balance_control_.initialize(control_cycle_sec_*1000.0);
  balance_control_.setGyroBalanceEnable(false); // Gyro
  balance_control_.setOrientationBalanceEnable(false); // IMU
  balance_control_.setForceTorqueBalanceEnable(false); // FT

  balance_l_foot_force_x_ = 0.0;
  balance_l_foot_force_y_ = 0.0;
  balance_l_foot_force_z_ = 0.0;
  balance_l_foot_torque_x_ = 0.0;
  balance_l_foot_torque_y_ = 0.0;
  balance_l_foot_torque_z_ = 0.0;

  balance_r_foot_force_x_ = 0.0;
  balance_r_foot_force_y_ = 0.0;
  balance_r_foot_force_z_ = 0.0;
  balance_r_foot_torque_x_ = 0.0;
  balance_r_foot_torque_y_ = 0.0;
  balance_r_foot_torque_z_ = 0.0;

  std::string balance_gain_path = ros::package::getPath("thormang3_wholebody_module") + "/config/balance_gain.yaml";
  parseBalanceGainData(balance_gain_path);

  total_mass_ = robotis_->calcTotalMass(0);
  ROS_INFO("total_mass: %f", total_mass_);

  walking_phase_ = DSP;
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

  /* publish topics */

  // for gui
  status_msg_pub_     = ros_node.advertise<robotis_controller_msgs::StatusMsg>("/robotis/status", 1);
  movement_done_pub_  = ros_node.advertise<std_msgs::String>("/robotis/movement_done", 1);
  goal_joint_state_pub_ = ros_node.advertise<sensor_msgs::JointState>("/robotis/goal_joint_state", 1);

  // Client
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

void WholebodyModule::reset()
{
  desired_body_position_[0] = 0.0;
  desired_body_position_[1] = 0.0;
  desired_body_position_[2] = 0.734;

  desired_body_orientation_[0] = 0.0;
  desired_body_orientation_[1] = 0.0;
  desired_body_orientation_[2] = 0.0;
  desired_body_orientation_[3] = 1.0;

  desired_right_foot_position_[0] = 0.0;
  desired_right_foot_position_[1] = -0.093;
  desired_right_foot_position_[2] = 0.0;

  desired_right_foot_orientation_[0] = 0.0;
  desired_right_foot_orientation_[1] = 0.0;
  desired_right_foot_orientation_[2] = 0.0;
  desired_right_foot_orientation_[3] = 1.0;

  desired_left_foot_position_[0] = 0.0;
  desired_left_foot_position_[1] = 0.093;
  desired_left_foot_position_[2] = 0.0;

  desired_left_foot_orientation_[0] = 0.0;
  desired_left_foot_orientation_[1] = 0.0;
  desired_left_foot_orientation_[2] = 0.0;
  desired_left_foot_orientation_[3] = 1.0;

  x_lipm_[0] = 0.0;
  x_lipm_[1] = 0.0;
  x_lipm_[2] = 0.0;

  y_lipm_[0] = 0.0;
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

  foot_roll_gyro_p_gain_ = doc["foot_roll_gyro_p_gain"].as<double>();
  foot_roll_gyro_d_gain_ = doc["foot_roll_gyro_d_gain"].as<double>();

  foot_pitch_gyro_p_gain_ = doc["foot_pitch_gyro_p_gain"].as<double>();
  foot_pitch_gyro_d_gain_ = doc["foot_pitch_gyro_d_gain"].as<double>();

  foot_roll_angle_p_gain_ = doc["foot_roll_angle_p_gain"].as<double>();
  foot_roll_angle_d_gain_ = doc["foot_roll_angle_d_gain"].as<double>();

  foot_pitch_angle_p_gain_ = doc["foot_pitch_angle_p_gain"].as<double>();
  foot_pitch_angle_d_gain_ = doc["foot_pitch_angle_d_gain"].as<double>();

  foot_x_force_p_gain_ = doc["foot_x_force_p_gain"].as<double>();
  foot_x_force_d_gain_ = doc["foot_x_force_d_gain"].as<double>();

  foot_y_force_p_gain_ = doc["foot_y_force_p_gain"].as<double>();
  foot_y_force_d_gain_ = doc["foot_y_force_d_gain"].as<double>();

  foot_z_force_p_gain_ = doc["foot_z_force_p_gain"].as<double>();
  foot_z_force_d_gain_ = doc["foot_z_force_d_gain"].as<double>();

  foot_roll_torque_p_gain_ = doc["foot_roll_torque_p_gain"].as<double>();
  foot_roll_torque_d_gain_ = doc["foot_roll_torque_d_gain"].as<double>();

  foot_pitch_torque_p_gain_ = doc["foot_pitch_torque_p_gain"].as<double>();
  foot_pitch_torque_d_gain_ = doc["foot_pitch_torque_d_gain"].as<double>();

  roll_gyro_cut_off_frequency_ = doc["roll_gyro_cut_off_frequency"].as<double>();
  pitch_gyro_cut_off_frequency_ = doc["pitch_gyro_cut_off_frequency"].as<double>();

  roll_angle_cut_off_frequency_ = doc["roll_angle_cut_off_frequency"].as<double>();
  pitch_angle_cut_off_frequency_ = doc["pitch_angle_cut_off_frequency"].as<double>();

  foot_x_force_cut_off_frequency_ = doc["foot_x_force_cut_off_frequency"].as<double>();
  foot_y_force_cut_off_frequency_ = doc["foot_y_force_cut_off_frequency"].as<double>();
  foot_z_force_cut_off_frequency_ = doc["foot_z_force_cut_off_frequency"].as<double>();
  foot_roll_torque_cut_off_frequency_ = doc["foot_roll_torque_cut_off_frequency"].as<double>();
  foot_pitch_torque_cut_off_frequency_ = doc["foot_pitch_torque_cut_off_frequency"].as<double>();
}

void WholebodyModule::setWholebodyBalanceMsgCallback(const std_msgs::String::ConstPtr& msg)
{
  std::string balance_gain_path = ros::package::getPath("thormang3_wholebody_module") + "/config/balance_gain.yaml";
  parseBalanceGainData(balance_gain_path);

  if (msg->data == "balance_on")
  {
    ROS_INFO("balance on");
    goal_balance_gain_ratio_[0] = 1.0;
  }
  else if(msg->data == "balance_off")
  {
    ROS_INFO("balance off");
    goal_balance_gain_ratio_[0] = 0.0;
  }

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

  balance_trajectory_ =
      new robotis_framework::MinimumJerk(ini_time, mov_time,
                                         desired_balance_gain_ratio_, balance_zero, balance_zero,
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
    desired_balance_gain_ratio_ = balance_trajectory_->getPosition(cur_time);

//    ROS_INFO("desired_balance_gain: %f", desired_balance_gain_[0]);

    if (balance_step_ == balance_size_-1)
    {
      balance_step_ = 0;
      is_balancing_ = false;
      delete balance_trajectory_;

      if (desired_balance_gain_ratio_[0] == 0.0)
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
    reset();
}

void WholebodyModule::walkingParamCallback(const thormang3_wholebody_module_msgs::WalkingParam& msg)
{
  walking_param_ = msg;
}

void WholebodyModule::goalJointPoseCallback(const thormang3_wholebody_module_msgs::JointPose& msg)
{
  size_t joint_size = msg.pose.name.size();

  if (control_type_ == NONE || control_type_ == JOINT_CONTROL)
  {
    mov_time_ = msg.mov_time;

    for (size_t i = 0; i < msg.pose.name.size(); i++)
    {
      std::string joint_name = msg.pose.name[i];
      goal_joint_position_[joint_name_to_id_[joint_name] - 1] = msg.pose.position[i];
    }

    joint_control_initialize_ = false;
    control_type_ = JOINT_CONTROL;
    balance_type_ = OFF;
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

  joint_trajectory_ =
      new robotis_framework::MinimumJerk(ini_time, mov_time,
                                         desired_joint_position_, desired_joint_velocity_, desired_joint_acceleration_,
                                         goal_joint_position_, goal_joint_velocity_, goal_joint_acceleration_);
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

    desired_joint_position_ = joint_trajectory_->getPosition(cur_time);
    desired_joint_velocity_ = joint_trajectory_->getVelocity(cur_time);
    desired_joint_acceleration_ = joint_trajectory_->getAcceleration(cur_time);

    queue_mutex_.unlock();

    if (mov_step_ == mov_size_-1)
    {
      mov_step_ = 0;
      is_moving_ = false;
      delete joint_trajectory_;

      control_type_ = NONE;

      ROS_INFO("[END] Joint Control");
    }
    else
      mov_step_++;
  }
}

void WholebodyModule::goalKinematicsPoseCallback(const thormang3_wholebody_module_msgs::KinematicsPose& msg)
{
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
    wholegbody_control_group_= msg.name;

    goal_task_position_[0] = msg.pose.position.x;
    goal_task_position_[1] = msg.pose.position.y;
    goal_task_position_[2] = msg.pose.position.z;

    goal_task_orientation_[0] = msg.pose.orientation.x;
    goal_task_orientation_[1] = msg.pose.orientation.y;
    goal_task_orientation_[2] = msg.pose.orientation.z;
    goal_task_orientation_[3] = msg.pose.orientation.w;

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
                           desired_joint_position_, desired_joint_velocity_, desired_joint_acceleration_,
                           goal_task_position_, goal_task_orientation_);

  if (is_moving_ == true)
  {
    ROS_INFO("[UPDATE] Wholebody Control");
    wholebody_control_->update(desired_body_position_, desired_body_orientation_,
                               desired_task_position_, desired_task_velocity_, desired_task_acceleration_);
  }
  else
  {
    ROS_INFO("[START] Wholebody Control");

    wholebody_control_->initialize(desired_body_position_, desired_body_orientation_);
    is_moving_ = true;
  }
}

void WholebodyModule::calcWholebodyControl()
{
  if (is_moving_ == true)
  {
    double cur_time = (double) mov_step_ * control_cycle_sec_;
    if (wholebody_control_->set(cur_time) == true)
    {
//      queue_mutex_.lock();

//      desired_joint_position_ = wholebody_control_->getJointPosition(cur_time);

//      queue_mutex_.unlock();

      desired_task_position_ = wholebody_control_->getTaskPosition(cur_time);
      desired_task_velocity_ = wholebody_control_->getTaskVelocity(cur_time);
      desired_task_acceleration_ = wholebody_control_->getTaskAcceleration(cur_time);

      if (wholegbody_control_group_ == "body")
      {
        desired_body_position_ = wholebody_control_->getTaskPosition(cur_time);
        desired_body_orientation_ = wholebody_control_->getOrientation(cur_time);
      }
    }
    else
    {
      is_moving_ = false;
      wholebody_control_->finalize();

      control_type_ = NONE;

      ROS_WARN("[FAIL] Wholebody Control");
    }

    if (mov_step_ == mov_size_-1)
    {
      mov_step_ = 0;
      is_moving_ = false;
      wholebody_control_->finalize();

      control_type_ = NONE;

      walking_param_.zmp_offset_x = desired_body_position_[0];

      ROS_INFO("[END] Wholebody Control");
    }
    else
      mov_step_++;
  }
}

void WholebodyModule::footStepCommandCallback(const thormang3_wholebody_module_msgs::FootStepCommand& msg)
{
  if (balance_type_ == OFF)
  {
    ROS_WARN("[WARN] Balance is off!");
    return;
  }

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
                                        x_lipm_, y_lipm_,
                                        desired_joint_position_, desired_joint_velocity_, desired_joint_acceleration_);

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
      walking_control_->update(walking_leg_, walking_phase_,
                               foot_step_command_,
                               desired_body_position_, desired_body_orientation_,
                               desired_left_foot_position_, desired_left_foot_velocity_, desired_left_foot_acceleration_,
                               desired_right_foot_position_, desired_right_foot_velocity_, desired_right_foot_acceleration_);

      walking_control_->calcPreviewParam(preview_response_);
      ROS_INFO("[UPDATE] Walking Control (%d/%d)", walking_step_+1, walking_size_);
    }
    else
    {
      walking_control_->initialize(foot_step_command_,
                                   desired_body_position_, desired_body_orientation_,
                                   desired_right_foot_position_, desired_right_foot_orientation_,
                                   desired_left_foot_position_, desired_left_foot_orientation_);
      walking_control_->calcPreviewParam(preview_response_);
      is_moving_ = true;

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
    bool calc_result = walking_control_->set(cur_time, walking_step_);

    if(calc_result == true)
    {
//      queue_mutex_.lock();

      // Set joint position
//      desired_joint_position_ = walking_control_->getJointPosition(walking_step_, cur_time);
//      desired_joint_velocity_ = walking_control_->getJointVelocity(walking_step_, cur_time);
//      desired_joint_acceleration_ = walking_control_->getJointAcceleration(walking_step_, cur_time);

//      queue_mutex_.unlock();

      walking_control_->getWalkingPosition(desired_left_foot_position_,
                                           desired_right_foot_position_,
                                           desired_body_position_);
//      walking_control_->getWalkingVelocity(desired_left_foot_velocity_,
//                                           desired_right_foot_velocity_,
//                                           desired_body_velocity_);
//      walking_control_->getWalkingAccleration(desired_left_foot_acceleration_,
//                                              desired_right_foot_acceleration_,
//                                              desired_body_acceleration_);
      walking_control_->getWalkingOrientation(desired_left_foot_orientation_,
                                              desired_right_foot_orientation_,
                                              desired_body_orientation_);

//      ROS_INFO("body x: %f, y: %f, z: %f", desired_body_position_[0], desired_body_position_[1], desired_body_position_[2]);
//      ROS_INFO("lfoot x: %f, y: %f, z: %f", desired_left_foot_position_[0], desired_left_foot_position_[1], desired_left_foot_position_[2]);
//      ROS_INFO("rfoot x: %f, y: %f, z: %f", desired_right_foot_position_[0], desired_right_foot_position_[1], desired_right_foot_position_[2]);

      walking_control_->getLIPM(x_lipm_, y_lipm_);
      walking_control_->getWalkingState(walking_leg_, walking_phase_);
    }
    else
    {
      is_moving_ = false;
      walking_control_->finalize();

      control_type_ = NONE;
      walking_phase_ = DSP;

      ROS_INFO("[FAIL] Walking Control");
    }

    if (mov_step_ == mov_size_-1)
    {
      ROS_INFO("[END] Walking Control (%d/%d)", walking_step_+1, walking_size_);

      mov_step_ = 0;
      walking_control_->next();

      if (walking_step_ == walking_size_-1)
      {
        is_moving_ = false;
        walking_control_->finalize();
        reset();

        control_type_ = NONE;
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

void WholebodyModule::calcGoalFT()
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
  balance_control_.foot_roll_gyro_ctrl_.p_gain_ = foot_roll_gyro_p_gain_ * desired_balance_gain_ratio_[0];
  balance_control_.foot_roll_gyro_ctrl_.d_gain_ = foot_roll_gyro_d_gain_ * desired_balance_gain_ratio_[0];
  balance_control_.foot_pitch_gyro_ctrl_.p_gain_ = foot_pitch_gyro_p_gain_ * desired_balance_gain_ratio_[0];
  balance_control_.foot_pitch_gyro_ctrl_.d_gain_ = foot_pitch_gyro_d_gain_ * desired_balance_gain_ratio_[0];

  //orientation
  balance_control_.foot_roll_angle_ctrl_.p_gain_  = foot_roll_angle_p_gain_ * desired_balance_gain_ratio_[0];
  balance_control_.foot_roll_angle_ctrl_.d_gain_  = foot_roll_angle_d_gain_ * desired_balance_gain_ratio_[0];
  balance_control_.foot_pitch_angle_ctrl_.p_gain_ = foot_pitch_angle_p_gain_ * desired_balance_gain_ratio_[0];
  balance_control_.foot_pitch_angle_ctrl_.d_gain_ = foot_pitch_angle_d_gain_ * desired_balance_gain_ratio_[0];

  //force torque
  balance_control_.right_foot_force_x_ctrl_.p_gain_      = foot_x_force_p_gain_ * desired_balance_gain_ratio_[0];
  balance_control_.right_foot_force_y_ctrl_.p_gain_      = foot_y_force_p_gain_ * desired_balance_gain_ratio_[0];
  balance_control_.right_foot_force_z_ctrl_.p_gain_      = foot_z_force_p_gain_ * desired_balance_gain_ratio_[0];
  balance_control_.right_foot_torque_roll_ctrl_.p_gain_  = foot_roll_torque_p_gain_ * desired_balance_gain_ratio_[0];
  balance_control_.right_foot_torque_pitch_ctrl_.p_gain_ = foot_roll_torque_p_gain_ * desired_balance_gain_ratio_[0];
  balance_control_.right_foot_force_x_ctrl_.d_gain_      = foot_x_force_d_gain_ * desired_balance_gain_ratio_[0];
  balance_control_.right_foot_force_y_ctrl_.d_gain_      = foot_y_force_d_gain_ * desired_balance_gain_ratio_[0];
  balance_control_.right_foot_force_z_ctrl_.d_gain_      = foot_z_force_d_gain_ * desired_balance_gain_ratio_[0];
  balance_control_.right_foot_torque_roll_ctrl_.d_gain_  = foot_roll_torque_d_gain_ * desired_balance_gain_ratio_[0];
  balance_control_.right_foot_torque_pitch_ctrl_.d_gain_ = foot_roll_torque_d_gain_ * desired_balance_gain_ratio_[0];

  balance_control_.left_foot_force_x_ctrl_.p_gain_      = foot_x_force_p_gain_ * desired_balance_gain_ratio_[0];
  balance_control_.left_foot_force_y_ctrl_.p_gain_      = foot_y_force_p_gain_ * desired_balance_gain_ratio_[0];
  balance_control_.left_foot_force_z_ctrl_.p_gain_      = foot_z_force_p_gain_ * desired_balance_gain_ratio_[0];
  balance_control_.left_foot_torque_roll_ctrl_.p_gain_  = foot_roll_torque_p_gain_ * desired_balance_gain_ratio_[0];
  balance_control_.left_foot_torque_pitch_ctrl_.p_gain_ = foot_roll_torque_p_gain_ * desired_balance_gain_ratio_[0];
  balance_control_.left_foot_force_x_ctrl_.d_gain_      = foot_x_force_d_gain_ * desired_balance_gain_ratio_[0];
  balance_control_.left_foot_force_y_ctrl_.d_gain_      = foot_y_force_d_gain_ * desired_balance_gain_ratio_[0];
  balance_control_.left_foot_force_z_ctrl_.d_gain_      = foot_z_force_d_gain_ * desired_balance_gain_ratio_[0];
  balance_control_.left_foot_torque_roll_ctrl_.d_gain_  = foot_roll_torque_d_gain_ * desired_balance_gain_ratio_[0];
  balance_control_.left_foot_torque_pitch_ctrl_.d_gain_ = foot_roll_torque_d_gain_ * desired_balance_gain_ratio_[0];

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

//  double gain_ratio;
//  double max_pelvis = 0.734;
//  double min_pelvis = 0.3;

//  if (robotis_->thormang3_link_data_[ID_PELVIS]->position_.coeff(2,0) > max_pelvis)
//    gain_ratio = 1.0;
//  else if (robotis_->thormang3_link_data_[ID_PELVIS]->position_.coeff(2,0) < min_pelvis)
//    gain_ratio = 0.0;
//  else
//    gain_ratio = (robotis_->thormang3_link_data_[ID_PELVIS]->position_.coeff(2,0) - min_pelvis) / (max_pelvis - min_pelvis);

  //  double sim_constant;
  //  if (gazebo_ == true)
  //    sim_constant = 0.0;
  //  else
  //    sim_constant = 1.0;

//  double gyro_gain = gyro_gain_ * gain_ratio;

//  balance_control_.setGyroBalanceGainRatio(gyro_gain);

//  balance_control_.foot_roll_angle_ctrl_.gain_ = foot_roll_angle_gain_ * gain_ratio;
//  balance_control_.foot_pitch_angle_ctrl_.gain_ = foot_pitch_angle_gain_ * gain_ratio;

//  balance_control_.left_foot_force_x_ctrl_.gain_ = left_foot_force_x_gain_ * gain_ratio;
//  balance_control_.left_foot_force_y_ctrl_.gain_ = left_foot_force_y_gain_ * gain_ratio;

//  balance_control_.right_foot_force_x_ctrl_.gain_ = right_foot_force_x_gain_ * gain_ratio;
//  balance_control_.right_foot_force_y_ctrl_.gain_ = right_foot_force_y_gain_ * gain_ratio;

//  balance_control_.foot_force_z_diff_ctrl_.gain_ = foot_force_z_gain_ * gain_ratio;

//  balance_control_.right_foot_torque_roll_ctrl_.gain_ = left_foot_torque_roll_gain_ * gain_ratio;
//  balance_control_.right_foot_torque_pitch_ctrl_.gain_ = left_foot_torque_pitch_gain_ * gain_ratio;

//  balance_control_.left_foot_torque_roll_ctrl_.gain_ = right_foot_torque_roll_gain_ * gain_ratio;
//  balance_control_.left_foot_torque_pitch_ctrl_.gain_ = right_foot_torque_pitch_gain_ * gain_ratio;

//  balance_control_.foot_roll_angle_ctrl_.time_constant_sec_ = foot_roll_angle_time_constant_;
//  balance_control_.foot_pitch_angle_ctrl_.time_constant_sec_ = foot_pitch_angle_time_constant_;

//  balance_control_.left_foot_force_x_ctrl_.time_constant_sec_ = left_foot_force_x_time_constant_;
//  balance_control_.left_foot_force_y_ctrl_.time_constant_sec_ = left_foot_force_y_time_constant_;

//  balance_control_.right_foot_force_x_ctrl_.time_constant_sec_ = right_foot_force_x_time_constant_;
//  balance_control_.right_foot_force_y_ctrl_.time_constant_sec_ = right_foot_force_y_time_constant_;

//  balance_control_.foot_force_z_diff_ctrl_.time_constant_sec_ = foot_force_z_time_constant_;

//  balance_control_.right_foot_torque_roll_ctrl_.time_constant_sec_ = left_foot_torque_roll_time_constant_;
//  balance_control_.right_foot_torque_pitch_ctrl_.time_constant_sec_ = left_foot_torque_pitch_time_constant_;

//  balance_control_.left_foot_torque_roll_ctrl_.time_constant_sec_ = right_foot_torque_roll_time_constant_;
//  balance_control_.left_foot_torque_pitch_ctrl_.time_constant_sec_ = right_foot_torque_pitch_time_constant_;

//  balance_control_.setGyroBalanceGainRatio(gyro_gain * desired_balance_gain_[0]);

//  balance_control_.foot_roll_angle_ctrl_.gain_ *= desired_balance_gain_[0];
//  balance_control_.foot_pitch_angle_ctrl_.gain_ *= desired_balance_gain_[0];

//  balance_control_.left_foot_force_x_ctrl_.gain_ *= desired_balance_gain_[0];
//  balance_control_.left_foot_force_y_ctrl_.gain_ *= desired_balance_gain_[0];

//  balance_control_.right_foot_force_x_ctrl_.gain_ *= desired_balance_gain_[0];
//  balance_control_.right_foot_force_y_ctrl_.gain_ *= desired_balance_gain_[0];

//  balance_control_.foot_force_z_diff_ctrl_.gain_ *= desired_balance_gain_[0];

//  balance_control_.right_foot_torque_roll_ctrl_.gain_ *= desired_balance_gain_[0];
//  balance_control_.right_foot_torque_pitch_ctrl_.gain_ *= desired_balance_gain_[0];

//  balance_control_.left_foot_torque_roll_ctrl_.gain_ *= desired_balance_gain_[0];
//  balance_control_.left_foot_torque_pitch_ctrl_.gain_ *= desired_balance_gain_[0];
}

bool WholebodyModule::set()
{
//  ROS_INFO("Gain Ratio: %f", desired_balance_gain_ratio_[0]);

  // Set Balance Control
  balance_control_.setGyroBalanceEnable(true);
  balance_control_.setOrientationBalanceEnable(true);
  balance_control_.setForceTorqueBalanceEnable(true);

  setBalanceControlGain();
  calcGoalFT();

  // Set Inverse Kinematics
  int     max_iter    = 30;
  double  ik_tol      = 1e-5;

  bool ik_success = false;

  // BODY
  Eigen::MatrixXd desired_body_pos = Eigen::MatrixXd::Zero(3,1);
  desired_body_pos.coeffRef(0,0) = desired_body_position_[0];
  desired_body_pos.coeffRef(1,0) = desired_body_position_[1];
  desired_body_pos.coeffRef(2,0) = desired_body_position_[2];

  Eigen::Quaterniond desired_body_quaternion(desired_body_orientation_[3],desired_body_orientation_[0],
                                             desired_body_orientation_[1],desired_body_orientation_[2]);

  Eigen::MatrixXd desired_body_rot = robotis_framework::convertQuaternionToRotation(desired_body_quaternion);

//  PRINT_MAT(robotis_->thormang3_link_data_[ID_PELVIS]->position_);

  // Right Leg
  Eigen::MatrixXd desired_right_foot_pos = Eigen::MatrixXd::Zero(3,1);
  desired_right_foot_pos.coeffRef(0,0) = desired_right_foot_position_[0];
  desired_right_foot_pos.coeffRef(1,0) = desired_right_foot_position_[1];
  desired_right_foot_pos.coeffRef(2,0) = desired_right_foot_position_[2];

//  PRINT_MAT(desired_right_foot_pos);

  Eigen::Quaterniond desired_right_foot_quaternion(desired_right_foot_orientation_[3],desired_right_foot_orientation_[0],
                                                   desired_right_foot_orientation_[1],desired_right_foot_orientation_[2]);

  Eigen::MatrixXd desired_right_foot_rot = robotis_framework::convertQuaternionToRotation(desired_right_foot_quaternion);

//  Eigen::MatrixXd desired_right_foot_rpy = robotis_framework::convertQuaternionToRPY(desired_right_foot_quaternion);
//  ROS_INFO("desired_right_foot_z: %f", desired_right_foot_rpy.coeff(2,0));

//  bool ik_rleg_success = false;
//  ik_rleg_success = robotis_->calcInverseKinematics(ID_PELVIS, ID_R_LEG_END,
//                                                    desired_right_foot_pos, desired_right_foot_rot,
//                                                    max_iter, ik_tol);

  // Left Leg
  Eigen::MatrixXd desired_left_foot_pos = Eigen::MatrixXd::Zero(3,1);
  desired_left_foot_pos.coeffRef(0,0) = desired_left_foot_position_[0];
  desired_left_foot_pos.coeffRef(1,0) = desired_left_foot_position_[1];
  desired_left_foot_pos.coeffRef(2,0) = desired_left_foot_position_[2];

//  PRINT_MAT(desired_left_foot_pos);

  Eigen::Quaterniond desired_left_foot_quaternion(desired_left_foot_orientation_[3],desired_left_foot_orientation_[0],
                                                  desired_left_foot_orientation_[1],desired_left_foot_orientation_[2]);

  Eigen::MatrixXd desired_left_foot_rot = robotis_framework::convertQuaternionToRotation(desired_left_foot_quaternion);

//  Eigen::MatrixXd desired_left_foot_rpy = robotis_framework::convertQuaternionToRPY(desired_left_foot_quaternion);
//  ROS_INFO("desired_left_foot_z: %f", desired_left_foot_rpy.coeff(2,0));

//  bool ik_lleg_success = false;
//  ik_lleg_success = robotis_->calcInverseKinematics(ID_PELVIS, ID_L_LEG_END,
//                                                    desired_left_foot_pos, desired_left_foot_rot,
//                                                    max_iter, ik_tol);

//  PRINT_MAT(desired_right_foot_pos);
//  PRINT_MAT(desired_right_foot_rot);
//  PRINT_MAT(desired_left_foot_pos);
//  PRINT_MAT(desired_left_foot_rot);

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

  Eigen::MatrixXd g_to_r_foot_force =
      robotis_->thormang3_link_data_[ID_R_LEG_FT]->orientation_ * robotis_framework::getRotationX(M_PI) *
      robotis_framework::getTransitionXYZ(r_foot_ft_data_msg_.force.x, r_foot_ft_data_msg_.force.y, r_foot_ft_data_msg_.force.z);

  Eigen::MatrixXd g_to_r_foot_torque =
      robotis_->thormang3_link_data_[ID_R_LEG_FT]->orientation_ * robotis_framework::getRotationX(M_PI) *
      robotis_framework::getTransitionXYZ(r_foot_ft_data_msg_.torque.x, r_foot_ft_data_msg_.torque.y, r_foot_ft_data_msg_.torque.z);

  Eigen::MatrixXd g_to_l_foot_force =
      robotis_->thormang3_link_data_[ID_L_LEG_FT]->orientation_ * robotis_framework::getRotationX(M_PI) *
      robotis_framework::getTransitionXYZ(l_foot_ft_data_msg_.force.x, l_foot_ft_data_msg_.force.y, l_foot_ft_data_msg_.force.z);

  Eigen::MatrixXd g_to_l_foot_torque =
      robotis_->thormang3_link_data_[ID_L_LEG_FT]->orientation_ * robotis_framework::getRotationX(M_PI) *
      robotis_framework::getTransitionXYZ(l_foot_ft_data_msg_.torque.x, l_foot_ft_data_msg_.torque.y, l_foot_ft_data_msg_.torque.z);

  balance_control_.setCurrentOrientationSensorOutput(imu_rpy.coeff(0,0), imu_rpy.coeff(1,0));
  balance_control_.setCurrentFootForceTorqueSensorOutput(g_to_r_foot_force.coeff(0,0),  g_to_r_foot_force.coeff(1,0),  g_to_r_foot_force.coeff(2,0),
                                                         g_to_r_foot_torque.coeff(0,0), g_to_r_foot_torque.coeff(1,0), g_to_r_foot_torque.coeff(2,0),
                                                         g_to_l_foot_force.coeff(0,0),  g_to_l_foot_force.coeff(1,0),  g_to_l_foot_force.coeff(2,0),
                                                         g_to_l_foot_torque.coeff(0,0), g_to_l_foot_torque.coeff(1,0), g_to_l_foot_torque.coeff(2,0));

//  ROS_INFO("g_to_r_foot_force x: %f, y: %f, z: %f", g_to_r_foot_force.coeff(0,0), g_to_r_foot_force.coeff(1,0), g_to_r_foot_force.coeff(2,0));
//  ROS_INFO("g_to_l_foot_force x: %f, y: %f, z: %f", g_to_l_foot_force.coeff(0,0), g_to_l_foot_force.coeff(1,0), g_to_l_foot_force.coeff(2,0));

  balance_control_.setDesiredCOBGyro(0.0, 0.0);
  balance_control_.setDesiredCOBOrientation(robotis_->thormang3_link_data_[ID_PELVIS_ROT_X]->joint_angle_,
                                            robotis_->thormang3_link_data_[ID_PELVIS_ROT_Y]->joint_angle_);

//  ROS_INFO("r_foot_force x: %f, y: %f, z: %f", balance_r_foot_force_x_, balance_r_foot_force_y_, balance_r_foot_force_z_);
//  ROS_INFO("l_foot_force x: %f, y: %f, z: %f", balance_l_foot_force_x_, balance_l_foot_force_y_, balance_l_foot_force_z_);

  balance_control_.setDesiredFootForceTorque(balance_r_foot_force_x_, balance_r_foot_force_y_, balance_r_foot_force_z_,
                                             balance_r_foot_torque_x_, balance_r_foot_torque_y_, balance_r_foot_torque_z_,
                                             balance_l_foot_force_x_, balance_l_foot_force_y_, balance_l_foot_force_z_,
                                             balance_l_foot_torque_x_, balance_l_foot_torque_y_, balance_l_foot_torque_z_);

  // Set Desired Value for Balance Control
  Eigen::MatrixXd body_pose = Eigen::MatrixXd::Identity(4,4);
  body_pose.block<3,3>(0,0) = desired_body_rot;
  body_pose.block<3,1>(0,3) = desired_body_pos;

  Eigen::MatrixXd l_foot_pose = Eigen::MatrixXd::Identity(4,4);
  l_foot_pose.block<3,3>(0,0) = desired_left_foot_rot;
  l_foot_pose.block<3,1>(0,3) = desired_left_foot_pos;

  Eigen::MatrixXd r_foot_pose = Eigen::MatrixXd::Identity(4,4);
  r_foot_pose.block<3,3>(0,0) = desired_right_foot_rot;
  r_foot_pose.block<3,1>(0,3) = desired_right_foot_pos;

  balance_control_.setDesiredPose(body_pose, r_foot_pose, l_foot_pose);

  int error;
  Eigen::MatrixXd body_pose_new, r_foot_pose_new, l_foot_pose_new;
  balance_control_.process(&error, &body_pose_new, &r_foot_pose_new, &l_foot_pose_new);

  Eigen::MatrixXd desired_body_rot_new = body_pose_new.block<3,3>(0,0);
  Eigen::MatrixXd desired_body_pos_new = body_pose_new.block<3,1>(0,3);

  Eigen::MatrixXd desired_right_foot_rot_new = r_foot_pose_new.block<3,3>(0,0);
  Eigen::MatrixXd desired_right_foot_pos_new = r_foot_pose_new.block<3,1>(0,3);
  Eigen::MatrixXd desired_left_foot_rot_new = l_foot_pose_new.block<3,3>(0,0);
  Eigen::MatrixXd desired_left_foot_pos_new = l_foot_pose_new.block<3,1>(0,3);

//  ROS_INFO("--");

//  ROS_INFO("desired_body_pos_new x: %f, y: %f, z: %f", desired_body_pos_new.coeff(0,0), desired_body_pos_new.coeff(1,0), desired_body_pos_new.coeff(2,0));
//  ROS_INFO("desired_body_pos     x: %f, y: %f, z: %f", desired_body_pos.coeff(0,0), desired_body_pos.coeff(1,0), desired_body_pos.coeff(2,0));

//  Eigen::MatrixXd body_rpy_new = robotis_framework::convertRotationToRPY(desired_body_rot_new);
//  Eigen::MatrixXd body_rpy = robotis_framework::convertRotationToRPY(desired_body_rot);
//  ROS_INFO("body_rpy_new r: %f, p: %f, y: %f", body_rpy_new.coeff(0,0), body_rpy_new.coeff(1,0), body_rpy_new.coeff(2,0));
//  ROS_INFO("body_rpy r: %f, p: %f, y: %f", body_rpy.coeff(0,0), body_rpy.coeff(1,0), body_rpy.coeff(2,0));

//  ROS_INFO("desired_right_foot_pos_new x: %f, y: %f, z: %f", desired_right_foot_pos_new.coeff(0,0), desired_right_foot_pos_new.coeff(1,0), desired_right_foot_pos_new.coeff(2,0));
//  ROS_INFO("desired_right_foot_pos     x: %f, y: %f, z: %f", desired_right_foot_pos.coeff(0,0), desired_right_foot_pos.coeff(1,0), desired_right_foot_pos.coeff(2,0));

//  ROS_INFO("desired_left_foot_pos_new x: %f, y: %f, z: %f", desired_left_foot_pos_new.coeff(0,0), desired_left_foot_pos_new.coeff(1,0), desired_left_foot_pos_new.coeff(2,0));
//  ROS_INFO("desired_left_foot_pos     x: %f, y: %f, z: %f", desired_left_foot_pos.coeff(0,0), desired_left_foot_pos.coeff(1,0), desired_left_foot_pos.coeff(2,0));

//  Eigen::MatrixXd left_foot_rpy_new = robotis_framework::convertRotationToRPY(desired_left_foot_rot_new);
//  Eigen::MatrixXd left_foot_rpy = robotis_framework::convertRotationToRPY(desired_left_foot_rot);
//  ROS_INFO("left_foot_rpy_new r: %f, p: %f, y: %f", left_foot_rpy_new.coeff(0,0), left_foot_rpy_new.coeff(1,0), left_foot_rpy_new.coeff(2,0));
//  ROS_INFO("left_foot_rpy r: %f, p: %f, y: %f", left_foot_rpy.coeff(0,0), left_foot_rpy.coeff(1,0), left_foot_rpy.coeff(2,0));

  // Set Body Pose
  robotis_->thormang3_link_data_[ID_PELVIS_POS_X]->relative_position_.coeffRef(0,0) = desired_body_pos_new.coeff(0,0);
  robotis_->thormang3_link_data_[ID_PELVIS_POS_Y]->relative_position_.coeffRef(1,0) = desired_body_pos_new.coeff(1,0);
  robotis_->thormang3_link_data_[ID_PELVIS_POS_Z]->relative_position_.coeffRef(2,0) = desired_body_pos_new.coeff(2,0);
//  robotis_->thormang3_link_data_[ID_PELVIS_POS_X]->relative_position_.coeffRef(0,0) = desired_body_pos.coeff(0,0);
//  robotis_->thormang3_link_data_[ID_PELVIS_POS_Y]->relative_position_.coeffRef(1,0) = desired_body_pos.coeff(1,0);
//  robotis_->thormang3_link_data_[ID_PELVIS_POS_Z]->relative_position_.coeffRef(2,0) = desired_body_pos.coeff(2,0);

  Eigen::MatrixXd desired_body_rpy_new = robotis_framework::convertRotationToRPY(desired_body_rot_new);
  robotis_->thormang3_link_data_[ID_PELVIS_ROT_X]->joint_angle_ = desired_body_rpy_new.coeff(0,0);
  robotis_->thormang3_link_data_[ID_PELVIS_ROT_Y]->joint_angle_ = desired_body_rpy_new.coeff(1,0);
  robotis_->thormang3_link_data_[ID_PELVIS_ROT_Z]->joint_angle_ = desired_body_rpy_new.coeff(2,0);
//  Eigen::MatrixXd desired_body_rpy = robotis_framework::convertRotationToRPY(desired_body_rot);
//  robotis_->thormang3_link_data_[ID_PELVIS_ROT_X]->joint_angle_ = desired_body_rpy.coeff(0,0);
//  robotis_->thormang3_link_data_[ID_PELVIS_ROT_Y]->joint_angle_ = desired_body_rpy.coeff(1,0);
//  robotis_->thormang3_link_data_[ID_PELVIS_ROT_Z]->joint_angle_ = desired_body_rpy.coeff(2,0);

  // Forward Kinematics
  for (int id=1; id<=MAX_JOINT_ID; id++)
    robotis_->thormang3_link_data_[id]->joint_angle_ = desired_joint_position_[id-1];

  robotis_->calcForwardKinematics(0);

  // Inverse Kinematics
  ik_success = robotis_->calcInverseKinematicsDual(ID_PELVIS, ID_R_LEG_END, desired_right_foot_pos_new, desired_right_foot_rot_new,
                                                   ID_PELVIS, ID_L_LEG_END, desired_left_foot_pos_new, desired_left_foot_rot_new,
                                                   max_iter, ik_tol);
//  ik_success = robotis_->calcInverseKinematicsDual(ID_PELVIS, ID_R_LEG_END, desired_right_foot_pos, desired_right_foot_rot,
//                                                   ID_PELVIS, ID_L_LEG_END, desired_left_foot_pos, desired_left_foot_rot,
//                                                   max_iter, ik_tol);

  if (ik_success == true)
  {
    for (int id=1; id<=MAX_JOINT_ID; id++)
      desired_joint_position_[id-1] = robotis_->thormang3_link_data_[id]->joint_angle_;
  }

  return ik_success;
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

    double joint_curr_position = dxl->dxl_state_->present_position_;
    double joint_goal_position = dxl->dxl_state_->goal_position_;

    desired_joint_position_[joint_name_to_id_[joint_name]-1] = joint_goal_position;
  }

  /* Trajectory Calculation */
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
//    ros::Time begin = ros::Time::now();

    if(walking_initialize_ == true)
      calcWalkingControl();

//    ros::Duration time_duration = ros::Time::now() - begin;
//    ROS_INFO("calc time: %f", time_duration.toSec());
  }

  ros::Time begin = ros::Time::now();

  if (balance_type_ == ON)
  {
    initBalanceControl();
    calcBalanceControl();

    if (set() == false)
    {
      is_moving_ = false;
      is_balancing_ = false;

      balance_type_ = OFF;
      control_type_ = NONE;

      reset();
      robotis_ = new KinematicsDynamics(WholeBody);

      ROS_INFO("[FAIL] Task Space Control");
    }
  }

  ros::Duration time_duration = ros::Time::now() - begin;

  if (time_duration.toSec() > 0.004)
    ROS_INFO("calc time: %f", time_duration.toSec());


//  ROS_INFO("Balance Gain: %f", desired_balance_gain_[0]);

  /*----- set joint data -----*/
  for (std::map<std::string, robotis_framework::DynamixelState *>::iterator state_iter = result_.begin();
      state_iter != result_.end(); state_iter++)
  {
    std::string joint_name = state_iter->first;
    result_[joint_name]->goal_position_ = desired_joint_position_[joint_name_to_id_[joint_name]-1];
  }
}

void WholebodyModule::stop()
{
  for (int i=0; i<number_of_joints_; i++)
  {
    desired_joint_torque_[i] = 0.0;
    desired_joint_acceleration_[i] = 0.0;
    desired_joint_velocity_[i] = 0.0;
    desired_joint_position_[i] = 0.0;
  }

  is_moving_ = false;
  is_balancing_ = false;

  joint_control_initialize_ = false;
  wholebody_initialize_ = false;
  walking_initialize_ = false;
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
  for (int i=0; i<MAX_JOINT_ID; i++)
  {
    res.pose.pose.name.push_back(joint_name_[i]);
    res.pose.pose.position.push_back(desired_joint_position_[i]);
  }

  return true;
}

bool WholebodyModule::getKinematicsPoseCallback(thormang3_wholebody_module_msgs::GetKinematicsPose::Request &req,
                                           thormang3_wholebody_module_msgs::GetKinematicsPose::Response &res)
{
  std::string group_name = req.name;

  geometry_msgs::Pose msg;
  wholebody_control_->getGroupPose(group_name, &msg);
  res.pose.pose = msg;

  return true;
}

bool WholebodyModule::getPreviewMatrix(thormang3_wholebody_module_msgs::PreviewRequest msg)
{
  thormang3_wholebody_module_msgs::GetPreviewMatrix get_preview_matrix;

  // request
  get_preview_matrix.request.req.control_cycle = msg.control_cycle;
  get_preview_matrix.request.req.lipm_height = msg.lipm_height;

  // response
  if ( get_preview_matrix_client_.call( get_preview_matrix ) )
  {
    preview_response_.K = get_preview_matrix.response.res.K;
    preview_response_.K_row = get_preview_matrix.response.res.K_row;
    preview_response_.K_col = get_preview_matrix.response.res.K_col;

    preview_response_.P = get_preview_matrix.response.res.P;
    preview_response_.P_row = get_preview_matrix.response.res.P_row;
    preview_response_.P_col = get_preview_matrix.response.res.P_col;

    return true;
  }
  else
    return false;
}
