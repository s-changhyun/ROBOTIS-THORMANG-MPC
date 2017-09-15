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
    joint_control_initialize_(false),
    wholebody_initialize_(false),
    walking_initialize_(false)
{
  enable_       = false;
  module_name_  = "wholebody_module";
  control_mode_ = robotis_framework::PositionControl;
  control_type_ = NONE;

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
  desired_body_position_[0] = 0.0;
  desired_body_position_[2] = 0.727;
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
  walking_param_.lipm_height = 0.6;
  walking_param_.foot_height_max = 0.1;
  walking_param_.zmp_offset_x = 0.0; // not applied
  walking_param_.zmp_offset_y = 0.0;
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
  desired_body_position_[2] = 0.727;

  desired_body_orientation_[0] = 0.0;
  desired_body_orientation_[1] = 0.0;
  desired_body_orientation_[2] = 0.0;
  desired_body_orientation_[3] = 1.0;

  x_lipm_[0] = 0.0;
  x_lipm_[1] = 0.0;
  x_lipm_[2] = 0.0;

  y_lipm_[0] = 0.0;
  y_lipm_[1] = 0.0;
  y_lipm_[2] = 0.0;
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
      queue_mutex_.lock();

      desired_joint_position_ = wholebody_control_->getJointPosition(cur_time);

      queue_mutex_.unlock();

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

      ROS_INFO("[END] Wholebody Control");
    }
    else
      mov_step_++;
  }
}

void WholebodyModule::footStepCommandCallback(const thormang3_wholebody_module_msgs::FootStepCommand& msg)
{
  if (control_type_ == NONE || control_type_ == WALKING_CONTROL)
  {
    walking_size_ = msg.step_num + 2;
    mov_time_ = msg.step_time;

    foot_step_command_ = msg;
    foot_step_command_.step_num = walking_size_;

    walking_initialize_ = false;
    control_type_ = WALKING_CONTROL;
  }
  else
    ROS_WARN("[WARN] Control type is different!");
}

void WholebodyModule::initWalkingControl()
{
  if (walking_initialize_ == true)
    return;

  walking_initialize_ = true;

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
      walking_control_->initialize(foot_step_command_, desired_body_position_, desired_body_orientation_);
      walking_control_->calcPreviewParam(preview_response_);
      is_moving_ = true;

      ROS_INFO("[START] Walking Control (%d/%d)", walking_step_+1, walking_size_);
    }
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
      queue_mutex_.lock();

      // Set joint position
      desired_joint_position_ = walking_control_->getJointPosition(walking_step_, cur_time);
      desired_joint_velocity_ = walking_control_->getJointVelocity(walking_step_, cur_time);
      desired_joint_acceleration_ = walking_control_->getJointAcceleration(walking_step_, cur_time);

      queue_mutex_.unlock();

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
//        reset();

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
    ros::Time begin = ros::Time::now();

    initWalkingControl();
    calcWalkingControl();

    ros::Duration time_duration = ros::Time::now() - begin;
    ROS_INFO("calc time: %f", time_duration.toSec());
  }

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

  joint_control_initialize_ = false;
  wholebody_initialize_ = false;
  walking_initialize_ = false;

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
