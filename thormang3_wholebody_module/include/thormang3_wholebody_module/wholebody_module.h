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

#ifndef THORMANG3_WHOLEBODY_MODULE_WHOLEBODY_MODULE_H_
#define THORMANG3_WHOLEBODY_MODULE_WHOLEBODY_MODULE_H_

#include <map>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/package.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <boost/thread.hpp>
#include <yaml-cpp/yaml.h>

#include "joint_control.h"
#include "wholebody_control.h"
#include "walking_control.h"

#include "robotis_math/robotis_math.h"
#include "robotis_framework_common/motion_module.h"

#include "robotis_controller_msgs/JointCtrlModule.h"
#include "robotis_controller_msgs/StatusMsg.h"

#include "thormang3_kinematics_dynamics/kinematics_dynamics.h"

#include "thormang3_wholebody_module_msgs/JointPose.h"
#include "thormang3_wholebody_module_msgs/KinematicsPose.h"
#include "thormang3_wholebody_module_msgs/FootStepCommand.h"
#include "thormang3_wholebody_module_msgs/PreviewRequest.h"
#include "thormang3_wholebody_module_msgs/PreviewResponse.h"
#include "thormang3_wholebody_module_msgs/WalkingParam.h"

#include "thormang3_wholebody_module_msgs/GetJointPose.h"
#include "thormang3_wholebody_module_msgs/GetKinematicsPose.h"
#include "thormang3_wholebody_module_msgs/GetPreviewMatrix.h"

namespace thormang3
{

enum CONTROL_TYPE {
  JOINT_CONTROL,
  WHOLEBODY_CONTROL,
  WALKING_CONTROL,
  NONE
};

class WholebodyModule: public robotis_framework::MotionModule,
                       public robotis_framework::Singleton<WholebodyModule>
{
public:
  WholebodyModule();
  virtual ~WholebodyModule();

  /* ROS Topic Callback Functions */

  /* ROS Calculation Functions */

  /* ROS Framework Functions */
  void initialize(const int control_cycle_msec, robotis_framework::Robot *robot);
  void process(std::map<std::string, robotis_framework::Dynamixel *> dxls, std::map<std::string, double> sensors);
  void stop();
  bool isRunning();

  void setResetBodyCallback(const std_msgs::Bool::ConstPtr& msg);
  void goalJointPoseCallback(const thormang3_wholebody_module_msgs::JointPose &msg);
  void goalKinematicsPoseCallback(const thormang3_wholebody_module_msgs::KinematicsPose& msg);
  void footStepCommandCallback(const thormang3_wholebody_module_msgs::FootStepCommand& msg);
  void walkingParamCallback(const thormang3_wholebody_module_msgs::WalkingParam& msg);

  void reset();

  bool getJointPoseCallback(thormang3_wholebody_module_msgs::GetJointPose::Request &req,
                            thormang3_wholebody_module_msgs::GetJointPose::Response &res);
  bool getKinematicsPoseCallback(thormang3_wholebody_module_msgs::GetKinematicsPose::Request &req,
                                 thormang3_wholebody_module_msgs::GetKinematicsPose::Response &res);
  bool getPreviewMatrix(thormang3_wholebody_module_msgs::PreviewRequest msg);

  void publishStatusMsg(unsigned int type, std::string msg);

  /* Parameter */
//  KinematicsDynamics *robotis_;
//  RobotisDevice     *robotis_;
  WholebodyControl  *wholebody_control_;
  WalkingControl    *walking_control_;

private:
  void queueThread();

  void initJointControl();
  void calcJointControl();
  void initWholebodyControl();
  void calcWholebodyControl();
  void initWalkingControl();
  void calcWalkingControl();

  double          control_cycle_sec_;
  boost::thread   queue_thread_;
  boost::mutex    queue_mutex_;

  std_msgs::String movement_done_msg_;

  ros::Publisher  status_msg_pub_;
  ros::Publisher  movement_done_pub_;
  ros::Publisher  goal_joint_state_pub_;

  ros::ServiceClient get_preview_matrix_client_;

  bool is_moving_;
  double mov_time_;
  int mov_size_, mov_step_;

  bool joint_control_initialize_;
  bool wholebody_initialize_;
  bool walking_initialize_;

  int walking_leg_, walking_phase_;
  int walking_size_, walking_step_;

  robotis_framework::MinimumJerk *joint_trajectory_;

  std::map<std::string, int> joint_name_to_id_;

  CONTROL_TYPE control_type_;

  std::string wholegbody_control_group_;

  size_t number_of_joints_;

  std::vector<std::string> joint_name_;

  std::vector<double_t> ff_p_gain_, ff_d_gain_, fb_p_gain_, fb_d_gain_;

  std::vector<double_t> present_joint_torque_, present_joint_acceleration_, present_joint_velocity_, present_joint_position_;
  std::vector<double_t> desired_joint_torque_, desired_joint_acceleration_, desired_joint_velocity_, desired_joint_position_;
  std::vector<double_t> goal_joint_torque_, goal_joint_acceleration_, goal_joint_velocity_, goal_joint_position_;

  std::vector<double_t> goal_task_position_, goal_task_orientation_;
  std::vector<double_t> desired_task_position_, desired_task_velocity_, desired_task_acceleration_, desired_task_orientation_;

  std::vector<double_t> desired_left_foot_position_, desired_left_foot_velocity_, desired_left_foot_acceleration_, desired_left_foot_orientation_;
  std::vector<double_t> desired_right_foot_position_, desired_right_foot_velocity_, desired_right_foot_acceleration_, desired_right_foot_orientation_;
  std::vector<double_t> desired_body_position_, desired_body_velocity_, desired_body_acceleration_, desired_body_orientation_;

  std::vector<double_t> x_lipm_, y_lipm_;

  thormang3_wholebody_module_msgs::FootStepCommand foot_step_command_;
  thormang3_wholebody_module_msgs::PreviewRequest preview_request_;
  thormang3_wholebody_module_msgs::PreviewResponse preview_response_;
  thormang3_wholebody_module_msgs::WalkingParam walking_param_;
};

}

#endif /* THORMANG3_WHOLEBODY_MODULE_WHOLEBODY_MODULE_H_ */
