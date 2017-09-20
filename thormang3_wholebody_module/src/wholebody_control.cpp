#include <stdio.h>
#include "thormang3_wholebody_module/wholebody_control.h"

WholebodyControl::WholebodyControl(std::string control_group,
                                   double init_time, double fin_time,
                                   std::vector<double_t> init_joint_pos, std::vector<double_t> init_joint_vel, std::vector<double_t> init_joint_accel,
                                   std::vector<double_t> goal_task_pos, std::vector<double_t> goal_task_orien)
{
  control_group_ = control_group;

  init_time_ = init_time;
  fin_time_ = fin_time;

  init_joint_pos_ = init_joint_pos;
  init_joint_vel_ = init_joint_vel;
  init_joint_accel_ = init_joint_accel;

  goal_task_pos_ = goal_task_pos;
  Eigen::Quaterniond quaternion(goal_task_orien[3],goal_task_orien[0],goal_task_orien[1],goal_task_orien[2]);
  goal_task_quaternion_ = quaternion;

  goal_task_vel_.resize(3, 0.0);
  goal_task_accel_.resize(3, 0.0);
  init_task_pos_.resize(3, 0.0);
  init_task_vel_.resize(3, 0.0);
  init_task_accel_.resize(3, 0.0);
  desired_task_pos_.resize(3, 0.0);
  desired_task_vel_.resize(3, 0.0);
  desired_task_accel_.resize(3, 0.0);

  robot_ = new thormang3::KinematicsDynamics(thormang3::WholeBody);
}

WholebodyControl::~WholebodyControl()
{

}

void WholebodyControl::initialize(std::vector<double_t> init_body_pos, std::vector<double_t> init_body_rot)
{
  robot_->thormang3_link_data_[ID_PELVIS_POS_X]->relative_position_.coeffRef(0,0) = init_body_pos[0];
  robot_->thormang3_link_data_[ID_PELVIS_POS_Y]->relative_position_.coeffRef(1,0) = init_body_pos[1];
  robot_->thormang3_link_data_[ID_PELVIS_POS_Z]->relative_position_.coeffRef(2,0) = init_body_pos[2];

  Eigen::Quaterniond init_body_quat(init_body_rot[3],init_body_rot[0],init_body_rot[1],init_body_rot[2]);
  Eigen::MatrixXd init_body_rpy = robotis_framework::convertQuaternionToRPY(init_body_quat);

  robot_->thormang3_link_data_[ID_PELVIS_ROT_X]->joint_angle_  = init_body_rpy.coeff(0,0);
  robot_->thormang3_link_data_[ID_PELVIS_ROT_Y]->joint_angle_ = init_body_rpy.coeff(1,0);
  robot_->thormang3_link_data_[ID_PELVIS_ROT_Z]->joint_angle_   = init_body_rpy.coeff(2,0);

  for (int id=1; id<=MAX_JOINT_ID; id++)
    robot_->thormang3_link_data_[id]->joint_angle_ = init_joint_pos_[id-1];

  robot_->calcForwardKinematics(0);

  if (control_group_ == "body")
    end_link_ = ID_PELVIS;
  else if (control_group_ == "right_leg")
    end_link_ = ID_R_LEG_END;
  else if (control_group_ == "left_leg")
    end_link_ = ID_L_LEG_END;
  else
    end_link_ = ID_BASE;

  Eigen::MatrixXd init_task_pos = robot_->thormang3_link_data_[end_link_]->position_;
  init_task_pos_[0] = init_task_pos.coeff(0,0);
  init_task_pos_[1] = init_task_pos.coeff(1,0);
  init_task_pos_[2] = init_task_pos.coeff(2,0);

  Eigen::MatrixXd rot = robot_->thormang3_link_data_[end_link_]->orientation_;
  init_task_quaternion_ = robotis_framework::convertRotationToQuaternion(rot);

  task_trajectory_ =
      new robotis_framework::MinimumJerk(init_time_, fin_time_,
                                         init_task_pos_, init_task_vel_, init_task_accel_,
                                         goal_task_pos_, goal_task_vel_, goal_task_accel_);

  init_body_pos_ = robot_->thormang3_link_data_[ID_PELVIS]->position_;
  init_body_rot_ = robot_->thormang3_link_data_[ID_PELVIS]->orientation_;

  init_rleg_pos_ = robot_->thormang3_link_data_[ID_R_LEG_END]->position_;
  init_rleg_rot_ = robot_->thormang3_link_data_[ID_R_LEG_END]->orientation_;

  init_lleg_pos_ = robot_->thormang3_link_data_[ID_L_LEG_END]->position_;
  init_lleg_rot_ = robot_->thormang3_link_data_[ID_L_LEG_END]->orientation_;
}

void WholebodyControl::update(std::vector<double_t> init_body_pos, std::vector<double_t> init_body_rot,
                              std::vector<double_t> init_task_pos, std::vector<double_t> init_task_vel, std::vector<double_t> init_task_accel)
{
  robot_->thormang3_link_data_[ID_PELVIS_POS_X]->relative_position_.coeffRef(0,0) = init_body_pos[0];
  robot_->thormang3_link_data_[ID_PELVIS_POS_Y]->relative_position_.coeffRef(1,0) = init_body_pos[1];
  robot_->thormang3_link_data_[ID_PELVIS_POS_Z]->relative_position_.coeffRef(2,0) = init_body_pos[2];

  Eigen::Quaterniond init_body_quat(init_body_rot[3],init_body_rot[0],init_body_rot[1],init_body_rot[2]);
  Eigen::MatrixXd init_body_rpy = robotis_framework::convertQuaternionToRPY(init_body_quat);

  robot_->thormang3_link_data_[ID_PELVIS_ROT_X]->joint_angle_  = init_body_rpy.coeff(0,0);
  robot_->thormang3_link_data_[ID_PELVIS_ROT_Y]->joint_angle_ = init_body_rpy.coeff(1,0);
  robot_->thormang3_link_data_[ID_PELVIS_ROT_Z]->joint_angle_   = init_body_rpy.coeff(2,0);

  for (int id=1; id<=MAX_JOINT_ID; id++)
    robot_->thormang3_link_data_[id]->joint_angle_ = init_joint_pos_[id-1];

  robot_->calcForwardKinematics(0);

  if (control_group_ == "body")
    end_link_ = ID_PELVIS;
  else if (control_group_ == "right_leg")
    end_link_ = ID_R_LEG_END;
  else if (control_group_ == "left_leg")
    end_link_ = ID_L_LEG_END;
  else
    end_link_ = ID_BASE;

  Eigen::MatrixXd rot = robot_->thormang3_link_data_[end_link_]->orientation_;
  init_task_quaternion_ = robotis_framework::convertRotationToQuaternion(rot);

  init_task_pos_ = init_task_pos;
  init_task_vel_ = init_task_vel;
  init_task_accel_ = init_task_accel;

  task_trajectory_ =
      new robotis_framework::MinimumJerk(init_time_, fin_time_,
                                         init_task_pos_, init_task_vel_, init_task_accel_,
                                         goal_task_pos_, goal_task_vel_, goal_task_accel_);

  init_body_pos_ = robot_->thormang3_link_data_[ID_PELVIS]->position_;
  init_body_rot_ = robot_->thormang3_link_data_[ID_PELVIS]->orientation_;

  init_rleg_pos_ = robot_->thormang3_link_data_[ID_R_LEG_END]->position_;
  init_rleg_rot_ = robot_->thormang3_link_data_[ID_R_LEG_END]->orientation_;

  init_lleg_pos_ = robot_->thormang3_link_data_[ID_L_LEG_END]->position_;
  init_lleg_rot_ = robot_->thormang3_link_data_[ID_L_LEG_END]->orientation_;
}

void WholebodyControl::finalize()
{
  delete task_trajectory_;
}

bool WholebodyControl::set(double time)
{
  std::vector<double_t> desired_task_pos = task_trajectory_->getPosition(time);

  Eigen::MatrixXd ik_pos = Eigen::MatrixXd::Zero(3,1);
  for (int i=0; i<3; i++)
    ik_pos.coeffRef(i,0) = desired_task_pos[i];

  double count = time / fin_time_;

  Eigen::Quaterniond ik_quat = init_task_quaternion_.slerp(count, goal_task_quaternion_);
  Eigen::MatrixXd ik_rot = robotis_framework::convertQuaternionToRotation(ik_quat);

  desired_task_quaternion_ = ik_quat;

  int     max_iter    = 30;
  double  ik_tol      = 1e-5;

//  bool ik_success = false;
  bool ik_success = true;

  if (control_group_ == "left_leg" || control_group_ == "right_leg")
  {
//    ik_success = robot_->calcInverseKinematics(ID_PELVIS, end_link_,
//                                               ik_pos, ik_rot,
//                                               max_iter, ik_tol);
  }
  else if (control_group_ == "body")
  {
    robot_->thormang3_link_data_[ID_PELVIS_POS_X]->relative_position_.coeffRef(0,0) = ik_pos.coeff(0,0);
    robot_->thormang3_link_data_[ID_PELVIS_POS_Y]->relative_position_.coeffRef(1,0) = ik_pos.coeff(1,0);
    robot_->thormang3_link_data_[ID_PELVIS_POS_Z]->relative_position_.coeffRef(2,0) = ik_pos.coeff(2,0);

    Eigen::MatrixXd ik_rpy = robotis_framework::convertQuaternionToRPY(ik_quat);

    robot_->thormang3_link_data_[ID_PELVIS_ROT_X]->joint_angle_ = ik_rpy.coeff(0,0);
    robot_->thormang3_link_data_[ID_PELVIS_ROT_Y]->joint_angle_ = ik_rpy.coeff(1,0);
    robot_->thormang3_link_data_[ID_PELVIS_ROT_Z]->joint_angle_ = ik_rpy.coeff(2,0);

//    bool ik_rleg_success = robot_->calcInverseKinematics(ID_PELVIS, ID_R_LEG_END,
//                                                         init_rleg_pos_, init_rleg_rot_,
//                                                         max_iter, ik_tol);

//    bool ik_lleg_success = robot_->calcInverseKinematics(ID_PELVIS, ID_L_LEG_END,
//                                                         init_lleg_pos_, init_lleg_rot_,
//                                                         max_iter, ik_tol);

//    if (ik_rleg_success == true && ik_lleg_success == true)
//      ik_success = true;
//    else
//      ik_success = false;
  }

  return ik_success;
}

std::vector<double_t> WholebodyControl::getJointPosition(double time)
{
  std::vector<double_t> desired_joint_pos;
  desired_joint_pos.resize(MAX_JOINT_ID, 0.0);

  for (int id=1; id<=MAX_JOINT_ID; id++)
    desired_joint_pos[id-1] = robot_->thormang3_link_data_[id]->joint_angle_;

  return desired_joint_pos;
}

std::vector<double_t> WholebodyControl::getJointVelocity(double time)
{

}

std::vector<double_t> WholebodyControl::getJointAcceleration(double time)
{

}

std::vector<double_t> WholebodyControl::getTaskPosition(double time)
{
  desired_task_pos_ = task_trajectory_->getPosition(time);
  return desired_task_pos_;
}

std::vector<double_t> WholebodyControl::getTaskVelocity(double time)
{
  desired_task_vel_ = task_trajectory_->getVelocity(time);
  return desired_task_vel_;
}

std::vector<double_t> WholebodyControl::getTaskAcceleration(double time)
{
  desired_task_accel_ = task_trajectory_->getAcceleration(time);
  return desired_task_accel_;
}

std::vector<double_t> WholebodyControl::getOrientation(double time)
{
  std::vector<double_t> desired_task_orientation;
  desired_task_orientation.resize(4, 0.0);

  desired_task_orientation[0] = desired_task_quaternion_.x();
  desired_task_orientation[1] = desired_task_quaternion_.y();
  desired_task_orientation[2] = desired_task_quaternion_.z();
  desired_task_orientation[3] = desired_task_quaternion_.w();

  return desired_task_orientation;
}

void WholebodyControl::getGroupPose(std::string name, geometry_msgs::Pose *msg)
{
  int end_link;

  if (name == "body")
    end_link = ID_PELVIS;
  else if (name == "right_leg")
    end_link = ID_R_LEG_END;
  else if (name == "left_leg")
    end_link = ID_L_LEG_END;
  else
    end_link = ID_BASE;

  PRINT_MAT(robot_->thormang3_link_data_[ID_R_LEG_END]->position_);
  PRINT_MAT(robot_->thormang3_link_data_[ID_L_LEG_END]->position_);

  msg->position.x = robot_->thormang3_link_data_[end_link]->position_.coeff(0,0);
  msg->position.y = robot_->thormang3_link_data_[end_link]->position_.coeff(1,0);
  msg->position.z = robot_->thormang3_link_data_[end_link]->position_.coeff(2,0);

  Eigen::Quaterniond quat =
      robotis_framework::convertRotationToQuaternion(robot_->thormang3_link_data_[end_link]->orientation_);

  msg->orientation.x = quat.x();
  msg->orientation.y = quat.y();
  msg->orientation.z = quat.z();
  msg->orientation.w = quat.w();
}
