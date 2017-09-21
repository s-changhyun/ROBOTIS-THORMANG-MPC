#include <stdio.h>
#include "thormang3_wholebody_module/walking_control.h"

WalkingControl::WalkingControl(double control_cycle,
                               double dsp_ratio, double lipm_height, double foot_height_max, double zmp_offset_x, double zmp_offset_y,
                               std::vector<double_t> x_lipm, std::vector<double_t> y_lipm,
                               std::vector<double_t> init_joint_pos, std::vector<double_t> init_joint_vel, std::vector<double_t> init_joint_accel)
  : walking_update_(false)
{
  control_cycle_ = control_cycle;

  // Trajectory
  init_time_ = 0.0;
  fin_time_ = 0.0;

  // Foot Paramater
  foot_step_size_ = 0;

  foot_size_x_ = 0.216;
  foot_size_y_ = 0.144;
  foot_origin_shift_x_ = 0.0;
  foot_origin_shift_y_ = 0.186;

  // Foot Trajectory Parameter
  dsp_ratio_ = dsp_ratio; // default:
  foot_trajectory_max_z_ = foot_height_max; // default:

  // Preview Control Parameter
  preview_time_ = 1.6;
  lipm_height_ = lipm_height; // default:
  preview_size_ = round(preview_time_/control_cycle_);

  // ZMP Offset Parameter
  zmp_offset_x_ = zmp_offset_x; // default :
  zmp_offset_y_ = zmp_offset_y; // default :

  // Joint Initial Pose
  init_joint_pos_ = init_joint_pos;
  init_joint_vel_ = init_joint_vel;
  init_joint_accel_ = init_joint_accel;

  // Initialization
  init_body_pos_.resize(3, 0.0);
  init_body_vel_.resize(3, 0.0);
  init_body_accel_.resize(3, 0.0);
  desired_body_pos_.resize(3, 0.0);
  desired_body_vel_.resize(3, 0.0);
  desired_body_accel_.resize(3, 0.0);
  goal_body_pos_.resize(3, 0.0);
  goal_body_vel_.resize(3, 0.0);
  goal_body_accel_.resize(3, 0.0);

  init_left_foot_pos_.resize(3, 0.0);
  init_left_foot_vel_.resize(3, 0.0);
  init_left_foot_accel_.resize(3, 0.0);
  desired_left_foot_pos_.resize(3, 0.0);
  desired_left_foot_vel_.resize(3, 0.0);
  desired_left_foot_accel_.resize(3, 0.0);
  goal_left_foot_pos_.resize(3, 0.0);
  goal_left_foot_vel_.resize(3, 0.0);
  goal_left_foot_accel_.resize(3, 0.0);

  init_right_foot_pos_.resize(3, 0.0);
  init_right_foot_vel_.resize(3, 0.0);
  init_right_foot_accel_.resize(3, 0.0);
  desired_right_foot_pos_.resize(3, 0.0);
  desired_right_foot_vel_.resize(3, 0.0);
  desired_right_foot_accel_.resize(3, 0.0);
  goal_right_foot_pos_.resize(3, 0.0);
  goal_right_foot_vel_.resize(3, 0.0);
  goal_right_foot_accel_.resize(3, 0.0);

  init_body_yaw_angle_ = 0.0;

  robot_ = new thormang3::KinematicsDynamics(thormang3::WholeBody);

  x_lipm_.resize(3,1);
  y_lipm_.resize(3,1);

  for (int i=0; i<3; i++)
  {
    x_lipm_.coeffRef(i,0) = x_lipm[i];
    y_lipm_.coeffRef(i,0) = y_lipm[i];
  }

  foot_x_diff_ = 0.0;
  foot_y_diff_ = 0.0;
  foot_z_diff_ = 0.0;

  walking_leg_ = LEG_COUNT;
  walking_phase_ = PHASE_COUNT;

  desired_joint_pos_.resize(MAX_JOINT_ID, 0.0);
  desired_joint_pos_past_.resize(MAX_JOINT_ID, 0.0);
  desired_joint_vel_.resize(MAX_JOINT_ID, 0.0);
  desired_joint_vel_past_.resize(MAX_JOINT_ID, 0.0);
  desired_joint_accel_.resize(MAX_JOINT_ID, 0.0);
}

WalkingControl::~WalkingControl()
{

}

void WalkingControl::initialize(thormang3_wholebody_module_msgs::FootStepCommand foot_step_command,
                                std::vector<double_t> init_body_pos, std::vector<double_t> init_body_rot)
{
  walking_update_ = false;

  desired_body_pos_ = init_body_pos;

  robot_->thormang3_link_data_[ID_PELVIS_POS_X]->relative_position_.coeffRef(0,0) = init_body_pos[0];
  robot_->thormang3_link_data_[ID_PELVIS_POS_Y]->relative_position_.coeffRef(1,0) = init_body_pos[1];
  robot_->thormang3_link_data_[ID_PELVIS_POS_Z]->relative_position_.coeffRef(2,0) = init_body_pos[2];

  Eigen::Quaterniond init_body_quat(init_body_rot[3],init_body_rot[0],init_body_rot[1],init_body_rot[2]);
  Eigen::MatrixXd init_body_rpy = robotis_framework::convertQuaternionToRPY(init_body_quat);

  robot_->thormang3_link_data_[ID_PELVIS_ROT_X]->joint_angle_ = init_body_rpy.coeff(0,0);
  robot_->thormang3_link_data_[ID_PELVIS_ROT_Y]->joint_angle_ = init_body_rpy.coeff(1,0);
  robot_->thormang3_link_data_[ID_PELVIS_ROT_Z]->joint_angle_ = init_body_rpy.coeff(2,0);

  init_body_yaw_angle_ = init_body_rpy.coeff(2,0);

  for (int id=1; id<=MAX_JOINT_ID; id++)
    robot_->thormang3_link_data_[id]->joint_angle_ = init_joint_pos_[id-1];

  robot_->calcForwardKinematics(0);

  Eigen::MatrixXd init_left_foot_pos = robot_->thormang3_link_data_[ID_L_LEG_END]->position_;
  Eigen::MatrixXd init_right_foot_pos = robot_->thormang3_link_data_[ID_R_LEG_END]->position_;

  init_body_pos_ = init_body_pos;
  for (int i=0; i<3; i++)
  {
    init_left_foot_pos_[i] = init_left_foot_pos.coeff(i,0);
    init_right_foot_pos_[i] = init_right_foot_pos.coeff(i,0);
  }

  desired_left_foot_pos_ = init_left_foot_pos_;
  desired_right_foot_pos_ = init_right_foot_pos_;

  ROS_INFO("init_left_foot_pos_ x: %f, y: %f, z: %f", init_left_foot_pos_[0], init_left_foot_pos_[1], init_left_foot_pos_[2]);
  ROS_INFO("init_right_foot_pos_ x: %f, y: %f, z: %f", init_right_foot_pos_[0], init_right_foot_pos_[1], init_right_foot_pos_[2]);
  ROS_INFO("init_body_pos_ x: %f, y: %f, z: %f", init_body_pos_[0], init_body_pos_[1], init_body_pos_[2]);

  Eigen::MatrixXd body_rot = robot_->thormang3_link_data_[ID_PELVIS]->orientation_;
  Eigen::MatrixXd left_foot_rot = robot_->thormang3_link_data_[ID_L_LEG_END]->orientation_;
  Eigen::MatrixXd right_foot_rot = robot_->thormang3_link_data_[ID_R_LEG_END]->orientation_;
  init_body_quaternion_ = robotis_framework::convertRotationToQuaternion(body_rot);
  init_left_foot_quaternion_ = robotis_framework::convertRotationToQuaternion(left_foot_rot);
  init_right_foot_quaternion_ = robotis_framework::convertRotationToQuaternion(right_foot_rot);

  desired_body_quaternion_ = init_body_quaternion_;
  desired_left_foot_quaternion_ = init_left_foot_quaternion_;
  desired_right_foot_quaternion_ = init_right_foot_quaternion_;

  // Calculation Foot Step
  foot_step_command_ = foot_step_command;
  calcFootStepParam();

  sum_of_zmp_x_ = 0.0;
  sum_of_zmp_y_ = 0.0;
  sum_of_cx_ = 0.0;
  sum_of_cy_ = 0.0;

  u_x_.resize(1,1);
  u_y_.resize(1,1);
  u_x_.fill(0.0);
  u_y_.fill(0.0);
}

void WalkingControl::update(int walking_leg, int walking_phase,
                            thormang3_wholebody_module_msgs::FootStepCommand foot_step_command,
                            std::vector<double_t> init_body_pos, std::vector<double_t> init_body_rot,
                            std::vector<double_t> init_left_foot_pos, std::vector<double_t> init_left_foot_vel, std::vector<double_t> init_left_foot_accel,
                            std::vector<double_t> init_right_foot_pos, std::vector<double_t> init_right_foot_vel, std::vector<double_t> init_right_foot_accel)
{
  if (walking_phase == SSP)
    walking_update_ = true;

  desired_body_pos_ = init_body_pos;

  walking_leg_ = walking_leg;
  walking_phase_ = walking_phase;

  robot_->thormang3_link_data_[ID_PELVIS_POS_X]->relative_position_.coeffRef(0,0) = init_body_pos[0];
  robot_->thormang3_link_data_[ID_PELVIS_POS_Y]->relative_position_.coeffRef(1,0) = init_body_pos[1];
  robot_->thormang3_link_data_[ID_PELVIS_POS_Z]->relative_position_.coeffRef(2,0) = init_body_pos[2];

  Eigen::Quaterniond init_body_quat(init_body_rot[3],init_body_rot[0],init_body_rot[1],init_body_rot[2]);
  Eigen::MatrixXd init_body_rpy = robotis_framework::convertQuaternionToRPY(init_body_quat);

  robot_->thormang3_link_data_[ID_PELVIS_ROT_X]->joint_angle_  = init_body_rpy.coeff(0,0);
  robot_->thormang3_link_data_[ID_PELVIS_ROT_Y]->joint_angle_ = init_body_rpy.coeff(1,0);
  robot_->thormang3_link_data_[ID_PELVIS_ROT_Z]->joint_angle_   = init_body_rpy.coeff(2,0);

  init_body_yaw_angle_ = init_body_rpy.coeff(2,0);

  for (int id=1; id<=MAX_JOINT_ID; id++)
    robot_->thormang3_link_data_[id]->joint_angle_ = init_joint_pos_[id-1];

  robot_->calcForwardKinematics(0);

  init_body_pos_ = init_body_pos;
  for (int i=0; i<3; i++)
  {
    init_left_foot_pos_[i] = robot_->thormang3_link_data_[ID_L_LEG_END]->position_.coeff(i,0);
    init_right_foot_pos_[i] = robot_->thormang3_link_data_[ID_R_LEG_END]->position_.coeff(i,0);
  }

  init_left_foot_vel_ = init_left_foot_vel;
  init_left_foot_accel_ = init_left_foot_accel;

  init_right_foot_vel_ = init_right_foot_vel;
  init_right_foot_accel_ = init_right_foot_accel;

  Eigen::MatrixXd body_rot = robot_->thormang3_link_data_[ID_PELVIS]->orientation_;
  Eigen::MatrixXd left_foot_rot = robot_->thormang3_link_data_[ID_L_LEG_END]->orientation_;
  Eigen::MatrixXd right_foot_rot = robot_->thormang3_link_data_[ID_R_LEG_END]->orientation_;
  init_body_quaternion_ = robotis_framework::convertRotationToQuaternion(body_rot);
  init_left_foot_quaternion_ = robotis_framework::convertRotationToQuaternion(left_foot_rot);
  init_right_foot_quaternion_ = robotis_framework::convertRotationToQuaternion(right_foot_rot);

  if (walking_leg_ == LEFT_LEG)
  {
    ROS_INFO("LEFT_LEG");
    foot_x_diff_ = init_left_foot_pos_[0] - init_right_foot_pos_[0];
    foot_y_diff_ = init_left_foot_pos_[1] - init_right_foot_pos_[1];
    foot_z_diff_ = init_left_foot_pos_[2] - init_right_foot_pos_[2];
  }
  else if (walking_leg_ == RIGHT_LEG)
  {
    ROS_INFO("RIGHT_LEG");
    foot_x_diff_ = init_right_foot_pos_[0] - init_left_foot_pos_[0];
    foot_y_diff_ = init_right_foot_pos_[1] - init_left_foot_pos_[1];
    foot_z_diff_ = init_right_foot_pos_[1] - init_left_foot_pos_[1];
  }

  ROS_INFO("foot diff x: %f, y: %f, z: %f", foot_x_diff_, foot_y_diff_, foot_z_diff_);

//  ROS_INFO("init_left_foot_pos_ x: %f, y: %f, z: %f", init_left_foot_pos_[0], init_left_foot_pos_[1], init_left_foot_pos_[2]);
//  ROS_INFO("init_right_foot_pos_ x: %f, y: %f, z: %f", init_right_foot_pos_[0], init_right_foot_pos_[1], init_right_foot_pos_[2]);
//  ROS_INFO("init_body_pos_ x: %f, y: %f, z: %f", init_body_pos_[0], init_body_pos_[1], init_body_pos_[2]);

  // Calculation Foot Step
  foot_step_command_ = foot_step_command;
  updateFootStepParam();

  sum_of_zmp_x_ = 0.0;
  sum_of_zmp_y_ = 0.0;
  sum_of_cx_ = 0.0;
  sum_of_cy_ = 0.0;

  u_x_.resize(1,1);
  u_y_.resize(1,1);
  u_x_.fill(0.0);
  u_y_.fill(0.0);
}

void WalkingControl::next()
{
  init_right_foot_pos_ = goal_right_foot_pos_;
  init_right_foot_vel_ = goal_right_foot_vel_;
  init_right_foot_accel_ = goal_right_foot_accel_;

  init_left_foot_pos_ = goal_left_foot_pos_;
  init_left_foot_vel_ = goal_left_foot_vel_;
  init_left_foot_accel_ = goal_left_foot_accel_;
}

double WalkingControl::getLipmHeight()
{
  return lipm_height_;
}

void WalkingControl::finalize()
{

}

bool WalkingControl::set(double time, int step)
{
  if (time == 0.0)
    calcFootTrajectory(step);

  calcFootStepPose(time, step);
  calcRefZMP(step);
  calcPreviewControl(time, step);

//  if (walking_leg_ == LEFT_LEG)
//    ROS_INFO("walking_leg_ : LEFT");
//  else if (walking_leg_ == RIGHT_LEG)
//    ROS_INFO("walking_leg_ : RIGHT");

  /* ----- Inverse Kinematics ---- */
  double dsp_length = 0.5*(fin_time_ - init_time_)*dsp_ratio_;

  double init_time = init_time_ + dsp_length;
  double fin_time = fin_time_ - dsp_length;

  int     max_iter    = 30;
  double  ik_tol      = 1e-4;

  bool ik_success = false;

  // body
  if (time < init_time)
  {
    walking_phase_ = DSP;
    desired_body_quaternion_ = init_body_quaternion_;
  }
  else if (time > fin_time)
  {
    walking_phase_ = DSP;
    desired_body_quaternion_ = goal_body_quaternion_;
  }
  else
  {
    walking_phase_ = SSP;
    double count = (time - init_time) / fin_time;
    desired_body_quaternion_ = init_body_quaternion_.slerp(count, goal_body_quaternion_);
  }

  if (step == 0 || step == foot_step_size_ -1)
    walking_phase_ = DSP;

//  if (walking_phase_ == DSP)
//    ROS_INFO("DSP");
//  else if (walking_phase_ == SSP)
//    ROS_INFO("SSP");

  robot_->thormang3_link_data_[ID_PELVIS_POS_X]->relative_position_.coeffRef(0,0) = desired_body_pos_[0];
  robot_->thormang3_link_data_[ID_PELVIS_POS_Y]->relative_position_.coeffRef(1,0) = desired_body_pos_[1];
  //op3_->thormang3_link_data_[ID_PELVIS_POS_Z]->relative_position_.coeffRef(2,0) = 0.25;

  Eigen::MatrixXd desired_body_rpy = robotis_framework::convertQuaternionToRPY(desired_body_quaternion_);

  robot_->thormang3_link_data_[ID_PELVIS_ROT_X]->joint_angle_  = desired_body_rpy.coeff(0,0);
  robot_->thormang3_link_data_[ID_PELVIS_ROT_Y]->joint_angle_ = desired_body_rpy.coeff(1,0);
  robot_->thormang3_link_data_[ID_PELVIS_ROT_Z]->joint_angle_   = desired_body_rpy.coeff(2,0);

  // right foot
  Eigen::MatrixXd desired_right_foot_pos = Eigen::MatrixXd::Zero(3,1);
  desired_right_foot_pos.coeffRef(0,0) = desired_right_foot_pos_[0];
  desired_right_foot_pos.coeffRef(1,0) = desired_right_foot_pos_[1];
  desired_right_foot_pos.coeffRef(2,0) = desired_right_foot_pos_[2];

//  ros::Time begin = ros::Time::now();

  if (time < init_time)
    desired_right_foot_quaternion_ = init_right_foot_quaternion_;
  else if (time > fin_time)
    desired_right_foot_quaternion_ = goal_right_foot_quaternion_;
  else
  {
    double count = (time - init_time) / fin_time;
    desired_right_foot_quaternion_ = init_right_foot_quaternion_.slerp(count, goal_right_foot_quaternion_);
  }

//  Eigen::MatrixXd desired_right_foot_rot = robotis_framework::convertQuaternionToRotation(desired_right_foot_quaternion_);

  bool ik_rleg_success = true;
//  ik_rleg_success = robot_->calcInverseKinematics(ID_PELVIS, ID_R_LEG_END,
//                                                  desired_right_foot_pos, desired_right_foot_rot,
//                                                  max_iter, ik_tol);

  // left foot
  Eigen::MatrixXd desired_left_foot_pos = Eigen::MatrixXd::Zero(3,1);
  desired_left_foot_pos.coeffRef(0,0) = desired_left_foot_pos_[0];
  desired_left_foot_pos.coeffRef(1,0) = desired_left_foot_pos_[1];
  desired_left_foot_pos.coeffRef(2,0) = desired_left_foot_pos_[2];

  if (time < init_time)
    desired_left_foot_quaternion_ = init_left_foot_quaternion_;
  else if (time > fin_time)
    desired_left_foot_quaternion_ = goal_left_foot_quaternion_;
  else
  {
    double count = (time - init_time) / fin_time;
    desired_left_foot_quaternion_ = init_left_foot_quaternion_.slerp(count, goal_left_foot_quaternion_);
  }

//  Eigen::MatrixXd desired_left_foot_rot = robotis_framework::convertQuaternionToRotation(desired_left_foot_quaternion_);

  bool ik_lleg_success = true;
//  ik_lleg_success = robot_->calcInverseKinematicsWOFK(ID_PELVIS, ID_L_LEG_END,
//                                                      desired_left_foot_pos, desired_left_foot_rot,
//                                                      max_iter, ik_tol);

//  ros::Duration time_duration = ros::Time::now() - begin;
//  ROS_INFO("calc time: %f", time_duration.toSec());

  if (ik_rleg_success == true && ik_lleg_success == true)
    ik_success = true;
  else
    ik_success = false;

  return ik_success;
}

void WalkingControl::calcFootStepParam()
{
  fin_time_ = foot_step_command_.step_time;
  foot_step_size_ = foot_step_command_.step_num;

  int walking_start_leg;
  if (foot_step_command_.start_leg == "left_leg")
    walking_start_leg = RIGHT_LEG;
  else if (foot_step_command_.start_leg == "right_leg")
    walking_start_leg = LEFT_LEG;

  if (foot_step_command_.command == "right")
    walking_start_leg = LEFT_LEG;
  else if (foot_step_command_.command == "left")
    walking_start_leg = RIGHT_LEG;
  else if (foot_step_command_.command == "turn_right")
    walking_start_leg = LEFT_LEG;
  else if (foot_step_command_.command == "turn_left")
    walking_start_leg = RIGHT_LEG;

  int walking_leg = walking_start_leg;
  double foot_angle = init_body_yaw_angle_;

  for (int i=0; i<foot_step_size_; i++)
  {
    geometry_msgs::Pose2D msg;

    // Forward Step
    msg.x = foot_step_command_.step_length;

    if (foot_step_command_.command == "stop")
      msg.x *= 0.0;

    if (foot_step_command_.command == "backward")
      msg.x *= -1.0;

    if (foot_step_command_.command == "left" ||
        foot_step_command_.command == "right")
      msg.x *= 0.0;

    if (foot_step_command_.command == "turn_left" ||
        foot_step_command_.command == "turn_right")
      msg.x *= 0.0;

    // Side Step
    walking_leg = walking_start_leg++ % LEG_COUNT;
    double lr = walking_leg;
    if (foot_step_command_.command == "left")
    {
      lr += -1.0; lr *= -1.0;
    }

    if ((foot_step_command_.command == "forward" || foot_step_command_.command == "backward") &&
        foot_step_command_.start_leg == "left_leg")
    {
      lr += -1.0; lr *= -1.0;
    }

    if (foot_step_command_.command == "turn_left" ||
        foot_step_command_.command == "turn_right")
      lr = 0.0;

    if (foot_step_command_.command == "stop")
      lr *= 0.0;

    msg.y = foot_origin_shift_y_ + lr*foot_step_command_.side_length;

    // Theta
    double theta;
    theta = foot_step_command_.step_angle;

    if (foot_step_command_.command == "turn_right")
      theta *= -1.0;

    if ((foot_step_command_.command == "forward" || foot_step_command_.command == "backward") &&
        foot_step_command_.start_leg == "right_leg")
      theta *= -1.0;

    if (foot_step_command_.command == "left" ||
        foot_step_command_.command == "right")
      theta *= 0.0;

    if (foot_step_command_.command == "stop")
      theta *= 0.0;

    if (i == 0 ||
        i == foot_step_size_-1 ||
        i == foot_step_size_-2)
    {
      msg.x = 0.0;
      msg.y = foot_origin_shift_y_;
      theta = 0.0;
    }

    foot_angle += theta;
    msg.theta = foot_angle;
//    msg.theta = theta;

    foot_step_param_.moving_foot.push_back(walking_leg);
    foot_step_param_.data.push_back(msg);
  }

//  ROS_INFO("--");
//  for (int i=0; i<foot_step_size_; i++)
//  {
//    ROS_INFO("movint foot : %d", foot_step_param_.moving_foot[i]);
//    ROS_INFO("foot position x: %f", foot_step_param_.data[i].x);
//    ROS_INFO("foot position y: %f", foot_step_param_.data[i].y);
//    ROS_INFO("foot position angle: %f", foot_step_param_.data[i].theta);
//  }
}

void WalkingControl::updateFootStepParam()
{
  fin_time_ = foot_step_command_.step_time;
  foot_step_size_ = foot_step_command_.step_num;

  int walking_start_leg;
  walking_start_leg = walking_leg_;

  int walking_leg = walking_start_leg;
  double foot_angle = init_body_yaw_angle_;

  for (int i=0; i<foot_step_size_; i++)
  {
    geometry_msgs::Pose2D msg;

    /* TODO */
//    if ( i==0 )
//    {
//      ROS_INFO("set");
//      if (foot_x_diff_ > 0.02)
//        msg.x = foot_x_diff_;
//    }
//    else
//      msg.x = 0.0;

    // Forward Step
    msg.x = 0.0;

    // Side Step
    walking_leg = walking_start_leg++ % LEG_COUNT;

    msg.y = foot_origin_shift_y_;

    // Theta
    double theta;
    theta = foot_step_command_.step_angle;
    msg.theta = theta;

    foot_step_param_.moving_foot.push_back(walking_leg);
    foot_step_param_.data.push_back(msg);
  }

//  ROS_INFO("--");
//  for (int i=0; i<foot_step_size_; i++)
//  {
//    ROS_INFO("movint foot : %d", foot_step_param_.moving_foot[i]);
//    ROS_INFO("foot position x: %f", foot_step_param_.data[i].x);
//    ROS_INFO("foot position y: %f", foot_step_param_.data[i].y);
//    ROS_INFO("foot position angle: %f", foot_step_param_.data[i].theta);
//  }
}

void WalkingControl::calcFootTrajectory(int step)
{
//  Eigen::MatrixXd body_rot = robot_->thormang3_link_data_[ID_PELVIS]->orientation_;
//  Eigen::MatrixXd left_foot_rot = robot_->thormang3_link_data_[ID_L_LEG_END]->orientation_;
//  Eigen::MatrixXd right_foot_rot = robot_->thormang3_link_data_[ID_R_LEG_END]->orientation_;

  Eigen::MatrixXd body_rot = robotis_framework::convertQuaternionToRotation(desired_body_quaternion_);
  Eigen::MatrixXd left_foot_rot = robotis_framework::convertQuaternionToRotation(desired_left_foot_quaternion_);
  Eigen::MatrixXd right_foot_rot = robotis_framework::convertQuaternionToRotation(desired_right_foot_quaternion_);

  init_body_quaternion_ = robotis_framework::convertRotationToQuaternion(body_rot);
  init_left_foot_quaternion_ = robotis_framework::convertRotationToQuaternion(left_foot_rot);
  init_right_foot_quaternion_ = robotis_framework::convertRotationToQuaternion(right_foot_rot);

  if (foot_step_param_.moving_foot[step] == LEFT_LEG)
  {
    double angle = foot_step_param_.data[step].theta;

    // Goal
    goal_left_foot_pos_[0] = init_right_foot_pos_[0]
        + cos(angle)*foot_step_param_.data[step].x
        - sin(angle)*foot_step_param_.data[step].y;
    goal_left_foot_pos_[1] = init_right_foot_pos_[1]
        + sin(angle)*foot_step_param_.data[step].x
        + cos(angle)*foot_step_param_.data[step].y;
    goal_left_foot_pos_[2] = init_right_foot_pos_[2];

    goal_left_foot_quaternion_ = robotis_framework::convertRPYToQuaternion(0.0, 0.0, angle);

    goal_right_foot_pos_ = init_right_foot_pos_;
    goal_right_foot_quaternion_ = init_right_foot_quaternion_;

    goal_body_quaternion_ = robotis_framework::convertRPYToQuaternion(0.0, 0.0, angle);

    // Via point
    double via_time = 0.5*(init_time_ + fin_time_);

    std::vector<double_t> via_left_foot_pos, via_left_foot_vel, via_left_foot_accel;
    via_left_foot_pos.resize(3, 0.0);
    via_left_foot_vel.resize(3, 0.0);
    via_left_foot_accel.resize(3, 0.0);

    via_left_foot_pos[0] = 0.5*(init_left_foot_pos_[0] + goal_left_foot_pos_[0]);
    via_left_foot_pos[1] = 0.5*(init_left_foot_pos_[1] + goal_left_foot_pos_[1]);
    via_left_foot_pos[2] = foot_trajectory_max_z_;

    if (walking_update_ == false)
    {
      if (step == 0)
        via_left_foot_pos[2] = 0.0;
    }

    if (step == foot_step_size_-1)
      via_left_foot_pos[2] = 0.0;

    // Trajectory
    left_foot_trajectory_ = new robotis_framework::MinimumJerkViaPoint(init_time_, fin_time_, via_time, dsp_ratio_,
                                                                       init_left_foot_pos_, init_left_foot_vel_, init_left_foot_accel_,
                                                                       goal_left_foot_pos_, goal_left_foot_vel_, goal_left_foot_accel_,
                                                                       via_left_foot_pos, via_left_foot_vel, via_left_foot_accel);

//    ROS_INFO("angle: %f", angle);
  }
  else if (foot_step_param_.moving_foot[step] == RIGHT_LEG)
  {
    double angle = foot_step_param_.data[step].theta;

    // Goal
    goal_right_foot_pos_[0] = init_left_foot_pos_[0]
        + cos(angle)*foot_step_param_.data[step].x
        + sin(angle)*foot_step_param_.data[step].y;
    goal_right_foot_pos_[1] = init_left_foot_pos_[1]
        + sin(angle)*foot_step_param_.data[step].x
        - cos(angle)*foot_step_param_.data[step].y;
    goal_right_foot_pos_[2] = init_left_foot_pos_[2];

    goal_right_foot_quaternion_ = robotis_framework::convertRPYToQuaternion(0.0, 0.0, angle);

    goal_left_foot_pos_ = init_left_foot_pos_;
    goal_left_foot_quaternion_ = init_left_foot_quaternion_;

    goal_body_quaternion_ = robotis_framework::convertRPYToQuaternion(0.0, 0.0, angle);

    // Via point
    double via_time = 0.5*(init_time_ + fin_time_);

    std::vector<double_t> via_right_foot_pos, via_right_foot_vel, via_right_foot_accel;
    via_right_foot_pos.resize(3, 0.0);
    via_right_foot_vel.resize(3, 0.0);
    via_right_foot_accel.resize(3, 0.0);

    via_right_foot_pos[0] = 0.5*(init_right_foot_pos_[0] + goal_right_foot_pos_[0]);
    via_right_foot_pos[1] = 0.5*(init_right_foot_pos_[1] + goal_right_foot_pos_[1]);
    via_right_foot_pos[2] = foot_trajectory_max_z_;

    if (walking_update_ == false)
    {
      if (step == 0)
        via_right_foot_pos[2] = 0.0;
    }

    if (step == foot_step_size_-1)
      via_right_foot_pos[2] = 0.0;

    // Trajectory
    right_foot_trajectory_ = new robotis_framework::MinimumJerkViaPoint(init_time_, fin_time_, via_time, dsp_ratio_,
                                                                        init_right_foot_pos_, init_right_foot_vel_, init_right_foot_accel_,
                                                                        goal_right_foot_pos_, goal_right_foot_vel_, goal_right_foot_accel_,
                                                                        via_right_foot_pos, via_right_foot_vel, via_right_foot_accel);

//    ROS_INFO("angle: %f", angle);
  }

//  for (int i=0; i<3; i++)
//    ROS_INFO("init_left_foot_pos_[%d] : %f",i,init_left_foot_pos_[i]);
//  for (int i=0; i<3; i++)
//    ROS_INFO("goal_left_foot_pos_[%d] : %f", i, goal_left_foot_pos_[i]);
//  for (int i=0; i<3; i++)
//    ROS_INFO("init_right_foot_pos_[%d] : %f",i,init_right_foot_pos_[i]);
//  for (int i=0; i<3; i++)
//    ROS_INFO("goal_right_foot_pos_[%d] : %f", i, goal_right_foot_pos_[i]);
}

void WalkingControl::calcFootStepPose(double time, int step)
{
  if (foot_step_param_.moving_foot[step] == LEFT_LEG)
  {
    desired_left_foot_pos_ = left_foot_trajectory_->getPosition(time);
    desired_left_foot_vel_ = left_foot_trajectory_->getVelocity(time);
    desired_left_foot_accel_ = left_foot_trajectory_->getAcceleration(time);

    desired_right_foot_pos_ = goal_right_foot_pos_;
    desired_right_foot_vel_.resize(3, 0.0);
    desired_right_foot_accel_.resize(3, 0.0);

    walking_leg_ = LEFT_LEG;

//    ROS_INFO("left foot x: %f, y: %f, z: %f", desired_left_foot_pos_[0], desired_left_foot_pos_[1], desired_left_foot_pos_[2]);
  }
  else if (foot_step_param_.moving_foot[step] == RIGHT_LEG)
  {
    desired_right_foot_pos_ = right_foot_trajectory_->getPosition(time);
    desired_right_foot_vel_ = right_foot_trajectory_->getVelocity(time);
    desired_right_foot_accel_ = right_foot_trajectory_->getAcceleration(time);

    desired_left_foot_pos_ = goal_left_foot_pos_;
    desired_left_foot_vel_.resize(3, 0.0);
    desired_left_foot_accel_.resize(3, 0.0);

    walking_leg_ = RIGHT_LEG;

//    ROS_INFO("right foot x: %f, y: %f, z: %f", desired_right_foot_pos_[0], desired_right_foot_pos_[1], desired_right_foot_pos_[2]);
  }
}

void WalkingControl::calcRefZMP(int step)
{
  if (step == 0)
  {
    if (walking_update_ == false)
    {
      ref_zmp_x_ = 0.5*(goal_right_foot_pos_[0] + goal_left_foot_pos_[0]);
      ref_zmp_y_ = 0.5*(goal_right_foot_pos_[1] + goal_left_foot_pos_[1]);
    }
    else if (walking_update_ == true)
    {
      if (foot_step_param_.moving_foot[step] == LEFT_LEG)
      {
        ref_zmp_x_ = goal_right_foot_pos_[0];
        ref_zmp_y_ = goal_right_foot_pos_[1] + zmp_offset_y_;
      }
      else if (foot_step_param_.moving_foot[step] == RIGHT_LEG)
      {
        ref_zmp_x_ = goal_left_foot_pos_[0];
        ref_zmp_y_ = goal_left_foot_pos_[1] - zmp_offset_y_;
      }
    }
  }
  else if (step == foot_step_size_-1)
  {
    ref_zmp_x_ = 0.5*(goal_right_foot_pos_[0] + goal_left_foot_pos_[0]);
    ref_zmp_y_ = 0.5*(goal_right_foot_pos_[1] + goal_left_foot_pos_[1]);
  }
  else
  {
    if (foot_step_param_.moving_foot[step] == LEFT_LEG)
    {
      ref_zmp_x_ = goal_right_foot_pos_[0];
      ref_zmp_y_ = goal_right_foot_pos_[1] + zmp_offset_y_;
    }
    else if (foot_step_param_.moving_foot[step] == RIGHT_LEG)
    {
      ref_zmp_x_ = goal_left_foot_pos_[0];
      ref_zmp_y_ = goal_left_foot_pos_[1] - zmp_offset_y_;
    }
  }

//  ROS_INFO("ref zmp x: %f, y: %f", ref_zmp_x_, ref_zmp_y_);
}

double WalkingControl::calcRefZMPx(int step)
{
  double ref_zmp_x;

  if (step == 0)
  {
    if (walking_update_ == false)
    {
      ref_zmp_x = 0.5 * (goal_right_foot_pos_[0] + goal_left_foot_pos_[0]);
    }
    else if (walking_update_ == true)
    {
      if (foot_step_param_.moving_foot[step] == LEFT_LEG)
        ref_zmp_x = goal_right_foot_pos_[0];
      else if (foot_step_param_.moving_foot[step] == RIGHT_LEG)
        ref_zmp_x = goal_left_foot_pos_[0];
    }
  }
  else if (step >= foot_step_size_-1)
  {
    ref_zmp_x = 0.5 * (goal_right_foot_pos_[0] + goal_left_foot_pos_[0]);
  }
  else
  {
    if (foot_step_param_.moving_foot[step] == LEFT_LEG)
      ref_zmp_x = goal_right_foot_pos_[0];
    else if (foot_step_param_.moving_foot[step] == RIGHT_LEG)
      ref_zmp_x = goal_left_foot_pos_[0];
  }

  return ref_zmp_x;
}

double WalkingControl::calcRefZMPy(int step)
{
  double ref_zmp_y;

  if (step == 0)
  {
    if (walking_update_ == false)
    {
      ref_zmp_y = 0.5 * (goal_right_foot_pos_[1] + goal_left_foot_pos_[1]);
    }
    else
    {
      if (foot_step_param_.moving_foot[step] == LEFT_LEG)
        ref_zmp_y = goal_right_foot_pos_[1] + zmp_offset_y_;
      else if (foot_step_param_.moving_foot[step] == RIGHT_LEG)
        ref_zmp_y = goal_left_foot_pos_[1] - zmp_offset_y_;
    }
  }
  else if (step >= foot_step_size_-1)
  {
    ref_zmp_y = 0.5 * (goal_right_foot_pos_[1] + goal_left_foot_pos_[1]);
  }
  else
  {
    if (foot_step_param_.moving_foot[step] == LEFT_LEG)
      ref_zmp_y = goal_right_foot_pos_[1] + zmp_offset_y_;
    else if (foot_step_param_.moving_foot[step] == RIGHT_LEG)
      ref_zmp_y = goal_left_foot_pos_[1] - zmp_offset_y_;
  }

  return ref_zmp_y;
}

void WalkingControl::calcPreviewParam(thormang3_wholebody_module_msgs::PreviewResponse msg)
{
  //
  ROS_INFO("lipm_height_ : %f", lipm_height_);

  double t = control_cycle_;

  A_.resize(3,3);
  A_ << 1,  t,  t*t/2.0,
        0,  1,  t,
        0,  0,  1;

  b_.resize(3,1);
  b_ << t*t*t/6.0,
        t*t/2.0,
        t;

  c_.resize(1,3);
  c_ << 1, 0, -lipm_height_/9.81;

  //
  preview_response_ = msg;

  int row_K = preview_response_.K_row;
  int col_K = preview_response_.K_col;
  std::vector<double_t> matrix_K = preview_response_.K;

  int row_P = preview_response_.P_row;
  int col_P = preview_response_.P_col;
  std::vector<double_t> matrix_P = preview_response_.P;

  K_.resize(row_K,col_K);
  P_.resize(row_P,col_P);

  for(int j=0 ; j<row_K ; j++)
  {
    for(int i=0 ; i<col_K ; i++)
      K_.coeffRef(j,i) = matrix_K[i*row_K+j];
  }

  for(int j=0; j<row_P; j++)
  {
    for(int i=0 ; i<col_P; i++)
      P_.coeffRef(j,i) = matrix_P[i*row_P+j];
  }

  k_s_ = K_.coeff(0,0);
  k_x_.resize(1,3);
  k_x_ << K_.coeff(0,1), K_.coeff(0,2), K_.coeff(0,3);

  f_ = robot_->calcPreviewParam(preview_time_, control_cycle_,
                              lipm_height_,
                              K_, P_);
}

void WalkingControl::calcPreviewControl(double time, int step)
{
  double time_new;
  int step_new;

  preview_sum_zmp_x_ = 0.0;
  preview_sum_zmp_y_ = 0.0;

  for (int i=0; i<preview_size_; i++)
  {
    time_new = time + i*control_cycle_;

    if (time_new > fin_time_)
    {
      int num = time_new/fin_time_;
      step_new = step + num;
    }
    else
      step_new = step;

    double ref_zmp_x = calcRefZMPx(step_new);
    double ref_zmp_y = calcRefZMPy(step_new);

    double preview_zmp_x = f_.coeff(0,i)*ref_zmp_x;
    double preview_zmp_y = f_.coeff(0,i)*ref_zmp_y;

    preview_sum_zmp_x_ += preview_zmp_x;
    preview_sum_zmp_y_ += preview_zmp_y;
  }

//  ROS_INFO("preview_sum_zmp x: %f , y: %f", preview_sum_zmp_x_, preview_sum_zmp_y_);

  double cx = c_(0,0)*x_lipm_(0,0) + c_(0,1)*x_lipm_(1,0) + c_(0,2)*x_lipm_(2,0);
  double cy = c_(0,0)*y_lipm_(0,0) + c_(0,1)*y_lipm_(1,0) + c_(0,2)*y_lipm_(2,0);

  sum_of_cx_ += cx;
  sum_of_cy_ += cy;

  sum_of_zmp_x_ += ref_zmp_x_;
  sum_of_zmp_y_ += ref_zmp_y_;

  u_x_(0,0) =
      -k_s_*(sum_of_cx_ - sum_of_zmp_x_)
      -(k_x_(0,0)*x_lipm_(0,0) + k_x_(0,1)*x_lipm_(1,0) + k_x_(0,2)*x_lipm_(2,0))
      + preview_sum_zmp_x_;
  u_y_(0,0) =
      -k_s_*(sum_of_cy_ - sum_of_zmp_y_)
      -(k_x_(0,0)*y_lipm_(0,0) + k_x_(0,1)*y_lipm_(1,0) + k_x_(0,2)*y_lipm_(2,0))
      + preview_sum_zmp_y_;

  x_lipm_ = A_*x_lipm_ + b_*u_x_;
  y_lipm_ = A_*y_lipm_ + b_*u_y_;

  desired_body_pos_[0] = x_lipm_.coeff(0,0) + zmp_offset_x_;
  desired_body_pos_[1] = y_lipm_.coeff(0,0);

//  ROS_INFO("time: %f, desired_body_pos_ x: %f , y: %f", time, desired_body_pos_[0], desired_body_pos_[1]);
}

std::vector<double_t> WalkingControl::getJointPosition(int step, double time)
{
//  std::vector<double_t> desired_joint_pos;
//  desired_joint_pos.resize(MAX_JOINT_ID, 0.0);

  for (int id=1; id<=MAX_JOINT_ID; id++)
  {
    desired_joint_pos_[id-1] = robot_->thormang3_link_data_[id]->joint_angle_;
    desired_joint_vel_[id-1] = (desired_joint_pos_[id-1] - desired_joint_pos_past_[id-1]) / control_cycle_;
  }

  if (step == 0)
  {
    if (time == 0.0)
    {
      for (int id=1; id<=MAX_JOINT_ID; id++)
        desired_joint_vel_[id-1] = 0.0;
    }
  }

//  ROS_INFO("angle : %f", op3_->thormang3_link_data_[3]->joint_angle_);

  desired_joint_pos_past_ = desired_joint_pos_;

  return desired_joint_pos_;
}

std::vector<double_t> WalkingControl::getJointVelocity(int step, double time)
{
//  std::vector<double_t> desired_joint_vel;
//  desired_joint_vel.resize(MAX_JOINT_ID, 0.0);

  for (int id=1; id<=MAX_JOINT_ID; id++)
    desired_joint_accel_[id-1] = (desired_joint_vel_[id-1] - desired_joint_vel_past_[id-1]) / control_cycle_;

  if (step == 0)
  {
    if (time == 0.0 || time == control_cycle_)
    {
      for (int id=1; id<=MAX_JOINT_ID; id++)
        desired_joint_accel_[id-1] = 0.0;
    }
  }

  desired_joint_vel_past_ = desired_joint_vel_;

  return desired_joint_vel_;
}

std::vector<double_t> WalkingControl::getJointAcceleration(int step, double time)
{
//  std::vector<double_t> desired_joint_accel;
//  desired_joint_accel.resize(MAX_JOINT_ID, 0.0);

//  for (int id=1; id<=MAX_JOINT_ID; id++)
//    desired_joint_pos[id-1] = op3_->thormang3_link_data_[id]->joint_angle_;

  return desired_joint_accel_;
}

void WalkingControl::getWalkingPosition(std::vector<double_t> &left_foot_pos,
                                        std::vector<double_t> &right_foot_pos,
                                        std::vector<double_t> &body_pos)
{
  left_foot_pos = desired_left_foot_pos_;
  right_foot_pos = desired_right_foot_pos_;
  body_pos = desired_body_pos_;
}

void WalkingControl::getWalkingVelocity(std::vector<double_t> &left_foot_vel,
                                        std::vector<double_t> &right_foot_vel,
                                        std::vector<double_t> &body_vel)
{
  left_foot_vel = desired_left_foot_vel_;
  right_foot_vel = desired_right_foot_vel_;

  // TODO
  // body_vel =
}

void WalkingControl::getWalkingAccleration(std::vector<double_t> &left_foot_accel,
                                           std::vector<double_t> &right_foot_accel,
                                           std::vector<double_t> &body_accel)
{
  left_foot_accel = desired_left_foot_accel_;
  right_foot_accel = desired_right_foot_accel_;

  // TODO
  // body_accel =
}

void WalkingControl::getWalkingOrientation(std::vector<double_t> &left_foot_quat,
                                           std::vector<double_t> &right_foot_quat,
                                           std::vector<double_t> &body_quat)
{
//  Eigen::Quaterniond left_foot_quaternion =
//      robotis_framework::convertRotationToQuaternion(robot_->thormang3_link_data_[ID_L_LEG_END]->orientation_);

  left_foot_quat[0] = desired_left_foot_quaternion_.x();
  left_foot_quat[1] = desired_left_foot_quaternion_.y();
  left_foot_quat[2] = desired_left_foot_quaternion_.z();
  left_foot_quat[3] = desired_left_foot_quaternion_.w();

//  Eigen::Quaterniond right_foot_quaternion =
//      robotis_framework::convertRotationToQuaternion(robot_->thormang3_link_data_[ID_R_LEG_END]->orientation_);

  right_foot_quat[0] = desired_right_foot_quaternion_.x();
  right_foot_quat[1] = desired_right_foot_quaternion_.y();
  right_foot_quat[2] = desired_right_foot_quaternion_.z();
  right_foot_quat[3] = desired_right_foot_quaternion_.w();

//  Eigen::Quaterniond body_quaternion =
//      robotis_framework::convertRotationToQuaternion(robot_->thormang3_link_data_[ID_PELVIS]->orientation_);

  body_quat[0] = desired_body_quaternion_.x(); //body_quaternion.x();
  body_quat[1] = desired_body_quaternion_.y(); //body_quaternion.y();
  body_quat[2] = desired_body_quaternion_.z(); //body_quaternion.z();
  body_quat[3] = desired_body_quaternion_.w(); //body_quaternion.w();
}

void WalkingControl::getLIPM(std::vector<double_t> &x_lipm, std::vector<double_t> &y_lipm)
{
  x_lipm.resize(3, 0.0);
  y_lipm.resize(3, 0.0);

  for (int i=0; i<3; i++)
  {
    x_lipm[i] = x_lipm_.coeff(i,0);
    y_lipm[i] = y_lipm_.coeff(i,0);
  }
}

void WalkingControl::getWalkingState(int &walking_leg, int &walking_phase)
{
  walking_leg = walking_leg_;
  walking_phase = walking_phase_;
}
