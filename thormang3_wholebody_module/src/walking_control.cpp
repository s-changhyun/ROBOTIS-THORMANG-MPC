#include <stdio.h>
#include "thormang3_wholebody_module/walking_control.h"

WalkingControl::WalkingControl(double control_cycle,
                               double dsp_ratio, double lipm_height, double foot_height_max, double zmp_offset_x, double zmp_offset_y,
                               std::vector<double_t> x_lipm, std::vector<double_t> y_lipm)
  : walking_leg_(LEG_COUNT),
    walking_phase_(PHASE_COUNT)
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
  foot_tra_max_z_ = foot_height_max; // default:

  // Preview Control Parameter
  preview_time_ = 1.6;
  lipm_height_ = lipm_height; // default:
  preview_size_ = round(preview_time_/control_cycle_) + 1;

  // ZMP Offset Parameter
  zmp_offset_x_ = zmp_offset_x; // default :
  zmp_offset_y_ = zmp_offset_y; // default :

  // Initialization
  init_body_pos_.resize(3, 0.0);
  init_body_vel_.resize(3, 0.0);
  init_body_accel_.resize(3, 0.0);
  des_body_pos_.resize(3, 0.0);
  des_body_vel_.resize(3, 0.0);
  des_body_accel_.resize(3, 0.0);
  goal_body_pos_.resize(3, 0.0);
  goal_body_vel_.resize(3, 0.0);
  goal_body_accel_.resize(3, 0.0);

  init_l_foot_pos_.resize(3, 0.0);
  init_l_foot_vel_.resize(3, 0.0);
  init_l_foot_accel_.resize(3, 0.0);
  des_l_foot_pos_.resize(3, 0.0);
  des_l_foot_vel_.resize(3, 0.0);
  des_l_foot_accel_.resize(3, 0.0);
  goal_l_foot_pos_.resize(3, 0.0);
  goal_l_foot_vel_.resize(3, 0.0);
  goal_l_foot_accel_.resize(3, 0.0);

  init_r_foot_pos_.resize(3, 0.0);
  init_r_foot_vel_.resize(3, 0.0);
  init_r_foot_accel_.resize(3, 0.0);
  des_r_foot_pos_.resize(3, 0.0);
  des_r_foot_vel_.resize(3, 0.0);
  des_r_foot_accel_.resize(3, 0.0);
  goal_r_foot_pos_.resize(3, 0.0);
  goal_r_foot_vel_.resize(3, 0.0);
  goal_r_foot_accel_.resize(3, 0.0);

  init_body_yaw_angle_ = 0.0;

  robot_ = new thormang3::KinematicsDynamics(thormang3::WholeBody);

  x_lipm_.resize(3,1);
  y_lipm_.resize(3,1);

  for (int i=0; i<3; i++)
  {
    x_lipm_.coeffRef(i,0) = x_lipm[i];
    y_lipm_.coeffRef(i,0) = y_lipm[i];
  }

//  ROS_INFO("x_lipm: %f", x_lipm[0]);
//  ROS_INFO("y_lipm: %f", y_lipm[0]);
}

WalkingControl::~WalkingControl()
{

}

void WalkingControl::initialize(thormang3_wholebody_module_msgs::FootStepCommand foot_step_command,
                                std::vector<double_t> init_body_pos, std::vector<double_t> init_body_rot,
                                std::vector<double_t> init_r_foot_pos, std::vector<double_t> init_r_foot_Q,
                                std::vector<double_t> init_l_foot_pos, std::vector<double_t> init_l_foot_Q)
{
  init_body_pos_ = init_body_pos;
  des_body_pos_ = init_body_pos;

  Eigen::Quaterniond body_Q(init_body_rot[3],init_body_rot[0],init_body_rot[1],init_body_rot[2]);
  init_body_Q_ = body_Q;
  des_body_Q_ = body_Q;

  Eigen::MatrixXd init_body_rpy = robotis_framework::convertQuaternionToRPY(init_body_Q_);
  init_body_yaw_angle_ = init_body_rpy.coeff(2,0);

  init_r_foot_pos_ = init_r_foot_pos;
  init_l_foot_pos_ = init_l_foot_pos;

  des_l_foot_pos_ = init_l_foot_pos_;
  des_r_foot_pos_ = init_r_foot_pos_;

  Eigen::Quaterniond l_foot_Q(init_l_foot_Q[3],init_l_foot_Q[0],init_l_foot_Q[1],init_l_foot_Q[2]);
  init_l_foot_Q_ = l_foot_Q;
  des_l_foot_Q_ = l_foot_Q;

  Eigen::Quaterniond r_foot_Q(init_r_foot_Q[3],init_r_foot_Q[0],init_r_foot_Q[1],init_r_foot_Q[2]);
  init_r_foot_Q_ = r_foot_Q;
  desired_r_foot_Q_ = r_foot_Q;

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

void WalkingControl::next()
{
  init_r_foot_pos_    = goal_r_foot_pos_;
  init_r_foot_vel_    = goal_r_foot_vel_;
  init_r_foot_accel_  = goal_r_foot_accel_;

  init_l_foot_pos_     = goal_l_foot_pos_;
  init_l_foot_vel_     = goal_l_foot_vel_;
  init_l_foot_accel_   = goal_l_foot_accel_;
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

  calcFootStepPose(time,step);
  calcRefZMP(step);
  calcPreviewControl(time,step);

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
    des_body_Q_ = init_body_Q_;
  }
  else if (time > fin_time)
  {
    walking_phase_ = DSP;
    des_body_Q_ = goal_body_Q_;
  }
  else
  {
    walking_phase_ = SSP;
    double count = (time - init_time) / fin_time;
    des_body_Q_ = init_body_Q_.slerp(count, goal_body_Q_);
  }

  if (step == 0 || step == foot_step_size_ -1)
    walking_phase_ = DSP;

  // right foot
  Eigen::MatrixXd des_r_foot_pos = Eigen::MatrixXd::Zero(3,1);
  des_r_foot_pos.coeffRef(0,0) = des_r_foot_pos_[0];
  des_r_foot_pos.coeffRef(1,0) = des_r_foot_pos_[1];
  des_r_foot_pos.coeffRef(2,0) = des_r_foot_pos_[2];

  if (time < init_time)
    desired_r_foot_Q_ = init_r_foot_Q_;
  else if (time > fin_time)
    desired_r_foot_Q_ = goal_r_foot_Q_;
  else
  {
    double count = (time - init_time) / fin_time;
    desired_r_foot_Q_ = init_r_foot_Q_.slerp(count, goal_r_foot_Q_);
  }

  bool ik_rleg_success = true;

  // left foot
  Eigen::MatrixXd desired_left_foot_pos = Eigen::MatrixXd::Zero(3,1);
  desired_left_foot_pos.coeffRef(0,0) = des_l_foot_pos_[0];
  desired_left_foot_pos.coeffRef(1,0) = des_l_foot_pos_[1];
  desired_left_foot_pos.coeffRef(2,0) = des_l_foot_pos_[2];

  if (time < init_time)
    des_l_foot_Q_ = init_l_foot_Q_;
  else if (time > fin_time)
    des_l_foot_Q_ = goal_l_foot_Q_;
  else
  {
    double count = (time - init_time) / fin_time;
    des_l_foot_Q_ = init_l_foot_Q_.slerp(count, goal_l_foot_Q_);
  }

  bool ik_lleg_success = true;

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
    walking_start_leg = LEFT_LEG;
  else if (foot_step_command_.start_leg == "right_leg")
    walking_start_leg = RIGHT_LEG;

  if (foot_step_command_.command == "right")
    walking_start_leg = RIGHT_LEG;
  else if (foot_step_command_.command == "left")
    walking_start_leg = LEFT_LEG;
  else if (foot_step_command_.command == "turn_right")
    walking_start_leg = RIGHT_LEG;
  else if (foot_step_command_.command == "turn_left")
    walking_start_leg = LEFT_LEG;

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
        i == 1 ||
        i == foot_step_size_-1)
    {
      msg.x = 0.0;
      msg.y = foot_origin_shift_y_;
      theta = 0.0;
    }

    if (i == foot_step_size_-2)
    {
      msg.x = 0.0;
      msg.y = foot_origin_shift_y_;
      theta *= 0.0;
    }

    foot_angle += theta;
    msg.theta = foot_angle;

    foot_step_param_.moving_foot.push_back(walking_leg);
    foot_step_param_.data.push_back(msg);
  }

  calcGoalFootPose();

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
  Eigen::MatrixXd body_rot = robotis_framework::convertQuaternionToRotation(des_body_Q_);
  Eigen::MatrixXd left_foot_rot = robotis_framework::convertQuaternionToRotation(des_l_foot_Q_);
  Eigen::MatrixXd r_foot_rot = robotis_framework::convertQuaternionToRotation(desired_r_foot_Q_);

  init_body_Q_ = robotis_framework::convertRotationToQuaternion(body_rot);
  init_l_foot_Q_ = robotis_framework::convertRotationToQuaternion(left_foot_rot);
  init_r_foot_Q_ = robotis_framework::convertRotationToQuaternion(r_foot_rot);

  if (foot_step_param_.moving_foot[step] == LEFT_LEG)
  {
    double angle = foot_step_param_.data[step].theta;

    // Goal
    goal_l_foot_pos_[0] = init_r_foot_pos_[0]
        + cos(angle) * foot_step_param_.data[step].x
        - sin(angle) * foot_step_param_.data[step].y;
    goal_l_foot_pos_[1] = init_r_foot_pos_[1]
        + sin(angle) * foot_step_param_.data[step].x
        + cos(angle) * foot_step_param_.data[step].y;
    goal_l_foot_pos_[2] = init_r_foot_pos_[2];

    goal_l_foot_Q_ = robotis_framework::convertRPYToQuaternion(0.0, 0.0, angle);

    goal_r_foot_pos_ = init_r_foot_pos_;
    goal_r_foot_Q_ = init_r_foot_Q_;

    goal_body_Q_ = robotis_framework::convertRPYToQuaternion(0.0, 0.0, angle);

    // Via point
    double via_time = 0.5*(init_time_ + fin_time_);

    std::vector<double_t> via_l_foot_pos, via_l_foot_vel, via_l_foot_accel;
    via_l_foot_pos.resize(3, 0.0);
    via_l_foot_vel.resize(3, 0.0);
    via_l_foot_accel.resize(3, 0.0);

    via_l_foot_pos[0] = 0.5*(init_l_foot_pos_[0] + goal_l_foot_pos_[0]);
    via_l_foot_pos[1] = 0.5*(init_l_foot_pos_[1] + goal_l_foot_pos_[1]);
    via_l_foot_pos[2] = foot_tra_max_z_;

    if (step == 0 || step == 1)
      via_l_foot_pos[2] = 0.0;

    if (step == foot_step_size_-1)
      via_l_foot_pos[2] = 0.0;

    // Trajectory
    l_foot_tra_ = new robotis_framework::MinimumJerkViaPoint(init_time_, fin_time_, via_time, dsp_ratio_,
                                                             init_l_foot_pos_, init_l_foot_vel_, init_l_foot_accel_,
                                                             goal_l_foot_pos_, goal_l_foot_vel_, goal_l_foot_accel_,
                                                             via_l_foot_pos, via_l_foot_vel, via_l_foot_accel);

//    ROS_INFO("angle: %f", angle);
  }
  else if (foot_step_param_.moving_foot[step] == RIGHT_LEG)
  {
    double angle = foot_step_param_.data[step].theta;

    // Goal
    goal_r_foot_pos_[0] = init_l_foot_pos_[0]
        + cos(angle) * foot_step_param_.data[step].x
        + sin(angle) * foot_step_param_.data[step].y;
    goal_r_foot_pos_[1] = init_l_foot_pos_[1]
        + sin(angle) * foot_step_param_.data[step].x
        - cos(angle) * foot_step_param_.data[step].y;
    goal_r_foot_pos_[2] = init_l_foot_pos_[2];

    goal_r_foot_Q_ = robotis_framework::convertRPYToQuaternion(0.0, 0.0, angle);

    goal_l_foot_pos_ = init_l_foot_pos_;
    goal_l_foot_Q_ = init_l_foot_Q_;

    goal_body_Q_ = robotis_framework::convertRPYToQuaternion(0.0, 0.0, angle);

    // Via point
    double via_time = 0.5*(init_time_ + fin_time_);

    std::vector<double_t> via_r_foot_pos, via_r_foot_vel, via_r_foot_accel;
    via_r_foot_pos.resize(3, 0.0);
    via_r_foot_vel.resize(3, 0.0);
    via_r_foot_accel.resize(3, 0.0);

    via_r_foot_pos[0] = 0.5*(init_r_foot_pos_[0] + goal_r_foot_pos_[0]);
    via_r_foot_pos[1] = 0.5*(init_r_foot_pos_[1] + goal_r_foot_pos_[1]);
    via_r_foot_pos[2] = foot_tra_max_z_;

    if (step == 0 || step == 1)
      via_r_foot_pos[2] = 0.0;

    if (step == foot_step_size_-1)
      via_r_foot_pos[2] = 0.0;

    // Trajectory
    r_foot_tra_ = new robotis_framework::MinimumJerkViaPoint(init_time_, fin_time_, via_time, dsp_ratio_,
                                                             init_r_foot_pos_, init_r_foot_vel_, init_r_foot_accel_,
                                                             goal_r_foot_pos_, goal_r_foot_vel_, goal_r_foot_accel_,
                                                             via_r_foot_pos, via_r_foot_vel, via_r_foot_accel);
  }

//  ROS_INFO("-----");
//  ROS_INFO("goal_r_foot_pos_ x: %f , y: %f", goal_r_foot_pos_[0], goal_r_foot_pos_[1]);
//  ROS_INFO("goal_l_foot_pos_ x: %f , y: %f", goal_l_foot_pos_[0], goal_l_foot_pos_[1]);
//  ROS_INFO("-----");
}

void WalkingControl::calcFootStepPose(double time, int step)
{
  if (foot_step_param_.moving_foot[step] == LEFT_LEG)
  {
    des_l_foot_pos_ = l_foot_tra_->getPosition(time);
    des_l_foot_vel_ = l_foot_tra_->getVelocity(time);
    des_l_foot_accel_ = l_foot_tra_->getAcceleration(time);

    des_r_foot_pos_ = goal_r_foot_pos_;
    des_r_foot_vel_.resize(3, 0.0);
    des_r_foot_accel_.resize(3, 0.0);

    walking_leg_ = LEFT_LEG;

//    ROS_INFO("left foot x: %f, y: %f, z: %f", desired_left_foot_pos_[0], desired_left_foot_pos_[1], desired_left_foot_pos_[2]);
  }
  else if (foot_step_param_.moving_foot[step] == RIGHT_LEG)
  {
    des_r_foot_pos_ = r_foot_tra_->getPosition(time);
    des_r_foot_vel_ = r_foot_tra_->getVelocity(time);
    des_r_foot_accel_ = r_foot_tra_->getAcceleration(time);

    des_l_foot_pos_ = goal_l_foot_pos_;
    des_l_foot_vel_.resize(3, 0.0);
    des_l_foot_accel_.resize(3, 0.0);

    walking_leg_ = RIGHT_LEG;

//    ROS_INFO("right foot x: %f, y: %f, z: %f", desired_right_foot_pos_[0], desired_right_foot_pos_[1], desired_right_foot_pos_[2]);
  }
}

void WalkingControl::calcRefZMP(int step)
{
  if (step == 0 || step == 1)
  {
    ref_zmp_x_ = 0.5*(goal_r_foot_pos_[0] + goal_l_foot_pos_[0]); // + zmp_offset_x_;
    ref_zmp_y_ = 0.5*(goal_r_foot_pos_[1] + goal_l_foot_pos_[1]);
  }
  else if (step == foot_step_size_-1)
  {
    ref_zmp_x_ = 0.5*(goal_r_foot_pos_[0] + goal_l_foot_pos_[0]); // + zmp_offset_x_;
    ref_zmp_y_ = 0.5*(goal_r_foot_pos_[1] + goal_l_foot_pos_[1]);
  }
  else
  {
    if (foot_step_param_.moving_foot[step] == LEFT_LEG)
    {
      ref_zmp_x_ = goal_r_foot_pos_[0]; // + zmp_offset_x_;
      ref_zmp_y_ = goal_r_foot_pos_[1] + zmp_offset_y_;
    }
    else if (foot_step_param_.moving_foot[step] == RIGHT_LEG)
    {
      ref_zmp_x_ = goal_l_foot_pos_[0]; // + zmp_offset_x_;
      ref_zmp_y_ = goal_l_foot_pos_[1] - zmp_offset_y_;
    }
  }

//  ROS_INFO("ref zmp x: %f, y: %f", ref_zmp_x_, ref_zmp_y_);
}

void WalkingControl::calcGoalFootPose()
{
  goal_r_foot_pos_buffer_ = Eigen::MatrixXd::Zero(foot_step_size_,2);
  goal_l_foot_pos_buffer_ = Eigen::MatrixXd::Zero(foot_step_size_,2);

  std::vector<double_t> init_r_foot_pos, init_l_foot_pos;
  init_r_foot_pos.resize(2, 0.0);
  init_r_foot_pos[0] = init_r_foot_pos_[0];
  init_r_foot_pos[1] = init_r_foot_pos_[1];

  init_l_foot_pos.resize(2, 0.0);
  init_l_foot_pos[0] = init_l_foot_pos_[0];
  init_l_foot_pos[1] = init_l_foot_pos_[1];

  std::vector<double_t> goal_r_foot_pos, goal_l_foot_pos;
  goal_r_foot_pos.resize(2, 0.0);
  goal_l_foot_pos.resize(2, 0.0);

  for (int step=0; step<foot_step_size_; step++)
  {
    double angle = foot_step_param_.data[step].theta;

    if (foot_step_param_.moving_foot[step] == LEFT_LEG)
    {
      goal_l_foot_pos[0] = init_r_foot_pos[0]
          + cos(angle) * foot_step_param_.data[step].x
          - sin(angle) * foot_step_param_.data[step].y;
      goal_l_foot_pos[1] = init_r_foot_pos[1]
          + sin(angle) * foot_step_param_.data[step].x
          + cos(angle) * foot_step_param_.data[step].y;

      goal_r_foot_pos = init_r_foot_pos;
    }
    else if(foot_step_param_.moving_foot[step] == RIGHT_LEG)
    {
      goal_r_foot_pos[0] = init_l_foot_pos[0]
          + cos(angle) * foot_step_param_.data[step].x
          + sin(angle) * foot_step_param_.data[step].y;
      goal_r_foot_pos[1] = init_l_foot_pos[1]
          + sin(angle) * foot_step_param_.data[step].x
          - cos(angle) * foot_step_param_.data[step].y;

      goal_l_foot_pos = init_l_foot_pos;
    }

    goal_r_foot_pos_buffer_.coeffRef(step,0) = goal_r_foot_pos[0];
    goal_r_foot_pos_buffer_.coeffRef(step,1) = goal_r_foot_pos[1];
    goal_l_foot_pos_buffer_.coeffRef(step,0) = goal_l_foot_pos[0];
    goal_l_foot_pos_buffer_.coeffRef(step,1) = goal_l_foot_pos[1];

    init_r_foot_pos = goal_r_foot_pos;
    init_l_foot_pos = goal_l_foot_pos;
  }
}

double WalkingControl::calcRefZMPx(int step)
{
  double ref_zmp_x;

  if (step == 0 || step == 1)
  {
    ref_zmp_x = 0.5 * (goal_r_foot_pos_buffer_.coeff(step,0) + goal_l_foot_pos_buffer_.coeff(step,0)); // + zmp_offset_x_;
  }
  else if (step >= foot_step_size_-1)
  {
    ref_zmp_x = 0.5 * (goal_r_foot_pos_buffer_.coeff(foot_step_size_-1,0) + goal_l_foot_pos_buffer_.coeff(foot_step_size_-1,0)); // + zmp_offset_x_;
  }
  else
  {
    if (foot_step_param_.moving_foot[step] == LEFT_LEG)
      ref_zmp_x = goal_r_foot_pos_buffer_.coeff(step,0); // + zmp_offset_x_;
    else if (foot_step_param_.moving_foot[step] == RIGHT_LEG)
      ref_zmp_x = goal_l_foot_pos_buffer_.coeff(step,0); // + zmp_offset_x_;
  }

  return ref_zmp_x;
}

double WalkingControl::calcRefZMPy(int step)
{
  double ref_zmp_y;

  if (step == 0 || step == 1)
  {
    ref_zmp_y = 0.5 * (goal_r_foot_pos_buffer_.coeff(step,1) + goal_l_foot_pos_buffer_.coeff(step,1));
  }
  else if (step >= foot_step_size_-1)
  {
    ref_zmp_y = 0.5 * (goal_r_foot_pos_buffer_.coeff(foot_step_size_-1,1) + goal_l_foot_pos_buffer_.coeff(foot_step_size_-1,1));
  }
  else
  {
    if (foot_step_param_.moving_foot[step] == LEFT_LEG)
      ref_zmp_y = goal_r_foot_pos_buffer_.coeff(step,1) + zmp_offset_y_;
    else if (foot_step_param_.moving_foot[step] == RIGHT_LEG)
      ref_zmp_y = goal_l_foot_pos_buffer_.coeff(step,1) - zmp_offset_y_;
  }

  return ref_zmp_y;
}

void WalkingControl::calcPreviewParam(thormang3_wholebody_module_msgs::PreviewResponse msg)
{
  //
//  ROS_INFO("lipm_height_ : %f", lipm_height_);

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

//  ROS_INFO("1: %f , 2: %f , 3: %f",
//           sum_of_cx_,
//           sum_of_zmp_x_,
//           k_s_);

//  ROS_INFO("1: %f , 2: %f , 3: %f",
//           -k_s_*(sum_of_cx_ - sum_of_zmp_x_),
//           -(k_x_(0,0)*x_lipm_(0,0) + k_x_(0,1)*x_lipm_(1,0) + k_x_(0,2)*x_lipm_(2,0)),
//           preview_sum_zmp_x_);

  x_lipm_ = A_*x_lipm_ + b_*u_x_;
  y_lipm_ = A_*y_lipm_ + b_*u_y_;

  des_body_pos_[0] = x_lipm_.coeff(0,0);
  des_body_pos_[1] = y_lipm_.coeff(0,0);

//  ROS_INFO("x_lipm pos: %f, vel: %f, accel:%f", x_lipm_.coeff(0,0), x_lipm_.coeff(1,0), x_lipm_.coeff(2,0));
//  ROS_INFO("y_lipm pos: %f, vel: %f, accel:%f", y_lipm_.coeff(0,0), y_lipm_.coeff(1,0), y_lipm_.coeff(2,0));

//  ROS_INFO("time: %f, desired_body_pos_ x: %f , y: %f", time, des_body_pos_[0], des_body_pos_[1]);
}

void WalkingControl::getWalkingPosition(std::vector<double_t> &l_foot_pos,
                                        std::vector<double_t> &r_foot_pos,
                                        std::vector<double_t> &body_pos)
{
  l_foot_pos = des_l_foot_pos_;
  r_foot_pos = des_r_foot_pos_;
  body_pos   = des_body_pos_;

//  ROS_INFO("body x: %f ,  y: %f", body_pos[0], body_pos[1]);
//  ROS_INFO("l_foot_pos x: %f ,  y: %f , z: %f", l_foot_pos[0], l_foot_pos[1], l_foot_pos[2]);
//  ROS_INFO("r_foot_pos x: %f ,  y: %f , z: %f", r_foot_pos[0], r_foot_pos[1], r_foot_pos[2]);
}

void WalkingControl::getWalkingVelocity(std::vector<double_t> &l_foot_vel,
                                        std::vector<double_t> &r_foot_vel,
                                        std::vector<double_t> &body_vel)
{
  l_foot_vel = des_l_foot_vel_;
  r_foot_vel = des_r_foot_vel_;

  // TODO
  // body_vel =
}

void WalkingControl::getWalkingAccleration(std::vector<double_t> &l_foot_accel,
                                           std::vector<double_t> &r_foot_accel,
                                           std::vector<double_t> &body_accel)
{
  l_foot_accel = des_l_foot_accel_;
  r_foot_accel = des_r_foot_accel_;

  // TODO
  // body_accel =
}

void WalkingControl::getWalkingOrientation(std::vector<double_t> &l_foot_Q,
                                           std::vector<double_t> &r_foot_Q,
                                           std::vector<double_t> &body_Q)
{
  l_foot_Q[0] = des_l_foot_Q_.x();
  l_foot_Q[1] = des_l_foot_Q_.y();
  l_foot_Q[2] = des_l_foot_Q_.z();
  l_foot_Q[3] = des_l_foot_Q_.w();

  r_foot_Q[0] = desired_r_foot_Q_.x();
  r_foot_Q[1] = desired_r_foot_Q_.y();
  r_foot_Q[2] = desired_r_foot_Q_.z();
  r_foot_Q[3] = desired_r_foot_Q_.w();

  body_Q[0] = des_body_Q_.x();
  body_Q[1] = des_body_Q_.y();
  body_Q[2] = des_body_Q_.z();
  body_Q[3] = des_body_Q_.w();
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
