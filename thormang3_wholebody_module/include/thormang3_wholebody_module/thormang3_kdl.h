
#ifndef THORMANG3_WHOLEBODY_MODULE_THORMANG3_KDL_
#define THORMANG3_WHOLEBODY_MODULE_THORMANG3_KDL_

#pragma once

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/package.h>

#include <math.h>
#include <stdint.h>
#include <string>
#include <vector>
#include <map>

#include <geometry_msgs/Pose.h>

#include <kdl/joint.hpp>
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>

#include <Eigen/Dense>

#define LEG_JOINT_NUM   (6)
#define D2R             (M_PI/180.0)

class Thormang3Kinematics
{
public:
  Thormang3Kinematics();
  virtual ~Thormang3Kinematics();

//  void initialize(std::vector<double_t> pelvis_position, std::vector<double_t> pelvis_orientation);
  void initialize(Eigen::MatrixXd pelvis_position, Eigen::MatrixXd pelvis_orientation);
  void setJointPosition(Eigen::VectorXd rleg_joint_position, Eigen::VectorXd lleg_joint_position);
  void solveForwardKinematics(std::vector<double_t> &rleg_position, std::vector<double_t> &rleg_orientation,
                              std::vector<double_t> &lleg_position, std::vector<double_t> &lleg_orientation);
  bool solveInverseKinematics(std::vector<double_t> &rleg_output,
                              Eigen::MatrixXd rleg_target_position, Eigen::Quaterniond rleg_target_orientation,
                              std::vector<double_t> &lleg_output,
                              Eigen::MatrixXd lleg_target_position, Eigen::Quaterniond lleg_target_orientation);
  void finalize();

protected:
//  KDL::Chain rleg_chain_;
  KDL::ChainDynParam *rleg_dyn_param_ = NULL;
  KDL::ChainJntToJacSolver *rleg_jacobian_solver_;
  KDL::ChainFkSolverPos_recursive *rleg_fk_solver_;
  KDL::ChainIkSolverVel_pinv *rleg_ik_vel_solver_;
  KDL::ChainIkSolverPos_NR_JL *rleg_ik_pos_solver_;

//  KDL::Chain lleg_chain_;
  KDL::ChainDynParam *lleg_dyn_param_ = NULL;
  KDL::ChainJntToJacSolver *lleg_jacobian_solver_;
  KDL::ChainFkSolverPos_recursive *lleg_fk_solver_;
  KDL::ChainIkSolverVel_pinv *lleg_ik_vel_solver_;
  KDL::ChainIkSolverPos_NR_JL *lleg_ik_pos_solver_;

  Eigen::VectorXd rleg_joint_position_, lleg_joint_position_;
  geometry_msgs::Pose rleg_pose_, lleg_pose_;



};

#endif
