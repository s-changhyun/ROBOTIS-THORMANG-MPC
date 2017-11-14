//#include <yaml-cpp/yaml.h>

#include "thormang3_localization/thormang3_localization.h"

namespace thormang3
{

Thormang3Localization::Thormang3Localization()
 : ros_node_(),
   transform_tolerance_(0.0),
   err_tol_(0.2)
//   is_moving_walking_(false)
{
  initialize();

//  pelvis_pose_base_walking_.pose.position.x = 0.0;
//  pelvis_pose_base_walking_.pose.position.y = 0.0;
//  pelvis_pose_base_walking_.pose.position.z = 0.0;
//  pelvis_pose_base_walking_.pose.orientation.x = 0.0;
//  pelvis_pose_base_walking_.pose.orientation.y = 0.0;
//  pelvis_pose_base_walking_.pose.orientation.z = 0.0;
//  pelvis_pose_base_walking_.pose.orientation.w = 1.0;

  pelvis_pose_offset_.pose.position.x = 0.0;
  pelvis_pose_offset_.pose.position.y = 0.0;
  pelvis_pose_offset_.pose.position.z = 0.7340152;
  pelvis_pose_offset_.pose.orientation.x = 0.0;
  pelvis_pose_offset_.pose.orientation.y = 0.0;
  pelvis_pose_offset_.pose.orientation.z = 0.0;
  pelvis_pose_offset_.pose.orientation.w = 1.0;

  pelvis_pose_old_.pose.position.x = 0.0;
  pelvis_pose_old_.pose.position.y = 0.0;
  pelvis_pose_old_.pose.position.z = 0.0;
  pelvis_pose_old_.pose.orientation.x = 0.0;
  pelvis_pose_old_.pose.orientation.y = 0.0;
  pelvis_pose_old_.pose.orientation.z = 0.0;
  pelvis_pose_old_.pose.orientation.w = 1.0;

  update();
}

Thormang3Localization::~Thormang3Localization()
{

}

void Thormang3Localization::initialize()
{
  // subscriber
  pelvis_offset_msg_sub_ = ros_node_.subscribe("/robotis/pelvis_pose_offset", 5,
                                                         &Thormang3Localization::pelvisPoseOffsetCallback, this);
//  pelvis_base_walking_msg_sub_ = ros_node_.subscribe("/robotis/pelvis_pose_base_walking", 5,
//                                                               &Thormang3Localization::pelvisPoseBaseWalkingCallback, this);

  pelvis_reset_msg_sub_ = ros_node_.subscribe("/robotis/pelvis_pose_reset", 5,
                                                       &Thormang3Localization::pelvisPoseResetCallback, this);

}

void Thormang3Localization::pelvisPoseOffsetCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  mutex_.lock();

  pelvis_pose_offset_ = *msg;
  pelvis_pose_.header.stamp = pelvis_pose_offset_.header.stamp;

  mutex_.unlock();
}

//void Thormang3Localization::pelvisPoseBaseWalkingCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
//{
//  mutex_.lock();

//  Eigen::Quaterniond msg_q;
//  tf::quaternionMsgToEigen(msg->pose.orientation, msg_q);

//  Eigen::MatrixXd rpy = robotis_framework::convertQuaternionToRPY(msg_q);
//  double yaw = rpy.coeff(2,0);

//  if ( fabs(msg->pose.position.x) <= 1e-3 &&
//       fabs(msg->pose.position.y) <= 1e-3 &&
//       fabs(yaw) <= 1e-3 )
//  {
//    if (is_moving_walking_ == true)
//    {
//      ROS_INFO("Walking Pelvis Pose Update");
//      pelvis_pose_old_.pose.position.x += pelvis_pose_base_walking_new_.pose.position.x;
//      pelvis_pose_old_.pose.position.y += pelvis_pose_base_walking_new_.pose.position.y;

//      Eigen::Quaterniond pose_old_quaternion(pelvis_pose_old_.pose.orientation.w,
//                                             pelvis_pose_old_.pose.orientation.x,
//                                             pelvis_pose_old_.pose.orientation.y,
//                                             pelvis_pose_old_.pose.orientation.z);

//      Eigen::Quaterniond pose_base_quaternion(pelvis_pose_base_walking_.pose.orientation.w,
//                                              pelvis_pose_base_walking_.pose.orientation.x,
//                                              pelvis_pose_base_walking_.pose.orientation.y,
//                                              pelvis_pose_base_walking_.pose.orientation.z);

//      Eigen::Quaterniond q = pose_old_quaternion * pose_base_quaternion;
//      tf::quaternionEigenToMsg(q, pelvis_pose_old_.pose.orientation);

//      is_moving_walking_ = false;
//    }
//  }
//  else
//  {
//    is_moving_walking_ = true;
//  }

//  pelvis_pose_base_walking_ = *msg;
//  pelvis_pose_.header.stamp = pelvis_pose_base_walking_.header.stamp;

//  mutex_.unlock();
//}

void Thormang3Localization::pelvisPoseResetCallback(const std_msgs::String::ConstPtr& msg)
{
  if (msg->data == "reset")
  {
    ROS_INFO("Pelvis Pose Reset");

    pelvis_pose_old_.pose.position.x = 0.0;
    pelvis_pose_old_.pose.position.y = 0.0;
    pelvis_pose_old_.pose.orientation.x = 0.0;
    pelvis_pose_old_.pose.orientation.y = 0.0;
    pelvis_pose_old_.pose.orientation.z = 0.0;
    pelvis_pose_old_.pose.orientation.w = 1.0;
  }
}

void Thormang3Localization::process()
{
  update();

  pelvis_trans_.setOrigin(tf::Vector3(pelvis_pose_.pose.position.x,
                                      pelvis_pose_.pose.position.y,
                                      pelvis_pose_.pose.position.z)
                          );

  tf::Quaternion q(pelvis_pose_.pose.orientation.x,
                   pelvis_pose_.pose.orientation.y,
                   pelvis_pose_.pose.orientation.z,
                   pelvis_pose_.pose.orientation.w);

  pelvis_trans_.setRotation(q);

  ros::Duration transform_tolerance(transform_tolerance_);
  ros::Time transform_expiration = (pelvis_pose_.header.stamp + transform_tolerance);

  tf::StampedTransform tmp_tf_stamped(pelvis_trans_, transform_expiration, "world", "pelvis_link");

  broadcaster_.sendTransform(tmp_tf_stamped);
}

void Thormang3Localization::update()
{
  mutex_.lock();

  Eigen::Quaterniond pose_old_quaternion(pelvis_pose_old_.pose.orientation.w,
                                         pelvis_pose_old_.pose.orientation.x,
                                         pelvis_pose_old_.pose.orientation.y,
                                         pelvis_pose_old_.pose.orientation.z);

//  Eigen::Quaterniond pose_base_walking_quaternion(pelvis_pose_base_walking_.pose.orientation.w,
//                                                  pelvis_pose_base_walking_.pose.orientation.x,
//                                                  pelvis_pose_base_walking_.pose.orientation.y,
//                                                  pelvis_pose_base_walking_.pose.orientation.z);

  Eigen::Quaterniond pose_offset_quaternion(pelvis_pose_offset_.pose.orientation.w,
                                            pelvis_pose_offset_.pose.orientation.x,
                                            pelvis_pose_offset_.pose.orientation.y,
                                            pelvis_pose_offset_.pose.orientation.z);

  Eigen::Quaterniond pose_quaternion =
      pose_old_quaternion *
//      pose_base_walking_quaternion *
      pose_offset_quaternion;

//  Eigen::MatrixXd position_walking = Eigen::MatrixXd::Zero(3,1);
//  position_walking.coeffRef(0,0) =
//      pelvis_pose_base_walking_.pose.position.x;
//  position_walking.coeffRef(1,0) =
//      pelvis_pose_base_walking_.pose.position.y;

  Eigen::MatrixXd position_offset = Eigen::MatrixXd::Zero(3,1);
  position_offset.coeffRef(0,0) =
      pelvis_pose_offset_.pose.position.x;
  position_offset.coeffRef(1,0) =
      pelvis_pose_offset_.pose.position.y;

  Eigen::MatrixXd orientation = robotis_framework::convertQuaternionToRotation(pose_old_quaternion);
//  Eigen::MatrixXd position_walking_new = orientation * position_walking;
  Eigen::MatrixXd position_offset_new = orientation * position_offset;

//  pelvis_pose_base_walking_new_.pose.position.x = position_walking_new.coeff(0,0);
//  pelvis_pose_base_walking_new_.pose.position.y = position_walking_new.coeff(1,0);

  pelvis_pose_offset_new_.pose.position.x = position_offset_new.coeff(0,0);
  pelvis_pose_offset_new_.pose.position.y = position_offset_new.coeff(1,0);

  pelvis_pose_.pose.position.x =
      pelvis_pose_old_.pose.position.x +
//      pelvis_pose_base_walking_new_.pose.position.x +
      pelvis_pose_offset_new_.pose.position.x;

  pelvis_pose_.pose.position.y =
      pelvis_pose_old_.pose.position.y +
//      pelvis_pose_base_walking_new_.pose.position.y +
      pelvis_pose_offset_new_.pose.position.y;

  pelvis_pose_.pose.position.z =
      pelvis_pose_offset_.pose.position.z;

  tf::quaternionEigenToMsg(pose_quaternion, pelvis_pose_.pose.orientation);

  mutex_.unlock();
}

} // namespace thormang3
