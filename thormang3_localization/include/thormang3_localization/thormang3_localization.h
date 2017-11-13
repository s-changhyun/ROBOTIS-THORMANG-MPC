#ifndef THORMANG3_LOCALIZATION_H_
#define THORMANG3_LOCALIZATION_H_

//std
#include <string>

//ros dependencies
#include <ros/ros.h>

// ros msg, srv
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <eigen_conversions/eigen_msg.h>

// eigen
#include <Eigen/Dense>

// boost
#include <boost/thread.hpp>

#include "robotis_math/robotis_math.h"

namespace thormang3
{

class Thormang3Localization
{

private:
    //ros node handle
    ros::NodeHandle ros_node_;

    //subscriber
    ros::Subscriber thormang3_pelvis_offset_msg_sub_;
    ros::Subscriber thormang3_pelvis_base_walking_msg_sub_;
    ros::Subscriber thormang3_pelvis_reset_msg_sub_;

    tf::TransformBroadcaster broadcaster_;
    tf::StampedTransform pelvis_trans_;

    geometry_msgs::PoseStamped thormang3_pelvis_pose_;
    geometry_msgs::PoseStamped thormang3_pelvis_pose_old_;
    geometry_msgs::PoseStamped thormang3_pelvis_pose_base_walking_;
    geometry_msgs::PoseStamped thormang3_pelvis_pose_offset_;

    geometry_msgs::PoseStamped thormang3_pelvis_pose_base_walking_new_;
    geometry_msgs::PoseStamped thormang3_pelvis_pose_offset_new_;

    double transform_tolerance_;
    double err_tol_;

    bool is_moving_walking_;

    boost::mutex mutex_;

public:
    void initialize();
    void thormang3PelvisPoseOffsetCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void thormang3PelvisPoseBaseWalkingCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void thormang3PelvisPoseResetCallback(const std_msgs::String::ConstPtr& msg);
    Eigen::MatrixXd calcVWerr(Eigen::MatrixXd tar_position, Eigen::MatrixXd curr_position,
                              Eigen::MatrixXd tar_orientation, Eigen::MatrixXd curr_orientation);

    //constructor
    Thormang3Localization();
    //destructor
    ~Thormang3Localization();

    void update();
    void process();

};

}       // namespace

#endif  // THORMANG3_LOCALIZATION_H_
