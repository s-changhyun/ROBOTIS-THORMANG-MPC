/*
 * ThorManipulation.h
 *
 *  Created on: 2016. 1. 18.
 *      Author: zerom
 */

#ifndef MANIPULATIONMODULE_H_
#define MANIPULATIONMODULE_H_

#include <map>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/package.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <boost/thread.hpp>
#include <yaml-cpp/yaml.h>

#define EIGEN_NO_DEBUG
#define EIGEN_NO_STATIC_ASSERT

#include "robotis_framework_common/MotionModule.h"

#include "RobotisCommon.h"
#include "RobotisData.h"
#include "RobotisLink.h"
#include "RobotisState.h"
#include "JointState.h"
#include "Transformation.h"
#include "Trajectory.h"

#include "thormang3_manipulation_module_msgs/JointPose.h"
#include "thormang3_manipulation_module_msgs/KinematicsPose.h"
#include "thormang3_manipulation_module_msgs/DemoPose.h"

#include "thormang3_manipulation_module_msgs/GetJointPose.h"
#include "thormang3_manipulation_module_msgs/GetKinematicsPose.h"

namespace ROBOTIS
{

//using namespace ROBOTIS_MANIPULATION;

class ManipulationModule : public MotionModule
{
private:
    static ManipulationModule *unique_instance_;

    int                 control_cycle_msec_;
    boost::thread       queue_thread_;
    boost::thread*       tra_gene_tread_;

    ros::Publisher      send_tra_pub_;

    std::map<std::string, int> joint_name_to_id;

    ManipulationModule();

    void QueueThread();

    void parseData( const std::string &path );
    void parseIniPoseData( const std::string &path );

public:
    virtual ~ManipulationModule();

    static ManipulationModule *GetInstance() { return unique_instance_; }

    /* ROS Topic Callback Functions */
    void    IniPoseMsgCallback( const std_msgs::String::ConstPtr& msg );
    void    JointDesMsgCallback( const thormang3_manipulation_module_msgs::JointPose::ConstPtr& msg );
    void    IkMsgCallback( const thormang3_manipulation_module_msgs::KinematicsPose::ConstPtr& msg );

    void    DemoMsgCallback( const thormang3_manipulation_module_msgs::DemoPose::ConstPtr& msg );
    void    BiManualCallback( const std_msgs::String::ConstPtr& msg );

    bool    GetJointPoseCallback( thormang3_manipulation_module_msgs::GetJointPose::Request &req , thormang3_manipulation_module_msgs::GetJointPose::Response &res );
    bool    GetKinematicsPoseCallback( thormang3_manipulation_module_msgs::GetKinematicsPose::Request &req , thormang3_manipulation_module_msgs::GetKinematicsPose::Response &res );

    /* ROS Calculation Functions */
    void    IniposeTraGeneProc( );
    void    JointTraGeneProc( );
    void    TaskTraGeneProc( );

    void    DemoIniTraGeneProc( );
    void    DemoLineTraGeneProc( );
    void    DemoCircleTraGeneProc( );

    void    BiManualIniTraGeneProc( );
    void    BiManualDemoTraGeneProc( );

    /* ROS Framework Functions */
    void    Initialize(const int control_cycle_msec);
    void    Process(std::map<std::string, Dynamixel *> dxls);

    /* Parameter */
    ROBOTIS_MANIPULATION::RobotisData *Humanoid;
    ROBOTIS_MANIPULATION::RobotisState *Param;

};

}


#endif /* MANIPULATIONMODULE_H_ */
