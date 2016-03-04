/*
 * jointstate.h
 *
 *  Created on: Jul 10, 2015
 *      Author: sch
 */

#ifndef MANIPULATION_MODULE_JOINTSTATE_H_
#define MANIPULATION_MODULE_JOINTSTATE_H_

namespace ROBOTIS_MANIPULATION
{

class JointData
{
public:
    double position;
    double velocity;
    double effort;

    int p_gain;
    int i_gain;
    int d_gain;
};

class JointState
{
public:
    static JointData curr_joint_state[MAX_JOINT_ID + 1];
    static JointData goal_joint_state[MAX_JOINT_ID + 1];
    static JointData fake_joint_state[MAX_JOINT_ID + 1];
};

}

#endif /* MANIPULATION_MODULE_JOINTSTATE_H_ */
